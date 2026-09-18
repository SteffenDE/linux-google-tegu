// SPDX-License-Identifier: GPL-2.0-only
/*
 * DisplayPort transmitter for the Google Tensor G4 (zumapro).
 *
 * The link block is Samsung's own, and it sits in the USB power domain rather
 * than with the rest of the display pipeline, because the lanes it drives come
 * off the USB-C combo PHY and are shared with USB3. This driver owns the link;
 * the lanes, their crossbar and the AUX analog front end belong to that PHY and
 * are reached through the generic PHY interface.
 *
 * There is no hotplug pin on a USB-C connector. Hotplug arrives as an alternate
 * mode attention over the configuration channel and is forced into the link by
 * software, which is why the block's HPD is driven rather than read.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/units.h>
#include <linux/usb/typec_mux.h>
#include <linux/hdmi.h>
#include <linux/math64.h>

#include <drm/display/drm_dp_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>

/* System registers */
#define ZUMAPRO_DP_SYSTEM_SW_RESET_CONTROL	0x0004
#define SW_RESET				BIT(0)

#define ZUMAPRO_DP_SYSTEM_CLK_CONTROL		0x0008
#define GFCLKMUX_SEL_10				BIT(5)
#define GFCLKMUX_SEL_20				BIT(4)

#define ZUMAPRO_DP_SYSTEM_OSCLK_QCH_FUNC_EN	0x000c
#define OSCCLK_QCH_FUNC_EN			BIT(0)

#define ZUMAPRO_DP_SYSTEM_MAIN_LINK_LANE_COUNT	0x0010
#define LANE_COUNT				GENMASK(2, 0)

#define ZUMAPRO_DP_SYSTEM_PLL_LOCK_CONTROL	0x002c
#define PLL_LOCK_STATUS				BIT(4)

#define ZUMAPRO_DP_SYSTEM_SW_FUNCTION_ENABLE	0x0014
#define SW_FUNC_EN				BIT(0)

#define ZUMAPRO_DP_SYSTEM_SST1_FUNCTION_ENABLE	0x001c
#define SST1_LH_PWR_ON_STATUS			BIT(5)
#define SST1_LH_PWR_ON				BIT(4)
#define SST1_VIDEO_FUNC_EN			BIT(0)

#define ZUMAPRO_DP_SYSTEM_COMMON_FUNCTION_ENABLE 0x0018
#define HDCP22_FUNC_EN				BIT(4)
#define HDCP13_FUNC_EN				BIT(3)
#define GTC_FUNC_EN				BIT(2)
#define PCS_FUNC_EN				BIT(1)
#define AUX_FUNC_EN				BIT(0)

/*
 * Every timer in the block counts oscillator cycles, and each of these holds a
 * different count derived from the oscillator's rate -- not the rate itself.
 * The expressions are the vendor's, and they take that rate in MHz.
 */
#define ZUMAPRO_DP_OSC_CLK_DIV_HPD		0x0050
#define HPD_EVENT_CLK_COUNT			GENMASK(18, 0)

#define ZUMAPRO_DP_OSC_CLK_DIV_HDCP_10US	0x0054
#define I2C_GEN10US_TIMER			GENMASK(11, 0)

#define ZUMAPRO_DP_OSC_CLK_DIV_GTC_1MS		0x0058
#define GTC_1MS_OSC_CLK_COUNT			GENMASK(17, 0)

#define ZUMAPRO_DP_OSC_CLK_DIV_AUX_1US		0x005c
#define AUX_1US_OSC_CLK_COUNT			GENMASK(7, 0)

#define ZUMAPRO_DP_OSC_CLK_DIV_AUX_MAN_UI	0x0060
#define AUX_MAN_UI_OSC_CLK_COUNT		GENMASK(7, 0)

#define ZUMAPRO_DP_OSC_CLK_DIV_AUX_10US		0x0064
#define AUX_10US_OSC_CLK_COUNT			GENMASK(11, 0)

#define ZUMAPRO_DP_SYSTEM_IRQ_COMMON_STATUS	0x0108
#define ZUMAPRO_DP_SYSTEM_IRQ_COMMON_STATUS_MASK 0x010c

/* AUX channel */
#define ZUMAPRO_DP_AUX_CONTROL			0x1000
#define AUX_POWER_DOWN				BIT(16)
#define AUX_REPLY_TIMER_MODE			GENMASK(13, 12)
#define AUX_REPLY_TIMER_MODE_1800US		3
#define AUX_RETRY_TIMER				GENMASK(10, 8)
#define AUX_PN_INV				BIT(1)
#define AUX_REG_MODE_MANCHESTER			BIT(0)

#define ZUMAPRO_DP_AUX_TRANSACTION_START	0x1004
#define AUX_TRAN_START				BIT(0)

#define ZUMAPRO_DP_AUX_BUFFER_CLEAR		0x1008
#define AUX_BUF_CLR				BIT(0)

#define ZUMAPRO_DP_AUX_ADDR_ONLY_COMMAND	0x100c
#define AUX_ADDR_ONLY_CMD			BIT(0)

#define ZUMAPRO_DP_AUX_REQUEST_CONTROL		0x1010
#define AUX_REQ_COMM				GENMASK(31, 28)
#define AUX_REQ_ADDR				GENMASK(27, 8)
#define AUX_REQ_LENGTH				GENMASK(5, 0)

#define ZUMAPRO_DP_AUX_COMMAND_CONTROL		0x1014
#define AUX_DEFER_CTRL_EN			BIT(8)
#define AUX_DEFER_COUNT				GENMASK(6, 0)

#define ZUMAPRO_DP_AUX_MONITOR_1		0x1018
#define AUX_BUF_DATA_COUNT			GENMASK(30, 24)
#define AUX_CMD_STATUS				GENMASK(11, 8)
#define AUX_CMD_STATUS_OK			0
#define AUX_CMD_STATUS_TIMEOUT			2
#define AUX_RX_COMM				GENMASK(7, 4)

#define ZUMAPRO_DP_AUX_MONITOR_2		0x101c

/* Four bytes to a register, least significant byte first. */
#define ZUMAPRO_DP_AUX_TX_DATA(_i)		(0x1030 + ((_i) / 4) * 4)
#define ZUMAPRO_DP_AUX_RX_DATA(_i)		(0x1040 + ((_i) / 4) * 4)
#define AUX_DATA_SHIFT(_i)			(((_i) % 4) * 8)

/* One transfer moves at most this many bytes, which is the DP AUX maximum. */
#define ZUMAPRO_DP_AUX_MAX_BYTES		16

struct zumapro_dp {
	struct device *dev;
	void __iomem *regs;
	struct clk_bulk_data *clks;
	int num_clks;
	struct phy *phy;
	struct drm_dp_aux aux;
	struct drm_bridge bridge;
	unsigned int osc_mhz;
	/* Serialises AUX against the connector telling us the sink changed */
	struct mutex lock;
	bool sink_present;
	struct typec_switch_dev *orien_sw;
	enum typec_orientation orientation;
	u8 dpcd[DP_RECEIVER_CAP_SIZE];
	unsigned int link_rate;
	unsigned int link_lanes;
	bool enhanced_framing;
};

static inline struct zumapro_dp *bridge_to_dp(struct drm_bridge *bridge)
{
	return container_of(bridge, struct zumapro_dp, bridge);
}

static void zumapro_dp_update(struct zumapro_dp *dp, u32 offset, u32 mask,
			      u32 val)
{
	u32 reg = readl(dp->regs + offset);

	reg &= ~mask;
	reg |= val & mask;
	writel(reg, dp->regs + offset);
}

/*
 * The video function is what runs the single-stream block. The vendor enables
 * it at link-up, before a link is even trained, so that everything later
 * written about a stream lands in a block that is running -- and so the
 * timing generator is switched on inside a running block, rather than starting
 * from wherever the function's release happens to find it.
 */
static void zumapro_dp_video_func_enable(struct zumapro_dp *dp)
{
	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_SST1_FUNCTION_ENABLE,
			  SST1_VIDEO_FUNC_EN, SST1_VIDEO_FUNC_EN);
}

/*
 * The reset bit does not clear itself. Writing it and polling for it to go
 * away leaves the block held in reset with everything after it reading back
 * as though it had never been programmed.
 */
static int zumapro_dp_reset(struct zumapro_dp *dp)
{
	u32 reg;

	writel(SW_RESET, dp->regs + ZUMAPRO_DP_SYSTEM_SW_RESET_CONTROL);
	fsleep(1);
	writel(0, dp->regs + ZUMAPRO_DP_SYSTEM_SW_RESET_CONTROL);

	return readl_poll_timeout(dp->regs + ZUMAPRO_DP_SYSTEM_SW_RESET_CONTROL,
				  reg, !(reg & SW_RESET), 10, 2000);
}

/*
 * The AUX unit interval, its microsecond and ten-microsecond ticks, the hotplug
 * deglitch and the millisecond tick are each a count of oscillator cycles, so
 * each is a different function of the rate the clock is actually running at.
 * Getting them from an assumed rate rather than a measured one is how a link
 * ends up unable to read a byte of DPCD with nothing obviously wrong.
 */
static void zumapro_dp_set_osc_divs(struct zumapro_dp *dp)
{
	unsigned int mhz = dp->osc_mhz;

	zumapro_dp_update(dp, ZUMAPRO_DP_OSC_CLK_DIV_HPD, HPD_EVENT_CLK_COUNT,
			  FIELD_PREP(HPD_EVENT_CLK_COUNT, mhz * 2000));
	zumapro_dp_update(dp, ZUMAPRO_DP_OSC_CLK_DIV_HDCP_10US,
			  I2C_GEN10US_TIMER,
			  FIELD_PREP(I2C_GEN10US_TIMER, mhz * 10 - 1));
	zumapro_dp_update(dp, ZUMAPRO_DP_OSC_CLK_DIV_GTC_1MS,
			  GTC_1MS_OSC_CLK_COUNT,
			  FIELD_PREP(GTC_1MS_OSC_CLK_COUNT, mhz * 1000 - 1));
	zumapro_dp_update(dp, ZUMAPRO_DP_OSC_CLK_DIV_AUX_1US,
			  AUX_1US_OSC_CLK_COUNT,
			  FIELD_PREP(AUX_1US_OSC_CLK_COUNT, mhz - 1));
	zumapro_dp_update(dp, ZUMAPRO_DP_OSC_CLK_DIV_AUX_MAN_UI,
			  AUX_MAN_UI_OSC_CLK_COUNT,
			  FIELD_PREP(AUX_MAN_UI_OSC_CLK_COUNT,
				     (mhz * 5) / 10 - 1));
	zumapro_dp_update(dp, ZUMAPRO_DP_OSC_CLK_DIV_AUX_10US,
			  AUX_10US_OSC_CLK_COUNT,
			  FIELD_PREP(AUX_10US_OSC_CLK_COUNT, mhz * 10));
}

/*
 * AUX rides the connector's SBU pair, so which of the two wires carries the
 * positive half depends on which way up the plug went in. Some parts correct
 * that in the crossbar the pins pass through; the one on this board has a
 * single DisplayPort setting and no polarity to it, so the link has to invert
 * instead. Getting it wrong is not subtle and not gradual: the sink answers
 * nothing at all, and half of all cable insertions are the wrong way up.
 */
static void zumapro_dp_aux_init(struct zumapro_dp *dp)
{
	bool flipped = dp->orientation == TYPEC_ORIENTATION_REVERSE;

	zumapro_dp_update(dp, ZUMAPRO_DP_AUX_CONTROL,
			  AUX_POWER_DOWN | AUX_REPLY_TIMER_MODE | AUX_PN_INV |
			  AUX_REG_MODE_MANCHESTER,
			  FIELD_PREP(AUX_REPLY_TIMER_MODE,
				     AUX_REPLY_TIMER_MODE_1800US) |
			  (flipped ? AUX_PN_INV : 0));
}

static void zumapro_dp_switch_unregister(void *sw)
{
	typec_switch_unregister(sw);
}

static int zumapro_dp_orientation_set(struct typec_switch_dev *sw,
				      enum typec_orientation orientation)
{
	struct zumapro_dp *dp = typec_switch_get_drvdata(sw);

	guard(mutex)(&dp->lock);

	if (dp->orientation == orientation)
		return 0;

	dp->orientation = orientation;

	/*
	 * A live AUX has to be told. The connector settles the orientation
	 * before it reports a sink, so in practice this arrives first, but
	 * nothing in the ordering guarantees it.
	 */
	if (dp->sink_present)
		zumapro_dp_aux_init(dp);

	return 0;
}

/*
 * Everything the block does with time is counted from this one clock, so a rate
 * it has not been told about is a link that cannot keep time -- which shows up
 * as AUX transactions a sink never answers rather than as anything obviously
 * clock-shaped.
 */
static int zumapro_dp_read_osc_rate(struct zumapro_dp *dp)
{
	unsigned long rate;
	struct clk *dposc;

	dposc = clk_get(dp->dev, "dposc");
	if (IS_ERR(dposc))
		return PTR_ERR(dposc);
	rate = clk_get_rate(dposc);
	clk_put(dposc);

	dp->osc_mhz = DIV_ROUND_CLOSEST(rate, HZ_PER_MHZ);
	if (!dp->osc_mhz) {
		dev_err(dp->dev, "the oscillator reads %lu Hz\n", rate);
		return -EINVAL;
	}

	/*
	 * The vendor specifies 40 MHz. Another rate is not fatal, because the
	 * counters are derived from whatever it is, but it means the device
	 * tree default did not take and the margins are not the board's.
	 */
	if (dp->osc_mhz != 40)
		dev_warn(dp->dev,
			 "oscillator at %u MHz, not the specified 40\n",
			 dp->osc_mhz);

	return 0;
}

static int zumapro_dp_link_init(struct zumapro_dp *dp)
{
	int ret;

	ret = zumapro_dp_reset(dp);
	if (ret) {
		dev_err(dp->dev, "the link did not come out of reset\n");
		return ret;
	}

	/* The block's own oscillator gate, through its Q-channel. */
	writel(OSCCLK_QCH_FUNC_EN,
	       dp->regs + ZUMAPRO_DP_SYSTEM_OSCLK_QCH_FUNC_EN);

	zumapro_dp_set_osc_divs(dp);

	/*
	 * Run the block from its oscillator. The other input is the PHY's
	 * transmit clock, which does not exist until a link has been trained,
	 * so anything enabled before that has to be clocked from here.
	 */
	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_CLK_CONTROL,
			  GFCLKMUX_SEL_10 | GFCLKMUX_SEL_20, 0);

	/* Nothing has asked for an interrupt yet; leave them all masked. */
	writel(0, dp->regs + ZUMAPRO_DP_SYSTEM_IRQ_COMMON_STATUS_MASK);
	writel(~0, dp->regs + ZUMAPRO_DP_SYSTEM_IRQ_COMMON_STATUS);

	/*
	 * Only AUX. The coding sublayer has nothing to do until there is a
	 * link to train, and enabling it while the lanes are still USB3's
	 * would clock it from a transmit clock that is not running.
	 */
	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_COMMON_FUNCTION_ENABLE,
			  HDCP22_FUNC_EN | HDCP13_FUNC_EN | GTC_FUNC_EN |
			  PCS_FUNC_EN | AUX_FUNC_EN,
			  AUX_FUNC_EN);

	zumapro_dp_aux_init(dp);

	writel(SW_FUNC_EN, dp->regs + ZUMAPRO_DP_SYSTEM_SW_FUNCTION_ENABLE);

	return 0;
}

static int zumapro_dp_aux_wait(struct zumapro_dp *dp)
{
	u32 mon1, mon2, reg;
	int ret;

	writel(AUX_TRAN_START, dp->regs + ZUMAPRO_DP_AUX_TRANSACTION_START);

	ret = readl_poll_timeout(dp->regs + ZUMAPRO_DP_AUX_TRANSACTION_START,
				 reg, !(reg & AUX_TRAN_START), 10, 50000);
	if (ret) {
		dev_err(dp->dev, "AUX transaction never finished\n");
		return ret;
	}

	mon1 = readl(dp->regs + ZUMAPRO_DP_AUX_MONITOR_1);
	mon2 = readl(dp->regs + ZUMAPRO_DP_AUX_MONITOR_2);
	if (FIELD_GET(AUX_CMD_STATUS, mon1) == AUX_CMD_STATUS_OK && !mon2)
		return 0;

	/*
	 * A sink that simply did not answer inside the reply window needs
	 * longer before being asked again than one that answered wrongly.
	 */
	if (FIELD_GET(AUX_CMD_STATUS, mon1) == AUX_CMD_STATUS_TIMEOUT) {
		usleep_range(1400, 1410);
		return -ETIMEDOUT;
	}

	usleep_range(400, 410);

	return -EIO;
}

static ssize_t zumapro_dp_aux_transfer(struct drm_dp_aux *aux,
				       struct drm_dp_aux_msg *msg)
{
	struct zumapro_dp *dp = container_of(aux, struct zumapro_dp, aux);
	u8 request = msg->request & ~DP_AUX_I2C_MOT;
	size_t len = msg->size;
	unsigned int i;
	u32 mon1, reg;
	bool read;
	int ret;

	if (len > ZUMAPRO_DP_AUX_MAX_BYTES)
		return -E2BIG;

	/*
	 * A status update carries no payload in either direction, so it is
	 * neither a read nor a write as far as the data registers go.
	 */
	read = request == DP_AUX_I2C_READ || request == DP_AUX_NATIVE_READ;
	if (request == DP_AUX_I2C_WRITE_STATUS_UPDATE)
		len = 0;

	guard(mutex)(&dp->lock);

	writel(AUX_BUF_CLR, dp->regs + ZUMAPRO_DP_AUX_BUFFER_CLEAR);
	zumapro_dp_update(dp, ZUMAPRO_DP_AUX_COMMAND_CONTROL,
			  AUX_DEFER_CTRL_EN, AUX_DEFER_CTRL_EN);

	/*
	 * An empty message is an address-only transaction, which is how a sink
	 * is probed over the I2C-over-AUX path without moving any data.
	 */
	writel(len ? 0 : AUX_ADDR_ONLY_CMD,
	       dp->regs + ZUMAPRO_DP_AUX_ADDR_ONLY_COMMAND);

	/*
	 * The block encodes the request the same way DisplayPort does, so the
	 * command travels through unchanged. The length field counts from zero,
	 * and is ignored entirely by an address-only transaction.
	 */
	reg = FIELD_PREP(AUX_REQ_COMM, msg->request) |
	      FIELD_PREP(AUX_REQ_ADDR, msg->address);
	if (len)
		reg |= FIELD_PREP(AUX_REQ_LENGTH, len - 1);
	zumapro_dp_update(dp, ZUMAPRO_DP_AUX_REQUEST_CONTROL,
			  AUX_REQ_COMM | AUX_REQ_ADDR | AUX_REQ_LENGTH, reg);

	if (!read)
		for (i = 0; i < len; i++)
			zumapro_dp_update(dp, ZUMAPRO_DP_AUX_TX_DATA(i),
					  0xffU << AUX_DATA_SHIFT(i),
					  (u32)((u8 *)msg->buffer)[i] <<
						  AUX_DATA_SHIFT(i));

	ret = zumapro_dp_aux_wait(dp);

	/*
	 * Whatever the sink answered with is in the monitor, and it is the same
	 * four bits DisplayPort puts on the wire. Hand it back even when the
	 * transaction failed: a native or I2C negative acknowledgement is a
	 * reply, not an error, and the helpers above act on the difference --
	 * they retry a defer and give up on a nack.
	 */
	mon1 = readl(dp->regs + ZUMAPRO_DP_AUX_MONITOR_1);
	msg->reply = FIELD_GET(AUX_RX_COMM, mon1);

	if (ret)
		return ret;

	if (read) {
		len = min_t(size_t, len,
			    FIELD_GET(AUX_BUF_DATA_COUNT, mon1));
		for (i = 0; i < len; i++)
			((u8 *)msg->buffer)[i] =
				readl(dp->regs + ZUMAPRO_DP_AUX_RX_DATA(i)) >>
					AUX_DATA_SHIFT(i);
	}

	return len;
}

/* Physical coding sublayer */
#define ZUMAPRO_DP_PCS_CONTROL			0x3000
#define PCS_LINK_TRAINING_PATTERN		GENMASK(6, 4)
#define PCS_BIT_SWAP				BIT(2)
#define PCS_SCRAMBLE_BYPASS			BIT(0)

/*
 * The block's own numbering for the training patterns, which is not the one
 * DisplayPort puts in the DPCD: pattern 4 is 5 here.
 */
#define PCS_PATTERN_NORMAL_DATA			0
#define PCS_PATTERN_TPS1			1
#define PCS_PATTERN_TPS2			2
#define PCS_PATTERN_TPS3			3
#define PCS_PATTERN_TPS4			5

#define ZUMAPRO_DP_PCS_LANE_CONTROL		0x3004
#define PCS_LANE_MAP(_l)			(GENMASK(1, 0) << ((_l) * 4))
#define PCS_LANE_MAP_ALL			(PCS_LANE_MAP(0) | \
						 PCS_LANE_MAP(1) | \
						 PCS_LANE_MAP(2) | \
						 PCS_LANE_MAP(3))

#define ZUMAPRO_DP_PCS_TEST_PATTERN_CONTROL	0x3008
#define PCS_LINK_QUALITY_PATTERN		GENMASK(2, 0)

/*
 * The handshake between this coding sublayer and the Synopsys transmitters in
 * the combo PHY. Without it the lanes stay quiet whatever the PHY is doing,
 * because the generation this register layer came from drove a PHY that needed
 * no such handshake.
 */
#define ZUMAPRO_DP_PCS_SNPS_DATAPATH_CONTROL	0x3200
#define SNPS_TX_DATA_EN				GENMASK(19, 16)
#define SNPS_TX_CLK_RDY				GENMASK(11, 8)
#define SNPS_TX_CLK_EN				GENMASK(3, 0)

/*
 * Link rates are kept in the unit the DisplayPort helpers use, kHz, because
 * most of what touches them is those helpers. The PHY interface counts in
 * Mb/s, so the conversion happens at that boundary and nowhere else.
 */
static unsigned int zumapro_dp_phy_rate(unsigned int link_rate_khz)
{
	return link_rate_khz / 100;
}

static void zumapro_dp_set_pattern(struct zumapro_dp *dp, unsigned int pattern)
{
	u32 val = FIELD_PREP(PCS_LINK_TRAINING_PATTERN, pattern);

	/*
	 * Scrambling belongs to the pattern rather than to the caller: the
	 * training patterns run unscrambled, real data and pattern 4 scrambled.
	 * Leaving it bypassed for real data gives a sink nothing its
	 * descrambler can recover, which looks like a link that trained and
	 * then showed nothing.
	 */
	if (pattern != PCS_PATTERN_NORMAL_DATA && pattern != PCS_PATTERN_TPS4)
		val |= PCS_SCRAMBLE_BYPASS;

	/*
	 * The bit swap is set on every pattern change and never cleared. It
	 * cannot be seen during training -- those patterns are symmetric under
	 * bit reversal -- only afterwards, on scrambled data.
	 */
	val |= PCS_BIT_SWAP;

	zumapro_dp_update(dp, ZUMAPRO_DP_PCS_TEST_PATTERN_CONTROL,
			  PCS_LINK_QUALITY_PATTERN, 0);
	zumapro_dp_update(dp, ZUMAPRO_DP_PCS_CONTROL,
			  PCS_LINK_TRAINING_PATTERN | PCS_BIT_SWAP |
			  PCS_SCRAMBLE_BYPASS, val);
}

/*
 * Hand the lanes to the transmitters and bring them up. The clocks go first and
 * the data enable last, with the power-up in between, because a transmitter
 * will not acknowledge a power-state request without a clock already running.
 */
static int zumapro_dp_set_data_path(struct zumapro_dp *dp, unsigned int lanes)
{
	struct phy_configure_opts_dp opts = {
		.lanes = lanes,
		.set_lanes = 1,
	};
	u32 mask = GENMASK(lanes - 1, 0);
	int ret;

	zumapro_dp_update(dp, ZUMAPRO_DP_PCS_SNPS_DATAPATH_CONTROL,
			  SNPS_TX_CLK_RDY | SNPS_TX_CLK_EN,
			  FIELD_PREP(SNPS_TX_CLK_RDY, mask) |
			  FIELD_PREP(SNPS_TX_CLK_EN, mask));

	ret = phy_configure(dp->phy, (union phy_configure_opts *)&opts);
	if (ret)
		return ret;

	zumapro_dp_update(dp, ZUMAPRO_DP_PCS_SNPS_DATAPATH_CONTROL,
			  SNPS_TX_DATA_EN, FIELD_PREP(SNPS_TX_DATA_EN, mask));

	return 0;
}

/*
 * Bring the lanes up at a rate: the PHY programmes its own loop, the link
 * waits for it to lock, and only then does the link stop running from its
 * oscillator and start running from the transmit clock. Doing that in the
 * other order clocks the coding sublayer at the oscillator's rate while it is
 * asked to emit multi-gigabit symbols, which locks clock recovery and then
 * fails equalisation with the error counters saturated.
 */
static int zumapro_dp_set_link_rate(struct zumapro_dp *dp, unsigned int rate,
				    unsigned int lanes)
{
	struct phy_configure_opts_dp opts = {
		.link_rate = zumapro_dp_phy_rate(rate),
		.lanes = lanes,
		.set_rate = 1,
	};
	u32 reg;
	int ret;

	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_CLK_CONTROL,
			  GFCLKMUX_SEL_10 | GFCLKMUX_SEL_20, 0);

	ret = phy_configure(dp->phy, (union phy_configure_opts *)&opts);
	if (ret)
		return ret;

	ret = readl_poll_timeout(dp->regs + ZUMAPRO_DP_SYSTEM_PLL_LOCK_CONTROL,
				 reg, reg & PLL_LOCK_STATUS, 10, 2000);
	if (ret) {
		dev_err(dp->dev, "the link PLL did not lock at %u Mb/s\n", rate);
		return ret;
	}

	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_CLK_CONTROL,
			  GFCLKMUX_SEL_10 | GFCLKMUX_SEL_20,
			  GFCLKMUX_SEL_10 | GFCLKMUX_SEL_20);

	/*
	 * Only now can the coding sublayer run: it is clocked from the transmit
	 * clock the mux above has just selected, which does not exist until the
	 * PLL has locked.
	 */
	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_COMMON_FUNCTION_ENABLE,
			  PCS_FUNC_EN, PCS_FUNC_EN);

	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_MAIN_LINK_LANE_COUNT,
			  LANE_COUNT, FIELD_PREP(LANE_COUNT, lanes));

	/* Identity lane map: any reordering is the crossbar's, in the PHY. */
	zumapro_dp_update(dp, ZUMAPRO_DP_PCS_LANE_CONTROL, PCS_LANE_MAP_ALL,
			  FIELD_PREP(PCS_LANE_MAP(0), 0) |
			  FIELD_PREP(PCS_LANE_MAP(1), 1) |
			  FIELD_PREP(PCS_LANE_MAP(2), 2) |
			  FIELD_PREP(PCS_LANE_MAP(3), 3));

	return zumapro_dp_set_data_path(dp, lanes);
}

/*
 * What the sink last asked for, clamped to what the board's table can drive.
 * A level the specification does not allow has no entry, so a request beyond
 * the pair's limit becomes the most that pairing permits, and the sink is told
 * the transmitter has nothing further to give.
 */
static void zumapro_dp_adjust_levels(unsigned int lanes,
				     const u8 link_status[DP_LINK_STATUS_SIZE],
				     u8 train_set[4])
{
	unsigned int i;

	for (i = 0; i < lanes; i++) {
		u8 v = drm_dp_get_adjust_request_voltage(link_status, i) >>
			DP_TRAIN_VOLTAGE_SWING_SHIFT;
		u8 p = drm_dp_get_adjust_request_pre_emphasis(link_status, i) >>
			DP_TRAIN_PRE_EMPHASIS_SHIFT;
		bool clamped = false;

		if (v > 3) {
			v = 3;
			clamped = true;
		}
		if (v + p > 3) {
			p = 3 - v;
			clamped = true;
		}

		train_set[i] = (v << DP_TRAIN_VOLTAGE_SWING_SHIFT) |
			       (p << DP_TRAIN_PRE_EMPHASIS_SHIFT);
		if (v == 3 || clamped)
			train_set[i] |= DP_TRAIN_MAX_SWING_REACHED;
		if (p == 3 || clamped)
			train_set[i] |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;
	}
}

/* Put the levels the sink asked for onto the lanes. */
static int zumapro_dp_set_levels(struct zumapro_dp *dp, unsigned int lanes,
				 const u8 train_set[4])
{
	struct phy_configure_opts_dp opts = {
		.lanes = lanes,
		.set_voltages = 1,
	};
	unsigned int i;

	for (i = 0; i < lanes; i++) {
		opts.voltage[i] = (train_set[i] &
				   DP_TRAIN_VOLTAGE_SWING_MASK) >>
					DP_TRAIN_VOLTAGE_SWING_SHIFT;
		opts.pre[i] = (train_set[i] & DP_TRAIN_PRE_EMPHASIS_MASK) >>
				DP_TRAIN_PRE_EMPHASIS_SHIFT;
	}

	return phy_configure(dp->phy, (union phy_configure_opts *)&opts);
}

/*
 * One pass of clock recovery, then one of channel equalisation, as the
 * specification lays them out. The block is told which pattern to emit and the
 * sink is told the same over DPCD; what comes back is the sink's opinion of the
 * lanes, which decides the drive levels for the next attempt.
 */
static int zumapro_dp_train_pattern(struct zumapro_dp *dp, unsigned int lanes,
				    unsigned int pcs_pattern, u8 dpcd_pattern,
				    bool eq, u8 train_set[4])
{
	unsigned int tries, max_tries = eq ? 5 : 10;
	u8 status[DP_LINK_STATUS_SIZE];
	u8 last_set[4] = { };
	unsigned int same = 0;
	int ret;

	zumapro_dp_set_pattern(dp, pcs_pattern);

	/*
	 * Pattern 4 is a scrambled pattern and the transmitter leaves its
	 * scrambler on for it, so the sink must not be told otherwise. Every
	 * other training pattern runs unscrambled at both ends.
	 */
	if (dpcd_pattern != DP_TRAINING_PATTERN_4)
		dpcd_pattern |= DP_LINK_SCRAMBLING_DISABLE;

	ret = drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET,
				 dpcd_pattern);
	if (ret < 0)
		return ret;

	for (tries = 0; tries < max_tries; tries++) {
		/*
		 * Put the levels on the lanes before telling the sink what to
		 * expect, so the two agree from the first attempt rather than
		 * from the second.
		 */
		ret = zumapro_dp_set_levels(dp, lanes, train_set);
		if (ret)
			return ret;

		ret = drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET,
					train_set, lanes);
		if (ret < 0)
			return ret;

		if (eq)
			drm_dp_link_train_channel_eq_delay(&dp->aux, dp->dpcd);
		else
			drm_dp_link_train_clock_recovery_delay(&dp->aux,
							       dp->dpcd);

		ret = drm_dp_dpcd_read_link_status(&dp->aux, status);
		if (ret < 0)
			return ret;

		if (eq) {
			/*
			 * Losing clock recovery during equalisation is not
			 * something more equalising fixes; the answer is a
			 * slower link.
			 */
			if (!drm_dp_clock_recovery_ok(status, lanes))
				return -EIO;
			if (drm_dp_channel_eq_ok(status, lanes))
				return 0;
		} else {
			if (drm_dp_clock_recovery_ok(status, lanes))
				return 0;

			/*
			 * A sink that has asked for the same levels five times
			 * running is not going to change its mind.
			 */
			if (!memcmp(last_set, train_set, lanes)) {
				if (++same >= 4)
					return -EIO;
			} else {
				same = 0;
			}
			memcpy(last_set, train_set, lanes);

			/* Nor is one already being driven as hard as we can. */
			if (train_set[0] & DP_TRAIN_MAX_SWING_REACHED)
				return -EIO;
		}

		zumapro_dp_adjust_levels(lanes, status, train_set);
	}

	return -EIO;
}

/*
 * Train at one rate and lane count. Everything the sink is told about the link
 * has to match what the transmitter was just set to, so the rate and lane
 * count go out first and the patterns follow.
 */
static int zumapro_dp_train_link(struct zumapro_dp *dp, unsigned int rate,
				 unsigned int lanes)
{
	u8 train_set[4] = { };
	u8 buf[2];
	int ret;

	/*
	 * The PHY knows what the connector negotiated and refuses a lane count
	 * the pin assignment did not leave for DisplayPort, so an attempt at
	 * four lanes on a two-lane contract fails here and falls back rather
	 * than taking lanes USB3 is still using.
	 */
	ret = phy_validate(dp->phy, PHY_MODE_DP, 0,
			   (union phy_configure_opts *)&(struct phy_configure_opts_dp){
				.link_rate = zumapro_dp_phy_rate(rate),
				.lanes = lanes,
				.set_rate = 1,
				.set_lanes = 1,
			   });
	if (ret)
		return ret;

	ret = zumapro_dp_set_link_rate(dp, rate, lanes);
	if (ret)
		return ret;

	/*
	 * Tell the sink what kind of link this is before asking it to train
	 * on one: eight-to-ten bit coding, no spread spectrum -- the PHY is
	 * not asked for any -- and awake, because a sink in a low power state
	 * will not train at all.
	 */
	drm_dp_dpcd_writeb(&dp->aux, DP_MAIN_LINK_CHANNEL_CODING_SET,
			   DP_SET_ANSI_8B10B);
	drm_dp_dpcd_writeb(&dp->aux, DP_DOWNSPREAD_CTRL, 0);

	dp->enhanced_framing = drm_dp_enhanced_frame_cap(dp->dpcd);

	zumapro_dp_video_func_enable(dp);

	buf[0] = drm_dp_link_rate_to_bw_code(rate);
	buf[1] = lanes;
	if (dp->enhanced_framing)
		buf[1] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;
	ret = drm_dp_dpcd_write(&dp->aux, DP_LINK_BW_SET, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	ret = zumapro_dp_train_pattern(dp, lanes, PCS_PATTERN_TPS1,
				       DP_TRAINING_PATTERN_1, false, train_set);
	if (ret)
		return ret;

	/*
	 * Equalise on the best pattern both ends have. Pattern 4 is scrambled,
	 * and the block numbers it differently from the DPCD, so the two
	 * encodings are kept apart.
	 */
	if (drm_dp_tps4_supported(dp->dpcd))
		ret = zumapro_dp_train_pattern(dp, lanes, PCS_PATTERN_TPS4,
					       DP_TRAINING_PATTERN_4, true,
					       train_set);
	else if (drm_dp_tps3_supported(dp->dpcd))
		ret = zumapro_dp_train_pattern(dp, lanes, PCS_PATTERN_TPS3,
					       DP_TRAINING_PATTERN_3, true,
					       train_set);
	else
		ret = zumapro_dp_train_pattern(dp, lanes, PCS_PATTERN_TPS2,
					       DP_TRAINING_PATTERN_2, true,
					       train_set);

	return ret;
}

/*
 * Train, falling back through the rates and lane counts the sink and the board
 * have in common. Whether it succeeds or not the transmitter stops sending a
 * training pattern: leaving one on means the sink has been told to expect real
 * data while the lanes still carry the pattern, which is indistinguishable
 * from a dead link.
 */
static int zumapro_dp_train(struct zumapro_dp *dp)
{
	static const unsigned int rates[] = { 810000, 540000, 270000, 162000 };
	static const unsigned int lane_counts[] = { 4, 2 };
	unsigned int i, j;
	int ret;

	ret = drm_dp_read_dpcd_caps(&dp->aux, dp->dpcd);
	if (ret < 0) {
		/*
		 * The first thing this asks the sink, so the first place a
		 * dead AUX shows up. Say so: without it a sink that cannot be
		 * reached is indistinguishable from one that was never
		 * plugged in, because everything downstream of here is quiet
		 * about a link that was never attempted.
		 */
		dev_err(dp->dev, "the sink did not answer DPCD (%d)\n", ret);
		return ret;
	}

	/* A sink asleep answers DPCD and will not train. */
	drm_dp_dpcd_writeb(&dp->aux, DP_SET_POWER, DP_SET_POWER_D0);
	usleep_range(1000, 2000);

	ret = -EINVAL;
	for (j = 0; j < ARRAY_SIZE(lane_counts); j++) {
		if (lane_counts[j] > drm_dp_max_lane_count(dp->dpcd))
			continue;

		for (i = 0; i < ARRAY_SIZE(rates); i++) {
			if (rates[i] > drm_dp_max_link_rate(dp->dpcd))
				continue;

			ret = zumapro_dp_train_link(dp, rates[i],
						    lane_counts[j]);
			if (!ret) {
				dp->link_rate = rates[i];
				dp->link_lanes = lane_counts[j];
				goto done;
			}
		}
	}

done:
	zumapro_dp_set_pattern(dp, PCS_PATTERN_NORMAL_DATA);
	drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_DISABLE);

	if (ret) {
		dev_err(dp->dev, "the link did not train\n");
		dp->link_rate = 0;
		dp->link_lanes = 0;
	} else {
		dev_info(dp->dev, "link trained: %u kHz on %u lanes\n",
			 dp->link_rate, dp->link_lanes);
	}

	return ret;
}

/* Single-stream transport: the video path proper */
#define ZUMAPRO_DP_SST1_MAIN_CONTROL		0x5000
#define SST1_VIDEO_MODE_SLAVE			BIT(6)
#define SST1_ENHANCED_MODE			BIT(5)

#define ZUMAPRO_DP_SST1_MVID_MASTER_MODE	0x5044
#define ZUMAPRO_DP_SST1_NVID_MASTER_MODE	0x5048
#define ZUMAPRO_DP_SST1_MVID_SFR_CONFIGURE	0x504c
#define ZUMAPRO_DP_SST1_NVID_SFR_CONFIGURE	0x5050
#define MNVID_SFR_CONFIG			GENMASK(23, 0)

#define ZUMAPRO_DP_SST1_ACTIVE_SYMBOL_INTEGER	0x5080
#define ACTIVE_SYMBOL_INTEGER			GENMASK(5, 0)
#define ZUMAPRO_DP_SST1_ACTIVE_SYMBOL_FRACTION	0x5084
#define ACTIVE_SYMBOL_FRACTION			GENMASK(29, 0)
#define ZUMAPRO_DP_SST1_ACTIVE_SYMBOL_THRESHOLD	0x5088
#define ACTIVE_SYMBOL_THRESHOLD			GENMASK(3, 0)
#define ZUMAPRO_DP_SST1_ACTIVE_SYMBOL_THRESHOLD_SEL 0x508c
#define ACTIVE_SYMBOL_THRESHOLD_SEL		BIT(0)

#define ZUMAPRO_DP_SST1_VIDEO_CONTROL		0x5400
#define VIDEO_DYNAMIC_RANGE_CEA			BIT(7)
#define VIDEO_BPC				GENMASK(6, 4)
#define VIDEO_BPC_6				0
#define VIDEO_BPC_8				1
#define VIDEO_BPC_10				2
#define VIDEO_COLOR_FORMAT			GENMASK(3, 2)
#define VIDEO_COLOR_FORMAT_RGB			0
#define VIDEO_VSYNC_POLARITY			BIT(1)
#define VIDEO_HSYNC_POLARITY			BIT(0)

#define ZUMAPRO_DP_SST1_VIDEO_ENABLE		0x5404
#define VIDEO_EN				BIT(0)

#define ZUMAPRO_DP_SST1_VIDEO_MASTER_TIMING_GEN	0x5408
#define VIDEO_MASTER_TIMING_GEN			BIT(0)

#define ZUMAPRO_DP_SST1_VIDEO_HTOTAL		0x5414
#define ZUMAPRO_DP_SST1_VIDEO_VTOTAL		0x5418
#define ZUMAPRO_DP_SST1_VIDEO_HFP		0x541c
#define ZUMAPRO_DP_SST1_VIDEO_HBP		0x5420
#define ZUMAPRO_DP_SST1_VIDEO_HACTIVE		0x5424
#define ZUMAPRO_DP_SST1_VIDEO_VFP		0x5428
#define ZUMAPRO_DP_SST1_VIDEO_VBP		0x542c
#define ZUMAPRO_DP_SST1_VIDEO_VACTIVE		0x5430

#define ZUMAPRO_DP_SST1_INFOFRAME_UPDATE	0x5c04
#define ZUMAPRO_DP_SST1_INFOFRAME_SEND		0x5c08
#define INFOFRAME_AVI				BIT(2)
#define INFOFRAME_SPD				BIT(0)
#define ZUMAPRO_DP_SST1_INFOFRAME_AVI_DATA	0x5c40
#define ZUMAPRO_DP_SST1_INFOFRAME_SPD_DATA	0x5c60

/*
 * The stream's framing. Enhanced framing has to match what the sink was told
 * when the lane count went out, or the two disagree about every frame
 * boundary; the vendor sets it at link-up, next to the video function, and
 * this block generates the timing rather than following anything upstream.
 */
static void zumapro_dp_set_framing(struct zumapro_dp *dp)
{
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_MAIN_CONTROL,
			  SST1_VIDEO_MODE_SLAVE | SST1_ENHANCED_MODE,
			  dp->enhanced_framing ? SST1_ENHANCED_MODE : 0);
}

/*
 * The transfer unit: how much of each 64-symbol window carries pixels. It is
 * the ratio of what the mode needs to what the lanes can carry, and getting it
 * wrong is not a link failure -- the sink locks symbols and aligns lanes on a
 * stream stuffed at the wrong rate exactly as it does on a correct one, and
 * simply never reports video.
 */
static void zumapro_dp_set_active_symbol(struct zumapro_dp *dp,
					 const struct drm_display_mode *mode,
					 unsigned int bpp)
{
	/*
	 * The link symbol rate, which is what this block counts in and
	 * also the unit the DisplayPort helpers keep a link rate in: one
	 * symbol carries eight bits of the ten on the wire, so 5.4 Gbps
	 * is 540000 either way.
	 */
	unsigned int ls_clk = dp->link_rate;
	u32 integer, fraction, threshold;
	u64 tu, rem;

	/*
	 * How many symbols of each 64-symbol window carry pixels, scaled by
	 * 10^10 so the integer and fractional halves come out of one division.
	 *
	 * Both clocks are in kHz here. The vendor rounds them to MHz first,
	 * which leaves the integer part right and the fraction short -- a
	 * 1920x1080-60 link over two lanes at HBR wants 26 and 4e8, and the
	 * rounded arithmetic gives 26 and 3.1e8. Keeping the kHz is both
	 * closer and no harder.
	 */
	tu = (u64)mode->clock * bpp * 32 * 10000000000ULL;
	do_div(tu, dp->link_lanes * ls_clk * 8);

	integer = div64_u64_rem(tu, 10000000000ULL, &rem);
	fraction = div_u64(rem, 10);

	if (integer <= 2)
		threshold = 7;
	else if (integer <= 5)
		threshold = 8;
	else
		threshold = 9;

	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_ACTIVE_SYMBOL_INTEGER,
			  ACTIVE_SYMBOL_INTEGER,
			  FIELD_PREP(ACTIVE_SYMBOL_INTEGER, integer));
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_ACTIVE_SYMBOL_FRACTION,
			  ACTIVE_SYMBOL_FRACTION,
			  FIELD_PREP(ACTIVE_SYMBOL_FRACTION, fraction));
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_ACTIVE_SYMBOL_THRESHOLD,
			  ACTIVE_SYMBOL_THRESHOLD,
			  FIELD_PREP(ACTIVE_SYMBOL_THRESHOLD, threshold));

	/*
	 * Without the select the hardware ignores the threshold just written
	 * and stuffs the transfer units on a boundary of its own choosing.
	 */
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_ACTIVE_SYMBOL_THRESHOLD_SEL,
			  ACTIVE_SYMBOL_THRESHOLD_SEL,
			  ACTIVE_SYMBOL_THRESHOLD_SEL);
}

static void zumapro_dp_set_video_config(struct zumapro_dp *dp,
					const struct drm_display_mode *mode,
					unsigned int bpc)
{
	/*
	 * The link symbol rate, which is what this block counts in and
	 * also the unit the DisplayPort helpers keep a link rate in: one
	 * symbol carries eight bits of the ten on the wire, so 5.4 Gbps
	 * is 540000 either way.
	 */
	unsigned int ls_clk = dp->link_rate;
	unsigned int bpp;
	u32 val;

	/*
	 * Both already set at link-up. Training writes the function enable
	 * outside the lock, so a disable racing a retrain can lose either
	 * side's write; repeating them before the stream is described is what
	 * makes that harmless.
	 */
	zumapro_dp_video_func_enable(dp);
	zumapro_dp_set_framing(dp);

	/*
	 * Only three depths exist in the register, so the bits per pixel the
	 * transfer unit is sized from have to come from the one written rather
	 * than from what was asked for -- a sink offering twelve would
	 * otherwise be sent eight and stuffed for twelve.
	 */
	val = FIELD_PREP(VIDEO_COLOR_FORMAT, VIDEO_COLOR_FORMAT_RGB);
	switch (bpc) {
	case 6:
		val |= FIELD_PREP(VIDEO_BPC, VIDEO_BPC_6);
		bpp = 18;
		break;
	case 10:
		val |= FIELD_PREP(VIDEO_BPC, VIDEO_BPC_10);
		bpp = 30;
		break;
	default:
		val |= FIELD_PREP(VIDEO_BPC, VIDEO_BPC_8);
		bpp = 24;
		break;
	}

	/* A set bit is a negative sync, not a positive one. */
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		val |= VIDEO_VSYNC_POLARITY;
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		val |= VIDEO_HSYNC_POLARITY;

	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_VIDEO_CONTROL,
			  VIDEO_DYNAMIC_RANGE_CEA | VIDEO_BPC |
			  VIDEO_COLOR_FORMAT | VIDEO_VSYNC_POLARITY |
			  VIDEO_HSYNC_POLARITY, val);

	writel(mode->htotal, dp->regs + ZUMAPRO_DP_SST1_VIDEO_HTOTAL);
	writel(mode->vtotal, dp->regs + ZUMAPRO_DP_SST1_VIDEO_VTOTAL);
	writel(mode->hsync_start - mode->hdisplay,
	       dp->regs + ZUMAPRO_DP_SST1_VIDEO_HFP);
	writel(mode->htotal - mode->hsync_end,
	       dp->regs + ZUMAPRO_DP_SST1_VIDEO_HBP);
	writel(mode->hdisplay, dp->regs + ZUMAPRO_DP_SST1_VIDEO_HACTIVE);
	writel(mode->vsync_start - mode->vdisplay,
	       dp->regs + ZUMAPRO_DP_SST1_VIDEO_VFP);
	writel(mode->vtotal - mode->vsync_end,
	       dp->regs + ZUMAPRO_DP_SST1_VIDEO_VBP);
	writel(mode->vdisplay, dp->regs + ZUMAPRO_DP_SST1_VIDEO_VACTIVE);

	/*
	 * The sink recovers its pixel clock from the link rate scaled by M/N.
	 * The master value is a quarter of the stream clock because the lanes
	 * come off a Synopsys PHY, which this register layer -- written for a
	 * generation that paired the link with a Samsung one -- does not know.
	 */
	writel(mode->clock / 4, dp->regs + ZUMAPRO_DP_SST1_MVID_MASTER_MODE);
	writel(ls_clk, dp->regs + ZUMAPRO_DP_SST1_NVID_MASTER_MODE);
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_MVID_SFR_CONFIGURE,
			  MNVID_SFR_CONFIG,
			  FIELD_PREP(MNVID_SFR_CONFIG, mode->clock));
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_NVID_SFR_CONFIGURE,
			  MNVID_SFR_CONFIG,
			  FIELD_PREP(MNVID_SFR_CONFIG, ls_clk));

	zumapro_dp_set_active_symbol(dp, mode, bpp);

	/* The transmitter generates the timing; nothing upstream paces it. */
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_VIDEO_MASTER_TIMING_GEN,
			  VIDEO_MASTER_TIMING_GEN, VIDEO_MASTER_TIMING_GEN);
}

static void zumapro_dp_write_infoframe(struct zumapro_dp *dp, u32 offset,
				       const u8 *data, size_t len)
{
	unsigned int i;

	for (i = 0; i < len; i += 4) {
		u32 word = 0;
		unsigned int j;

		for (j = 0; j < 4 && i + j < len; j++)
			word |= (u32)data[i + j] << (j * 8);

		writel(word, dp->regs + offset + i);
	}
}

/*
 * A sink that bridges DisplayPort to HDMI builds its outgoing infoframe from
 * ours, and without one it has nothing to describe the picture with and
 * produces no output. This is the kind of gap a register-by-register
 * comparison cannot find: not a value written wrongly, a packet never sent.
 */
static void zumapro_dp_send_infoframes(struct zumapro_dp *dp,
				       struct drm_connector *connector,
				       const struct drm_display_mode *mode)
{
	union hdmi_infoframe frame;
	u8 buf[HDMI_INFOFRAME_SIZE(AVI)];
	ssize_t len;

	if (drm_hdmi_avi_infoframe_from_display_mode(&frame.avi, connector,
						     mode))
		return;

	len = hdmi_infoframe_pack(&frame, buf, sizeof(buf));
	if (len < HDMI_INFOFRAME_HEADER_SIZE)
		return;

	/*
	 * The hardware supplies the header and the checksum, so only the body
	 * goes in. The packed buffer's first four bytes are type, version,
	 * length and checksum -- which is the whole of the header size, the
	 * checksum included.
	 */
	zumapro_dp_write_infoframe(dp, ZUMAPRO_DP_SST1_INFOFRAME_AVI_DATA,
				   buf + HDMI_INFOFRAME_HEADER_SIZE,
				   len - HDMI_INFOFRAME_HEADER_SIZE);

	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_INFOFRAME_UPDATE, INFOFRAME_AVI,
			  INFOFRAME_AVI);
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_INFOFRAME_SEND, INFOFRAME_AVI,
			  INFOFRAME_AVI);
}

/*
 * The long-hop channel carries the pixels from the transmitter to the lanes.
 * A transmitter generating pixels it cannot put on the wire looks exactly like
 * a link that trained and shows nothing, so its power-up is waited for and a
 * failure is not proceeded past.
 */
static int zumapro_dp_video_enable(struct zumapro_dp *dp)
{
	u32 reg;
	int ret;

	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_SST1_FUNCTION_ENABLE,
			  SST1_LH_PWR_ON, SST1_LH_PWR_ON);

	ret = readl_poll_timeout(dp->regs +
				 ZUMAPRO_DP_SYSTEM_SST1_FUNCTION_ENABLE,
				 reg, reg & SST1_LH_PWR_ON_STATUS, 10, 2000);
	if (ret) {
		dev_err(dp->dev, "the pixel channel did not power up\n");
		return ret;
	}

	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_VIDEO_ENABLE, VIDEO_EN, VIDEO_EN);

	return 0;
}

static void zumapro_dp_video_disable(struct zumapro_dp *dp)
{
	zumapro_dp_update(dp, ZUMAPRO_DP_SST1_VIDEO_ENABLE, VIDEO_EN, 0);
	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_SST1_FUNCTION_ENABLE,
			  SST1_LH_PWR_ON, 0);
}

/*
 * Hotplug comes from the connector, over the configuration channel, so all the
 * bridge has to do is tell the link what the connector saw.
 */
/*
 * Undo what training moved onto the transmit clock.
 *
 * When the link trained, the block's clock mux was switched from its own
 * oscillator to the PHY's transmit clock, and the coding sublayer was enabled
 * behind it. That clock stops existing the moment the PHY leaves DisplayPort
 * mode, and a block still selected on it cannot answer a register access at
 * all: the read does not fail, it never completes, and the interconnect
 * eventually reports the stall. The next plug is what walks into it.
 *
 * So this has to run while the transmit clock is still there -- before the PHY
 * is told to leave -- and the mux has to go first, because everything after it
 * is clocked by whatever it selects.
 */
static void zumapro_dp_link_stop(struct zumapro_dp *dp)
{
	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_CLK_CONTROL,
			  GFCLKMUX_SEL_10 | GFCLKMUX_SEL_20, 0);

	writel(0, dp->regs + ZUMAPRO_DP_PCS_SNPS_DATAPATH_CONTROL);

	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_COMMON_FUNCTION_ENABLE,
			  PCS_FUNC_EN, 0);
}

static void zumapro_dp_hpd_notify(struct drm_bridge *bridge,
				  struct drm_connector *connector,
				  enum drm_connector_status status)
{
	struct zumapro_dp *dp = bridge_to_dp(bridge);
	bool present = status == connector_status_connected;
	int ret;

	scoped_guard(mutex, &dp->lock) {
		if (present == dp->sink_present)
			return;

		if (present) {
			/*
			 * The lanes and the AUX front end belong to the combo
			 * PHY, and AUX has to be awake before the sink can be
			 * asked anything. The PHY refuses while USB has not
			 * brought it up, in which case there is nothing to
			 * talk to a sink with.
			 */
			ret = phy_set_mode(dp->phy, PHY_MODE_DP);
			if (ret) {
				dev_err(dp->dev,
					"cannot reach the sink: the PHY is not up (%d)\n",
					ret);
				return;
			}
			zumapro_dp_aux_init(dp);
		} else {
			zumapro_dp_link_stop(dp);
			dp->link_rate = 0;
			dp->link_lanes = 0;
			phy_set_mode(dp->phy, PHY_MODE_INVALID);
		}

		dp->sink_present = present;
	}

	/*
	 * Train outside the lock: it talks to the sink over AUX, and AUX takes
	 * the same one.
	 *
	 * Train here rather than at modeset, because what the link will carry
	 * decides which modes can be offered at all. A failure leaves the sink
	 * reported as present -- the connector says it is -- with no usable
	 * link rate, and no mode will then validate.
	 */
	if (present && !zumapro_dp_train(dp))
		zumapro_dp_set_framing(dp);
}

static enum drm_connector_status zumapro_dp_detect(struct drm_bridge *bridge,
						   struct drm_connector *connector)
{
	struct zumapro_dp *dp = bridge_to_dp(bridge);

	/*
	 * Report what the connector told us. There is no hotplug pin on a
	 * USB-C port, so the link's own hotplug status has nothing behind it.
	 */
	guard(mutex)(&dp->lock);

	return dp->sink_present ? connector_status_connected :
				  connector_status_disconnected;
}

static const struct drm_edid *zumapro_dp_edid_read(struct drm_bridge *bridge,
						   struct drm_connector *connector)
{
	struct zumapro_dp *dp = bridge_to_dp(bridge);

	scoped_guard(mutex, &dp->lock)
		if (!dp->sink_present)
			return NULL;

	return drm_edid_read_ddc(connector, &dp->aux.ddc);
}

static void zumapro_dp_aux_unregister(void *data)
{
	drm_dp_aux_unregister(data);
}

/*
 * The aux channel is published here rather than at probe: it wants the DRM
 * device it belongs to, and until a bridge is attached there is none -- and
 * nothing that could be asking a sink anything either.
 */
static int zumapro_dp_attach(struct drm_bridge *bridge,
			     struct drm_encoder *encoder,
			     enum drm_bridge_attach_flags flags)
{
	struct zumapro_dp *dp = bridge_to_dp(bridge);
	int ret;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
		return -EINVAL;

	dp->aux.drm_dev = bridge->dev;

	ret = drm_dp_aux_register(&dp->aux);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dp->dev, zumapro_dp_aux_unregister,
					&dp->aux);
}

static enum drm_mode_status
zumapro_dp_mode_valid(struct drm_bridge *bridge,
		      const struct drm_display_info *info,
		      const struct drm_display_mode *mode)
{
	struct zumapro_dp *dp = bridge_to_dp(bridge);
	unsigned int bpp, rate, lanes;

	scoped_guard(mutex, &dp->lock) {
		rate = dp->link_rate;
		lanes = dp->link_lanes;
	}

	if (!rate)
		return MODE_NOCLOCK;

	/*
	 * The trained link carries a fixed number of bits per second, and a
	 * mode that needs more than that cannot be sent however it is timed.
	 */
	bpp = max(info->bpc, 8U) * 3;
	if ((u64)mode->clock * bpp > (u64)rate * lanes * 8)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static void zumapro_dp_atomic_enable(struct drm_bridge *bridge,
				     struct drm_atomic_commit *state)
{
	struct zumapro_dp *dp = bridge_to_dp(bridge);
	const struct drm_display_mode *mode;
	struct drm_connector_state *conn_state;
	struct drm_connector *connector;
	struct drm_crtc_state *crtc_state;
	unsigned int bpc;

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);
	if (!connector)
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state || !conn_state->crtc)
		return;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (!crtc_state)
		return;

	mode = &crtc_state->adjusted_mode;

	guard(mutex)(&dp->lock);

	if (!dp->link_rate) {
		dev_err(dp->dev, "no trained link to put a picture on\n");
		return;
	}

	/*
	 * The depth is the lesser of what the sink offers and what the
	 * connector's own property allows, so a user capping it is honoured.
	 */
	bpc = connector->display_info.bpc ? : 8;
	if (conn_state->max_bpc)
		bpc = min(bpc, conn_state->max_bpc);

	zumapro_dp_set_video_config(dp, mode, bpc);
	zumapro_dp_send_infoframes(dp, connector, mode);
	zumapro_dp_video_enable(dp);
}

static void zumapro_dp_atomic_disable(struct drm_bridge *bridge,
				      struct drm_atomic_commit *state)
{
	struct zumapro_dp *dp = bridge_to_dp(bridge);

	guard(mutex)(&dp->lock);

	zumapro_dp_video_disable(dp);
}

static const struct drm_bridge_funcs zumapro_dp_bridge_funcs = {
	.attach			= zumapro_dp_attach,
	.detect			= zumapro_dp_detect,
	.edid_read		= zumapro_dp_edid_read,
	.hpd_notify		= zumapro_dp_hpd_notify,
	.mode_valid		= zumapro_dp_mode_valid,
	.atomic_enable		= zumapro_dp_atomic_enable,
	.atomic_disable		= zumapro_dp_atomic_disable,
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_reset		= drm_atomic_helper_bridge_reset,
};

static int zumapro_dp_probe(struct platform_device *pdev)
{
	struct typec_switch_desc sw_desc = { };
	struct device *dev = &pdev->dev;
	struct zumapro_dp *dp;
	int ret;

	dp = devm_drm_bridge_alloc(dev, struct zumapro_dp, bridge,
				   &zumapro_dp_bridge_funcs);
	if (IS_ERR(dp))
		return PTR_ERR(dp);

	dp->dev = dev;
	platform_set_drvdata(pdev, dp);

	dp->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dp->regs))
		return PTR_ERR(dp->regs);

	dp->num_clks = devm_clk_bulk_get_all(dev, &dp->clks);
	if (dp->num_clks < 0)
		return dev_err_probe(dev, dp->num_clks,
				     "failed to get the link clocks\n");

	/*
	 * The SuperSpeed half of the USB-C combo PHY. The lanes are shared with
	 * USB3, so the link asks for a rate, a lane count and drive levels
	 * through the PHY interface rather than mapping the PHY itself.
	 */
	dp->phy = devm_phy_get(dev, "dp");
	if (IS_ERR(dp->phy))
		return dev_err_probe(dev, PTR_ERR(dp->phy),
				     "failed to get the DisplayPort PHY\n");

	ret = devm_mutex_init(dev, &dp->lock);
	if (ret)
		return ret;

	ret = zumapro_dp_read_osc_rate(dp);
	if (ret)
		return dev_err_probe(dev, ret, "the oscillator is unusable\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	/*
	 * The block shares a power domain with USB, which gates it whenever
	 * nothing holds it, and nothing the link is programmed with survives
	 * that. Hold it for as long as the driver is bound.
	 */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power the link\n");

	/*
	 * The connector reports which way up the plug went in. Registering
	 * this is also what lets the Type-C port find us: the port defers
	 * until every switch its graph names has appeared.
	 */
	sw_desc.drvdata = dp;
	sw_desc.fwnode = dev_fwnode(dev);
	sw_desc.set = zumapro_dp_orientation_set;
	sw_desc.name = NULL;

	dp->orien_sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(dp->orien_sw)) {
		ret = dev_err_probe(dev, PTR_ERR(dp->orien_sw),
				    "failed to register the orientation switch\n");
		goto err_pm;
	}

	ret = devm_add_action_or_reset(dev, zumapro_dp_switch_unregister,
				       dp->orien_sw);
	if (ret)
		goto err_pm;

	dp->aux.name = "zumapro-dp";
	dp->aux.dev = dev;
	dp->aux.transfer = zumapro_dp_aux_transfer;
	drm_dp_aux_init(&dp->aux);

	dp->bridge.of_node = dev->of_node;
	/*
	 * No DRM_BRIDGE_OP_HPD: hotplug is not something this bridge detects,
	 * it is something the connector tells it, and claiming the op would
	 * stop the connector polling without anything replacing it.
	 */
	dp->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID;
	dp->bridge.type = DRM_MODE_CONNECTOR_DisplayPort;
	ret = devm_drm_bridge_add(dev, &dp->bridge);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_put(dev);

	return ret;
}

/*
 * The power domain is shared with USB and goes down whenever nothing holds it,
 * which resets everything the link was programmed with -- so the bring-up is
 * the resume path, and probe reaches it by taking the first reference.
 */
static int zumapro_dp_runtime_resume(struct device *dev)
{
	struct zumapro_dp *dp = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(dp->num_clks, dp->clks);
	if (ret)
		return ret;

	scoped_guard(mutex, &dp->lock) {
		/*
		 * The oscillator's parent is a device tree default applied once
		 * at probe, and the mux it selects lives in the domain that has
		 * just come back. Read the rate rather than assume it: every
		 * timer in the block is derived from this number.
		 */
		/*
		 * The block has been reset, so whatever was trained is gone.
		 * Say so, or a mode will be put on a link that is not there.
		 */
		dp->link_rate = 0;
		dp->link_lanes = 0;

		ret = zumapro_dp_read_osc_rate(dp);
		if (!ret)
			ret = zumapro_dp_link_init(dp);
	}

	if (ret)
		clk_bulk_disable_unprepare(dp->num_clks, dp->clks);

	return ret;
}

/*
 * The reverse of zumapro_dp_link_init(), in the order the vendor's
 * dp_hw_deinit() does it: mask the interrupts, stop the software functions,
 * put the block back on its own oscillator, drop the module functions, and
 * only then release the Q-channel.
 *
 * The order is not decorative. Everything after the clock mux is clocked by
 * whatever it selects, and the Q-channel is the answer this block gives the
 * power controller when asked whether it is idle -- so it goes last, once
 * nothing is left running to make it busy.
 *
 * Leaving any of it set does not show up here. It shows up as the whole HSI0
 * domain refusing to power down, a secure write that never completes, and a
 * system suspend that does not return.
 */
static void zumapro_dp_link_deinit(struct zumapro_dp *dp)
{
	writel(0, dp->regs + ZUMAPRO_DP_SYSTEM_IRQ_COMMON_STATUS_MASK);

	writel(0, dp->regs + ZUMAPRO_DP_SYSTEM_SW_FUNCTION_ENABLE);

	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_CLK_CONTROL,
			  GFCLKMUX_SEL_10 | GFCLKMUX_SEL_20, 0);

	zumapro_dp_update(dp, ZUMAPRO_DP_SYSTEM_COMMON_FUNCTION_ENABLE,
			  HDCP22_FUNC_EN | HDCP13_FUNC_EN | GTC_FUNC_EN |
			  PCS_FUNC_EN | AUX_FUNC_EN, 0);

	writel(0, dp->regs + ZUMAPRO_DP_SYSTEM_OSCLK_QCH_FUNC_EN);
}

static int zumapro_dp_runtime_suspend(struct device *dev)
{
	struct zumapro_dp *dp = dev_get_drvdata(dev);

	zumapro_dp_link_deinit(dp);

	clk_bulk_disable_unprepare(dp->num_clks, dp->clks);

	return 0;
}

static const struct dev_pm_ops zumapro_dp_pm_ops = {
	RUNTIME_PM_OPS(zumapro_dp_runtime_suspend, zumapro_dp_runtime_resume,
		       NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static const struct of_device_id zumapro_dp_of_match[] = {
	{ .compatible = "google,zumapro-dp" },
	{ }
};
MODULE_DEVICE_TABLE(of, zumapro_dp_of_match);

struct platform_driver zumapro_dp_driver = {
	.probe	= zumapro_dp_probe,
	.driver	= {
		.name = "zumapro-dp",
		.of_match_table = zumapro_dp_of_match,
		.pm = pm_ptr(&zumapro_dp_pm_ops),
	},
};
