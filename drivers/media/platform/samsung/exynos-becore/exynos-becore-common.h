/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google Zumapro camera back-end core -- the driver's own state
 *
 * exynos-becore-regs.h describes the silicon; this describes the driver.  What
 * is here is what more than one of the driver's translation units needs: the
 * device and everything hanging off it, the profiles and word enumerations
 * that name a DMA's fields, and the prototypes of the calls that cross a file
 * boundary.
 *
 * A function belongs here only because a second file calls it.  A static one
 * that stays static is an internal detail of the file it lives in, and moving
 * it here would widen the surface for nothing.
 */

#ifndef EXYNOS_BECORE_COMMON_H
#define EXYNOS_BECORE_COMMON_H

#include <linux/compiler_types.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/media/samsung/exynos-becore-config.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <media/exynos-becore.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-isp.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#include "exynos-becore-regs.h"
/* The longer of the two per-stage CRC lists; checked against both at probe. */
#define BECORE_STREAM_CRC_MAX		26
/* Debug register overrides: enough to sweep a small LUT, not a whole block. */
#define BECORE_OVERRIDE_MAX		32
#define BECORE_OVERRIDE_TEXT_MAX	1024

/* Fixed neutral LTM policy for the proven 4000x3000 processing profile. */
#define BECORE_LTM_GRID_ROW_BYTES	0x800
#define BECORE_LTM_GRID_ROWS		48
#define BECORE_LTM_GRID_CELL_BYTES	0x100
#define BECORE_LTM_GRID_WIDTH_CELLS	4
#define BECORE_LTM_GRID_HEIGHT_CELLS	24
#define BECORE_LTM_UNITY_Q14		BIT(14)
#define BECORE_GRID_SIZE			(BECORE_LTM_GRID_ROW_BYTES * \
					 BECORE_LTM_GRID_ROWS)

enum becore_block_id {
	BECORE_RGBP,
	BECORE_MCFP,
	BECORE_YUVP,
	BECORE_MCSC,
	BECORE_NUM_BLOCKS,
};

enum becore_c2serv_id {
	BECORE_C2SERV_RGBP,
	BECORE_C2SERV_YUVP,
	BECORE_C2SERV_MCSC,
	BECORE_NUM_C2SERV,
};

struct becore_regval {
	u32 offset;
	u32 value;
};

struct becore_override {
	u32 reg;
	u32 value;
};

struct becore_rect {
	u32 x;
	u32 y;
	u32 width;
	u32 height;
};

struct becore_rgbp_input_profile {
	u32 data_format;
	u32 comp_control;
	u32 sbwc_block_width;
	u32 bytes_per_pixel;
	u32 header_stride;
	u32 businfo;
};

enum becore_rgbp_input_profile_id {
	BECORE_RGBP_INPUT_SBWC,
	BECORE_RGBP_INPUT_LINEAR,
	BECORE_RGBP_INPUT_PROFILE_COUNT,
};

enum becore_rgbp_input_word {
	BECORE_RGBP_CROP_SIZE,
	BECORE_RGBP_CROP_START,
	BECORE_RGBP_SC_DST_SIZE,
	BECORE_RGBP_SC_H_RATIO,
	BECORE_RGBP_SC_V_RATIO,
	BECORE_RGBP_CHAIN_SRC_SIZE,
	BECORE_RGBP_CHAIN_DST_SIZE,
	BECORE_RGBP_INPUT_FORMAT,
	BECORE_RGBP_INPUT_COMP,
	BECORE_RGBP_INPUT_ACTIVE_WIDTH,
	BECORE_RGBP_INPUT_HEIGHT,
	BECORE_RGBP_INPUT_STRIDE,
	BECORE_RGBP_INPUT_HEADER_STRIDE,
	BECORE_RGBP_INPUT_BUSINFO,
	BECORE_RGBP_INPUT_ENABLE,
	BECORE_RGBP_INPUT_STORAGE_WIDTH,
	BECORE_RGBP_INPUT_WORD_COUNT,
};

/*
 * What the three rasters start at: the readout the whole captured corpus came
 * from, and the request it was serving.  The array's is a starting value only,
 * because a stream with a producer takes that raster from the producer's pad;
 * nothing yet sets the other two.  None of the three is a description of
 * anything -- becore->array, ->chain and ->scaled are what the driver reads.
 */
#define BECORE_ARRAY_WIDTH		4208U
#define BECORE_ARRAY_HEIGHT		3120U
#define BECORE_OUTPUT_WIDTH		4000U
#define BECORE_OUTPUT_HEIGHT		3000U
#define BECORE_CHAIN_WIDTH		4160U
#define BECORE_CHAIN_HEIGHT		3120U

/*
 * A raster the chain works on, carried rather than compiled in.  The blocks
 * below describe *formats* -- how a surface is laid out and compressed -- and
 * a format is the same whatever size the picture is, so the size is not one of
 * their fields.  Passing it beside the profile is the shape the input side
 * already has, and it is what lets the encoders stay pure functions of what
 * they are given: the offline loop can then encode for a geometry the hardware
 * is not currently running.
 */
struct becore_raster {
	u32 width;
	u32 height;
};

struct becore_yuvp_output_profile {
	u32 data_format;
	u32 mode;
	u32 lossy_byte32num;
	u32 votf_enable;
	/*
	 * A compressed surface has no bytes per pixel and does not set this;
	 * its stride is a count of blocks.  A plausible 2 left here is what the
	 * chain surface's stride was once derived from.
	 */
	u32 bytes_per_pixel;
	u32 block_width;
	u32 block_height;
	u32 luma_height_align;
	u32 plane_gap;
	u32 businfo;
};

enum becore_yuvp_output_profile_id {
	BECORE_YUVP_OUTPUT_SBWCL,
	BECORE_YUVP_OUTPUT_P010,
	BECORE_YUVP_OUTPUT_PROFILE_COUNT,
};

enum becore_yuvp_output_word {
	BECORE_YUVP_OUTPUT_VOTF,
	BECORE_YUVP_OUTPUT_FORMAT,
	BECORE_YUVP_OUTPUT_LOSSY,
	BECORE_YUVP_OUTPUT_MODE,
	BECORE_YUVP_OUTPUT_WIDTH,
	BECORE_YUVP_OUTPUT_HEIGHT,
	BECORE_YUVP_OUTPUT_STRIDE1,
	BECORE_YUVP_OUTPUT_STRIDE2,
	BECORE_YUVP_OUTPUT_BUSINFO,
	BECORE_YUVP_OUTPUT_ENABLE,
	BECORE_YUVP_OUTPUT_WORD_COUNT,
};

struct becore_gtnr_dma_profile {
	u32 votf_enable;
	u32 businfo;
	u32 max_mo;
	u32 max_bl;
	u32 enable;
};

enum becore_gtnr_dma_word {
	BECORE_GTNR_INPUT_VOTF,
	BECORE_GTNR_INPUT_FORMAT,
	BECORE_GTNR_INPUT_LOSSY,
	BECORE_GTNR_INPUT_COMP,
	BECORE_GTNR_INPUT_WIDTH,
	BECORE_GTNR_INPUT_HEIGHT,
	BECORE_GTNR_INPUT_STRIDE1,
	BECORE_GTNR_INPUT_STRIDE2,
	BECORE_GTNR_INPUT_BUSINFO,
	BECORE_GTNR_INPUT_MAX_MO,
	BECORE_GTNR_INPUT_MAX_BL,
	BECORE_GTNR_INPUT_ENABLE,
	BECORE_GTNR_OUTPUT_FORMAT,
	BECORE_GTNR_OUTPUT_LOSSY,
	BECORE_GTNR_OUTPUT_COMP,
	BECORE_GTNR_OUTPUT_WIDTH,
	BECORE_GTNR_OUTPUT_HEIGHT,
	BECORE_GTNR_OUTPUT_STRIDE1,
	BECORE_GTNR_OUTPUT_STRIDE2,
	BECORE_GTNR_OUTPUT_BUSINFO,
	BECORE_GTNR_OUTPUT_MAX_MO,
	BECORE_GTNR_OUTPUT_MAX_BL,
	BECORE_GTNR_OUTPUT_ENABLE,
	BECORE_GTNR_DMA_WORD_COUNT,
};

struct becore_mcsc_dma_profile {
	u32 data_format;
	u32 comp_control;
	u32 lossy_byte32num;
	u32 votf_enable;
	u32 businfo;
	u32 max_bl;
	u32 enable;
	u32 dither;
};

enum becore_mcsc_input_transport {
	BECORE_MCSC_INPUT_CAPTURED_VOTF,
	BECORE_MCSC_INPUT_MEMORY,
};

enum becore_mcsc_dma_word {
	BECORE_MCSC_INPUT_VOTF,
	BECORE_MCSC_INPUT_FORMAT,
	BECORE_MCSC_INPUT_LOSSY,
	BECORE_MCSC_INPUT_COMP,
	BECORE_MCSC_INPUT_WIDTH,
	BECORE_MCSC_INPUT_HEIGHT,
	BECORE_MCSC_INPUT_STRIDE1,
	BECORE_MCSC_INPUT_STRIDE2,
	BECORE_MCSC_INPUT_BUSINFO,
	BECORE_MCSC_INPUT_MAX_BL,
	BECORE_MCSC_INPUT_ENABLE,
	BECORE_MCSC_DJAG_IMG_SIZE,
	BECORE_MCSC_DJAG_PS_SRC_POS,
	BECORE_MCSC_DJAG_PS_SRC_SIZE,
	BECORE_MCSC_DJAG_PS_DST_SIZE,
	BECORE_MCSC_DJAG_PS_H_RATIO,
	BECORE_MCSC_DJAG_PS_V_RATIO,
	BECORE_MCSC_OUTPUT_FORMAT,
	BECORE_MCSC_OUTPUT_COMP,
	BECORE_MCSC_OUTPUT_WIDTH,
	BECORE_MCSC_OUTPUT_HEIGHT,
	BECORE_MCSC_OUTPUT_STRIDE1,
	BECORE_MCSC_OUTPUT_STRIDE2,
	BECORE_MCSC_OUTPUT_BUSINFO,
	BECORE_MCSC_OUTPUT_MAX_BL,
	BECORE_MCSC_OUTPUT_ENABLE,
	BECORE_MCSC_OUTPUT_DITHER,
	BECORE_MCSC_DMA_WORD_COUNT,
};

struct becore_device;

#define BECORE_INPUT_SLOT_COUNT	3

struct becore_dma_buffer {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	size_t staged_bytes;
	struct sg_table *sgt;
};

struct becore_ltm_gain_offset_group {
	__le16 gain[4];
	__le16 offset[4];
};

#define BECORE_LTM_GRID_GROUPS_PER_CELL \
	(BECORE_LTM_GRID_CELL_BYTES / \
	 sizeof(struct becore_ltm_gain_offset_group))

struct becore_ltm_grid_cell {
	struct becore_ltm_gain_offset_group
		groups[BECORE_LTM_GRID_GROUPS_PER_CELL];
};

enum becore_input_slot_state {
	BECORE_INPUT_FREE,
	BECORE_INPUT_PRODUCER,
	BECORE_INPUT_READY,
	BECORE_INPUT_BACKEND,
	BECORE_INPUT_QUARANTINED,
};

/*
 * A slot's pages are written by ISPFE's DMA and read by RGBP's, and both are
 * non-coherent masters that go to DRAM: the CPU is not part of that exchange,
 * so it needs no cache maintenance between them.  The one thing that does is
 * the debugfs staging path, which writes a slot through the cached vmap
 * dma_vmap_noncontiguous() returns.  cpu_dirty says a slot is in that state
 * and is the only reason a sync is issued.
 */
struct becore_input_slot {
	struct becore_dma_buffer buffer;
	enum becore_input_slot_state state;
	bool cpu_dirty;
	u64 producer_cookie;
	u64 ready_sequence;
};

/*
 * A CMDQ program's allocation and its contents are two different lengths. The
 * allocation is sized once, for the longest program the block can be asked to
 * run; the header count is what the last encode emitted, and it is what
 * CMDQ_QUE_CMD_M tells the hardware to execute.
 *
 * The payload area starts after the header list, so the two lengths also give
 * two different layouts -- which is why the debugfs readback returns the
 * encoded length rather than the allocation. A reader handed the allocation
 * would look for the payloads in the wrong place.
 */
struct becore_cmdq_program {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u32 capacity;
	u32 header_count;
};

struct becore_block {
	struct becore_device *becore;
	const char *name;
	void __iomem *base;
	u32 int0_mask_prepare;
	u32 int0_mask;
	u32 int1_mask;
	u32 cmdq_int_mask;
	u32 last_int0;
	u32 last_int1;
	u32 last_cmdq_int;
	atomic64_t int0_count;
	atomic64_t int1_count;
	u32 stream_crc_armed[BECORE_STREAM_CRC_MAX];
	u32 stream_crc_result[BECORE_STREAM_CRC_MAX];
};

struct becore_irq {
	struct becore_block *block;
	int irq;
	bool int1;
};

struct becore_video_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

/*
 * What userspace has most recently asked for, in the units the blocks are
 * specified in rather than as register words.  A frame carries only what
 * changed, so a value persists until a later buffer replaces it, and a value
 * that was never sent leaves the driver's own default in place.
 */
struct becore_params_state {
	s16 ccm[EXYNOS_BECORE_CCM_COEFFICIENTS];
	s16 ccm_offsets[EXYNOS_BECORE_CCM_OFFSETS];
	u16 ltm_curve[EXYNOS_BECORE_LTM_CURVE_POINTS];
	u16 clut_u[EXYNOS_BECORE_CLUT_NODES];
	u16 clut_v[EXYNOS_BECORE_CLUT_NODES];
	u16 gamma[EXYNOS_BECORE_GAMMA_CHANNELS][EXYNOS_BECORE_GAMMA_POINTS];
	struct exynos_becore_params_sharpen sharpen;
	struct exynos_becore_params_yuvnr yuvnr;
	struct exynos_becore_params_byr_dns byr_dns;
	struct exynos_becore_params_dmsc dmsc;
	bool ccm_valid;
	bool ltm_curve_valid;
	bool clut_valid;
	bool gamma_valid;
	bool sharpen_valid;
	bool yuvnr_valid;
	bool byr_dns_valid;
	bool dmsc_valid;
};

struct becore_params_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	/*
	 * A kernel copy taken at buf_prepare: validation is worthless against
	 * memory userspace can still write to after it has been checked.
	 */
	struct v4l2_isp_params_buffer *config;
};

static inline struct becore_params_buffer *
to_becore_params_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct becore_params_buffer, vb);
}

static inline struct becore_video_buffer *
to_becore_video_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct becore_video_buffer, vb);
}

/*
 * Where a run's wall clock goes.  The phases tile the whole of a run without
 * overlapping, so they sum to BECORE_TIMING_FRAME: each mark is taken once, at
 * the boundary between two phases, and hands its timestamp to the next.
 */
enum becore_timing_phase {
	BECORE_TIMING_RESUME,
	BECORE_TIMING_ENCODE,
	BECORE_TIMING_ARM,
	BECORE_TIMING_STAGE1,
	BECORE_TIMING_STAGE2,
	BECORE_TIMING_CRC,
	BECORE_TIMING_SUSPEND,
	BECORE_TIMING_FRAME,
	BECORE_TIMING_PHASE_COUNT,
};

struct becore_timing {
	u64 last_ns;
	u64 total_ns;
};

/* One producer endpoint, as the hardware reads it back. */
struct becore_c2serv_tws_state {
	u32 conn;
	u32 conn_raw;
	u32 rcv_valid;
	u32 enable;
	u32 limit;
	u32 dest;
	u32 lines_in_token;
	u32 busy;
	u32 fullness;
};

/* One consumer endpoint, as the hardware reads it back. */
struct becore_c2serv_trs_state {
	u32 conn;
	u32 conn_raw;
	u32 rcv_valid;
	u32 enable;
	u32 limit;
	u32 lines_in_first_token;
	u32 lines_in_token;
	u32 lines_count;
	u32 busy;
	u32 lost_connection;
};

/*
 * What a window said after being made ready.  Only `ready` is a verdict: it
 * comes from the software reset clearing itself, which is the one C2SERV read
 * the vendor's own stream makes.  The rest are read because they are cheap
 * and say something, not because anything is known about how they read back
 * -- and the two ring bits should read zero, because this driver does not
 * start the ring.
 *
 * `reset_enables` is the endpoint enable mask as it stands after the software
 * reset and before any endpoint is written: bit n for producer n, bit 16 + n
 * for consumer n.  It is sampled after SEL_REGISTER rather than before,
 * because until the immediate bank is selected a read answers for the shadow
 * alias instead.  Samsung's field table says both enables reset to 1, which
 * would mean a freshly reset window has every endpoint live; the whole safety
 * of preparing a window rests on that, so it is measured here rather than
 * assumed.
 */
struct becore_c2serv_state {
	bool ready;
	u32 ring_clk_en;
	u32 ring_enable;
	u32 local_ip;
	u32 reset_enables;
};

/*
 * The link's own registers, kept apart from the window state on purpose: a
 * one-shot suspends as soon as it finishes, and tearing the windows down
 * would otherwise erase the sample taken from a run that had just failed --
 * which is the only thing that says why.  Nothing clears this but the next
 * sample.
 */
struct becore_c2serv_link_state {
	bool sampled;
	struct becore_c2serv_tws_state tws[BECORE_C2SERV_LINK_PLANES];
	struct becore_c2serv_trs_state trs[BECORE_C2SERV_LINK_PLANES];
};

struct becore_device {
	struct device *dev;
	struct clk *intcam_clk;
	unsigned long saved_intcam_rate;
	bool intcam_rate_active;
	struct becore_block blocks[BECORE_NUM_BLOCKS];
	struct becore_irq irqs[BECORE_NUM_BLOCKS * 2];
	void __iomem *ssmt[14];
	void __iomem *sysreg_rgbp;
	void __iomem *sysreg_mcsc;
	void __iomem *c2serv[BECORE_NUM_C2SERV];
	struct becore_c2serv_state c2serv_state[BECORE_NUM_C2SERV];
	/* The link as armed, and as it stood when the run finished. */
	struct becore_c2serv_link_state c2serv_armed;
	struct becore_c2serv_link_state c2serv_done;
	struct dev_pm_domain_list *pm_domains;
	struct dentry *debugfs;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *red_balance;
	struct v4l2_ctrl *blue_balance;
	/*
	 * The Bayer array the front end sends, and the raster the chain runs
	 * at -- what RGBP hands YUVP, what YUVP writes, what MCSC reads back
	 * and what DJAG crops from.  Both were fields of the profiles that
	 * describe the *formats* involved, the chain's in six of them at once;
	 * a format is the same at any size, so neither belongs there, and a
	 * geometry that moved five of six copies would have programmed a chain
	 * disagreeing with itself.
	 */
	struct becore_raster array;
	struct becore_raster chain;
	/*
	 * The scaled picture MCSC writes, and so the capture node's format.
	 * It is the one of the three that userspace has any business setting;
	 * nothing lets it yet.
	 */
	struct becore_raster scaled;
	struct v4l2_subdev sd;
	struct media_pad sink_pad;
	bool sd_registered;
	/* Set once the entities exist; probe can return success without them. */
	bool video_ready;
	/*
	 * The mosaic the producer says it is sending, latched when a stream
	 * starts. Without a producer -- the offline loop -- it stays at the
	 * ultrawide's, which is the camera every captured program came from.
	 */
	u32 input_code;
	/*
	 * The white balance the back end encodes against: the Bayer denoiser's
	 * noise factors are its tuning times these gains.  Written by the
	 * control op rather than copied at STREAMON, so that there is one
	 * writer rather than a second source for a fact the controls already
	 * hold.  Green is unity, which is what normalises the other two.
	 *
	 * Both controls are grabbed for the length of a *stream*, so a video
	 * frame cannot see one move.  The offline loop is the other way round
	 * -- it runs precisely when nothing is streaming -- so these are read
	 * and written once each, and the worst a concurrent ioctl can do is
	 * give one offline frame a gain an ioctl old.
	 */
	u32 encode_balance_red;
	u32 encode_balance_blue;
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct vb2_queue queue;
	struct video_device params_vdev;
	struct media_pad params_pad;
	struct vb2_queue params_queue;
	/* Serializes V4L2 ioctls and vb2 queue setup on the parameters node. */
	struct mutex params_lock;
	struct list_head queued_params;
	struct work_struct params_work;
	struct becore_params_state params;
	/* Serializes V4L2 ioctls and vb2 queue setup/teardown. */
	struct mutex video_lock;
	/* Protects the pending processed-output and parameters buffer lists. */
	spinlock_t queue_lock;
	struct list_head queued_outputs;
	struct work_struct video_work;
	/* Serializes staging, input ownership, execution, and output inspection. */
	struct mutex lock;
	/* Protects IRQ-driven command-hold/frame completion state. */
	spinlock_t run_lock;
	struct completion run_completion;
	struct becore_input_slot inputs[BECORE_INPUT_SLOT_COUNT];
	struct becore_input_slot *run_input;
	struct becore_dma_buffer grid;
	struct becore_dma_buffer output;
	struct becore_dma_buffer gtnr_output;
	struct becore_dma_buffer mcsc_output;
	struct exynos_becore_input *input_producer;
	u64 producer_sequence;
	u64 input_sequence;
	struct becore_cmdq_program program[BECORE_NUM_BLOCKS];
	struct becore_cmdq_program gtnr_program;
	struct becore_cmdq_program mcsc_program;
	u8 *recipe;
	u8 *gtnr_recipe;
	u8 *mcsc_recipe;
	size_t recipe_staged_bytes;
	size_t gtnr_recipe_staged_bytes;
	size_t mcsc_recipe_staged_bytes;
	u32 recipe_generation;
	u32 gtnr_recipe_generation;
	u32 gtnr_encoded_generation;
	u32 mcsc_recipe_generation;
	u32 mcsc_encoded_generation;
	u32 grid_generation;
	u32 run_generation;
	u32 completed_generation;
	u32 video_sequence;
	u32 params_sequence;
	u32 cmdq_hold_mask;
	u32 frame_done_mask;
	u32 expected_mask;
	u32 output_changed_bytes;
	u32 output_first_changed;
	u32 mcsc_output_changed_bytes;
	u32 mcsc_output_first_changed;
	u32 stream_crc_seed;
	u32 stream_crc_armed_seed;
	u32 stream_crc_generation;
	struct becore_override overrides[BECORE_OVERRIDE_MAX];
	u32 override_count;
	char override_text[BECORE_OVERRIDE_TEXT_MAX];
	size_t override_text_len;
	/* Compared, never dereferenced: which descriptor owns the text. */
	const struct file *override_writer;
	u32 input_profile;
	u32 active_input_profile;
	u32 output_profile;
	u32 active_output_profile;
	/*
	 * Carry YUVP into MCSC over the fabric rather than through DRAM.  On
	 * by default; clearing it puts the frame back through memory, which
	 * is a slower path that is otherwise identical and is worth keeping
	 * as the thing to compare a result against.
	 */
	u32 votf;
	/* Debug geometry: zero means the captured value, per plane. */
	u32 votf_tws_limit;
	u32 votf_trs_limit;
	u32 votf_tws_token[BECORE_C2SERV_LINK_PLANES];
	u32 votf_trs_token[BECORE_C2SERV_LINK_PLANES];
	bool active_votf;
	u32 mcsc_completed_generation;
	u32 mcsc_completed_output_size;
	enum becore_mcsc_input_transport mcsc_encoded_transport;
	dma_addr_t active_output_dma;
	size_t active_output_size;
	size_t active_capture_size;
	size_t completed_output_size;
	int last_run_result;
	bool running;
	bool start_issued;
	bool abort_run;
	bool irq_error;
	bool irqs_enabled;
	bool reset_failed;
	bool active_output_packed;
	bool active_mcsc;
	bool output_quarantined;
	bool video_streaming;
	bool producer_streaming;
	bool stream_powered;
	/*
	 * Where MCSC writes this run.  The driver-owned buffer except during a
	 * capture, which redirects it at the queued vb2 buffer; every path that
	 * encodes MCSC sets it first rather than inheriting the last run's.
	 */
	dma_addr_t mcsc_dest_dma;
	/* Written only by a completed run, read under the same lock. */
	struct becore_timing timing[BECORE_TIMING_PHASE_COUNT];
	u32 timing_runs;
};

struct exynos_becore_input {
	struct becore_device *becore;
	struct device *producer;
	const struct exynos_becore_input_producer_ops *ops;
	void *producer_data;
	refcount_t callback_users;
	wait_queue_head_t callback_wait;
	bool disconnected;
	struct sg_table sgts[BECORE_INPUT_SLOT_COUNT];
	dma_addr_t dmas[BECORE_INPUT_SLOT_COUNT];
};

/*
 * Which parameters block carries a curve's knots.
 *
 * A curve whose knots the recipe still carries reads them once at probe; one a
 * block carries has to be re-read whenever a buffer arrives, because a slope
 * that describes knots the hardware was not given is a piecewise-linear curve
 * whose segments contradict itself.
 */
enum becore_noise_source {
	BECORE_NOISE_FROM_RECIPE,
	BECORE_NOISE_FROM_YUVNR,
	BECORE_NOISE_FROM_BYR_DNS,
};

/*
 * Each entry names the knots its slopes come from, so a chroma curve whose own
 * domain registers this table generates points at the luma copy it repeats
 * rather than at itself -- those would read back as zero.
 */
struct becore_noise_curve {
	enum becore_block_id block;
	u32 x_first;
	u32 y_first;
	u32 slope_first;
	u32 shift_reg;
	u32 domain_first;	/* 0 when nothing here generates the domain */
	bool round;		/* to nearest; false truncates */
	/*
	 * Whether the shift is searched for rather than fixed.  Both
	 * translators search, from eleven fractional bits down until the
	 * quotient fits, and both truncate the quotient to `short` before
	 * testing the fit -- no shipped tuning reaches either, but a
	 * parameters block's knots are userspace's and can.
	 */
	bool search;
	/* Where the range comes from when a parameters block carries it. */
	enum becore_noise_source source;
	size_t params_range;	/* offsetof() the range within that block */
};

/*
 * How many of them there are.  becore_noise_curves[] is declared at this
 * length so that a file which does not define the table can still walk it,
 * which makes a curve added without a count added here a build failure rather
 * than a table walked one entry short.
 */
#define BECORE_NOISE_CURVES		4

struct becore_cmdq_shape;

/* exynos-becore-params.c */
int becore_params_init(struct becore_device *becore);
void becore_params_work(struct work_struct *work);
void becore_params_consume(struct becore_device *becore);
int becore_params_value(const struct becore_device *becore, u32 reg, u32 *value);
int __must_check becore_params_check_yuvnr(struct device *dev,
					   const struct exynos_becore_params_yuvnr *yuvnr);
int becore_sharpen_value(const struct exynos_becore_params_sharpen *params,
			 u32 reg, u32 *value);
int becore_yuvnr_value(const struct exynos_becore_params_yuvnr *params,
		       u32 reg, u32 *value);
int becore_yuvnr_tnr_lut_x(const struct exynos_becore_params_yuvnr *params,
			   u32 reg, u32 *value);
int becore_yuvnr_tnr_slope(const struct exynos_becore_params_yuvnr *params,
			   u32 reg, u32 *value);
int becore_byrdns_value(const struct becore_device *becore,
			const struct exynos_becore_params_byr_dns *params,
			u32 reg, u32 *value);
int becore_byr_dns_biquad_value(const struct becore_raster *array,
				const struct exynos_becore_params_byr_dns *params,
				u32 *value);
int becore_dmsc_value(const struct exynos_becore_params_dmsc *params,
		      u32 reg, u32 *value);
int becore_rgbp_dmsc_value(u32 offset, u32 *value);
int becore_sharpen_table_validate(struct device *dev);
int becore_yuvnr_table_validate(struct device *dev);
int becore_byr_dns_table_validate(struct device *dev);
int becore_dmsc_table_validate(struct device *dev);
extern const struct exynos_becore_params_yuvnr becore_yuvnr_off;
extern const struct exynos_becore_params_dmsc becore_dmsc_neutral;

/*
 * How many words each block hands to becore_generated_value(). Held here
 * rather than in the generated table so that a recipe which quietly stopped
 * carrying one of them fails validation instead of programming the capture.
 */
#define BECORE_RGBP_GENERATED_WORDS	348
#define BECORE_YUVP_GENERATED_WORDS	1365
#define BECORE_MCSC_GENERATED_WORDS	116

/*
 * What a raster has to be before anything derives a register from it.  Every
 * clause here is a silent failure rather than a loud one, which is why they
 * are checked at all: none of them is reachable while the three rasters are
 * probe-time constants, and all of them become reachable the moment something
 * negotiates one.
 *
 * An extent has to fit the 16-bit half becore_pack_size() puts it in.  At
 * 65536 the shift walks into the other half instead of overflowing: an array
 * that wide encodes chain_src_img_size as 0x00000c30, a zero width, and no
 * register write fails.
 *
 * An odd extent has no whole last pair of anything.  The array is read out in
 * Bayer quads, the chain is the 4:2:0 surface YUVP writes and GTNR and MCSC
 * read back, and the scaled output is NV21 -- all three side their chroma on a
 * 2x2 grid the last row or column would fall off.
 */
#define BECORE_RASTER_EXTENT_MAX	U16_MAX

/*
 * The array's bound is tighter, and it comes from the two blocks that state a
 * radial fall-off centre.  BYR_DNS writes -(array extent / 2) and YUVNR writes
 * -(crop extent / 2), both into a 15-bit signed field, so the largest array
 * either can describe is twice that field's negative span.  The crop is cut
 * out of the array, so bounding the array bounds YUVNR's copy with it.
 */
#define BECORE_ARRAY_EXTENT_MAX		(BECORE_RGBP_DNS_CENTRE_MASK + 1)

/* exynos-becore-geometry.c */
size_t becore_rgbp_input_size(const struct becore_rgbp_input_profile *profile,
			      const struct becore_raster *array);
size_t
becore_rgbp_input_image_offset(const struct becore_rgbp_input_profile *profile,
			       const struct becore_raster *array);
size_t becore_input_allocation_size(const struct becore_raster *array);
int becore_input_profiles_validate(struct device *dev,
				   const struct becore_raster *array);
int becore_output_profiles_validate(struct device *dev);
u32 becore_pack_size(u32 high, u32 low);
int becore_zoom_ratio(u32 in, u32 out, u32 *ratio);
u32 becore_scaler_init_phase(u32 ratio);
int becore_raster_validate(struct device *dev, const char *name,
			   const struct becore_raster *raster, u32 max);
int becore_chain_validate(struct device *dev,
			  const struct becore_raster *chain);
int becore_rgbp_crop(const struct becore_raster *array,
		     const struct becore_raster *chain,
		     struct becore_rect *crop);
int becore_rgbp_input_value(const struct becore_rgbp_input_profile *profile,
			    const struct becore_raster *array,
			    const struct becore_raster *chain,
			    u32 index, u32 reg, u32 *value);
const struct becore_rgbp_input_profile *
becore_rgbp_input_profile(const struct becore_device *becore);
const struct becore_yuvp_output_profile *
becore_yuvp_output_profile(const struct becore_device *becore);
size_t becore_active_output_plane2_offset(const struct becore_device *becore);
size_t becore_active_output_size(const struct becore_device *becore);
size_t becore_yuvp_output_allocation_size(const struct becore_raster *chain);
size_t becore_yuvp_output_size(const struct becore_yuvp_output_profile *profile,
			       const struct becore_raster *chain);
int becore_gtnr_dma_value(const struct becore_raster *chain, u32 index,
			  u32 reg, u32 *value);
size_t becore_gtnr_surface_size(const struct becore_raster *chain);
u32 becore_mcsc_output_stride(const struct becore_raster *output);
size_t becore_mcsc_output_active_size(const struct becore_raster *output);
size_t becore_mcsc_output_size(const struct becore_raster *output);
int becore_mcsc_dma_value(const struct becore_raster *chain,
			  const struct becore_raster *output, u32 index, u32 reg,
			  enum becore_mcsc_input_transport transport,
			  const u32 *requested_token, u32 *value);
dma_addr_t becore_gtnr_address_dma(struct becore_device *becore, u32 reg);
dma_addr_t becore_mcsc_address_dma(struct becore_device *becore, u32 reg);
u32 becore_typed_word_count(enum becore_block_id id);
int becore_typed_value(struct becore_device *becore,
		       enum becore_block_id id, u32 index, u32 reg,
		       u32 *value);
extern const u32 becore_rgbp_input_regs[BECORE_RGBP_INPUT_WORD_COUNT];
extern const struct becore_yuvp_output_profile
	becore_yuvp_outputs[BECORE_YUVP_OUTPUT_PROFILE_COUNT];

/* exynos-becore-generated.c */
int becore_generated_value(const struct becore_device *becore,
			   enum becore_block_id id, u32 reg, u32 *value);
u32 becore_generated_word_count(enum becore_block_id id);
int becore_generated_tables_validate(struct device *dev);
int becore_noise_knots_resolve(struct device *dev);
int becore_yuvnr_geometry_value(const struct becore_raster *array,
				const struct becore_raster *chain,
				u32 offset, u32 *value);
int becore_yuvp_gamma_curve_value(const struct becore_params_state *params,
				  u32 reg, u32 *value);
extern const struct becore_noise_curve becore_noise_curves[BECORE_NOISE_CURVES];

/* exynos-becore-core.c */
int becore_bayer_phase(u32 code);
u32 becore_mcsc_votf_enable(const u32 *requested_token);
int becore_shape_register(const struct becore_cmdq_shape *shape,
			  u32 word, u32 *reg);
extern const struct v4l2_file_operations becore_fops;

#endif /* EXYNOS_BECORE_COMMON_H */
