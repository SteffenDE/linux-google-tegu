/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MEDIA_EXYNOS_BECORE_H__
#define __MEDIA_EXYNOS_BECORE_H__

#include <linux/types.h>

struct device;
struct media_entity;
struct v4l2_device;

/* Private attachment returned to one camera-front-end producer. */
struct exynos_becore_input;

#define EXYNOS_BECORE_WBG_GAIN_MIN_Q12		1U
#define EXYNOS_BECORE_WBG_GAIN_MAX_Q12		32751U
/* A gain of 1.0, and what the greens are captured at rather than held at. */
#define EXYNOS_BECORE_WBG_UNITY_Q12		4096U
#define EXYNOS_BECORE_WBG_RED_DEFAULT_Q12	8473U
#define EXYNOS_BECORE_WBG_BLUE_DEFAULT_Q12	6851U

/* Stream-latched semantic policy passed to the attached front end. */
struct exynos_becore_input_stream_config {
	/*
	 * Unsigned Q12, and the two the back end has: this configuration says
	 * nothing about the greens, so a front end keeps its own.
	 */
	u32 red_balance;
	u32 blue_balance;
};

/* Unsigned Q12 white balance gains, in the Bayer channel order R, Gr, Gb, B. */
struct exynos_becore_input_gains {
	u32 red;
	u32 green_red;
	u32 green_blue;
	u32 blue;
};

struct exynos_becore_input_producer_ops {
	int (*start_streaming)(void *data,
			       const struct exynos_becore_input_stream_config *config);
	void (*stop_streaming)(void *data);
};

/*
 * One driver-owned input slot reserved for a front-end producer.
 *
 * @gains is the white balance the frame in this buffer was taken through, and
 * it travels with the frame rather than beside it because that is what its one
 * consumer wants: the Bayer denoiser's noise factors are scaled by the gains
 * applied in front of it -- photon noise scales with them -- so what it needs
 * is the gains of *this* frame.  A "read whatever is newest" push would be
 * right most of the time and wrong exactly while the gains are moving, which
 * is when it matters.
 *
 * exynos_becore_input_producer_acquire() fills it with the balance the back end
 * *asked* for -- its stream configuration, with the greens at unity because
 * that is what the two gains it names are normalised against.  That is a seed
 * and not a statement: a producer whose gains differ from the ones it was
 * handed, for any reason including its own greens, has to overwrite this
 * before completing the buffer.  Every gain has to be within
 * EXYNOS_BECORE_WBG_GAIN_MIN_Q12 .. _MAX_Q12 or the buffer is refused.
 */
struct exynos_becore_input_buffer {
	dma_addr_t dma;
	size_t size;
	u64 cookie;
	unsigned int slot;
	struct exynos_becore_input_gains gains;
	/*
	 * Which frame this is, and when it ended, in the producer's own
	 * numbering.  Set by the producer when it completes a buffer and
	 * ignored on acquire.
	 *
	 * The consumer carries both to the frame it makes from this one,
	 * because a processed frame is that frame: it is what lets anything
	 * downstream pair the picture with the statistics metered from it and
	 * with the sensor settings it was taken at, neither of which the
	 * consumer has or should have.
	 *
	 * Optional.  producer_acquire() clears them, so a producer that has no
	 * frame numbering to offer -- or a path that hands a buffer on without
	 * one, which the ISPFE snapshot handoff does -- leaves them zero and
	 * the consumer numbers the frame itself.  Note that zero is not the
	 * signal for that: the consumer knows a producer filled the slot
	 * because it completed it, not because the number is set.
	 */
	u32 sequence;
	u64 timestamp;
};

struct exynos_becore_input *
exynos_becore_input_map(struct device *backend, struct device *producer,
			const struct exynos_becore_input_producer_ops *ops,
			void *producer_data);
void exynos_becore_input_disconnect(struct exynos_becore_input *input);
void exynos_becore_input_unmap(struct exynos_becore_input *input);

size_t exynos_becore_input_size(struct exynos_becore_input *input);

/*
 * Put the back end on the producer's media graph -- its input pad, and its
 * video nodes with it.
 *
 * The back end has to know the mosaic the front end is sending -- the demosaic
 * needs the CFA phase, which differs between this board's three cameras -- and
 * the front end has already negotiated it on its own pads.  Rather than pass
 * the format across this interface, where it would become a second source of
 * truth with no way to arbitrate, the back end registers a sink pad on the
 * producer's graph and reads the remote pad.
 *
 * The back end has no media device of its own, so the producer's is the one
 * media device the pipeline has, and the back end's nodes appear exactly while
 * a producer is bound.
 *
 * Call after the producer's own subdevice is registered and before its media
 * device is; the link is created enabled and immutable, because the two blocks
 * are wired to each other in silicon.
 *
 * On the way out two orderings matter, and neither is simply the reverse of
 * the above.  Call exynos_becore_input_disconnect() before unregistering the
 * graph, because taking the capture node down stops a stream and a stop calls
 * back into the producer; and unregister the graph before the producer's own
 * v4l2_device goes, because the two video nodes hold a reference to it that
 * their release path walks.  The media device may go either side of it.
 */
int exynos_becore_input_register_graph(struct exynos_becore_input *input,
				       struct v4l2_device *v4l2_dev,
				       struct media_entity *source,
				       u16 source_pad);
void exynos_becore_input_unregister_graph(struct exynos_becore_input *input);

int exynos_becore_input_producer_acquire(struct exynos_becore_input *input,
					 struct exynos_becore_input_buffer *buffer);
int exynos_becore_input_producer_complete(struct exynos_becore_input *input,
					  const struct exynos_becore_input_buffer *buffer);
void exynos_becore_input_producer_abort(struct exynos_becore_input *input,
					const struct exynos_becore_input_buffer *buffer);

#endif /* __MEDIA_EXYNOS_BECORE_H__ */
