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
/* What the green channels are held at, and so what normalises the other two. */
#define EXYNOS_BECORE_WBG_UNITY_Q12		4096U
#define EXYNOS_BECORE_WBG_RED_DEFAULT_Q12	8473U
#define EXYNOS_BECORE_WBG_BLUE_DEFAULT_Q12	6851U

/* Stream-latched semantic policy passed to the attached front end. */
struct exynos_becore_input_stream_config {
	/* Unsigned Q12 gains; green channels remain normalized to unity. */
	u32 red_balance;
	u32 blue_balance;
};

struct exynos_becore_input_producer_ops {
	int (*start_streaming)(void *data,
			       const struct exynos_becore_input_stream_config *config);
	void (*stop_streaming)(void *data);
};

/* One driver-owned input slot reserved for a front-end producer. */
struct exynos_becore_input_buffer {
	dma_addr_t dma;
	size_t size;
	u64 cookie;
	unsigned int slot;
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
