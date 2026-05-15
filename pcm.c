// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linux driver for ZOOM devices (L-8 only at the moment)
 *
 * Copyright 2021 (C) Sebastian Reimers
 * Copyright 2026 (C) Dennis Duda
 *
 * Authors:  Sebastian Reimers <hallo@studio-link.de>
 *           Dennis Duda <git@seri.tools>
 */

#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "pcm.h"
#include "driver.h"

#define IN_EP           0x82
#define OUT_EP          0x01
#define PCM_N_URBS      8
#define PCM_URB_SIZE    512
#define PCM_PACKET_SIZE (4 * 4) /* 32 Bit x Frames/URB */

struct pcm_urb {
	struct zoom_chip *chip;

	struct urb instance;
	u8 *buffer;
};

struct pcm_substream {
	spinlock_t lock;
	struct snd_pcm_substream *instance;

	bool active;
	snd_pcm_uframes_t dma_off;    /* current position in alsa dma_area */
	snd_pcm_uframes_t period_off; /* current position in current period */
};

enum { /* pcm streaming states */
	STREAM_DISABLED, /* no pcm streaming */
	STREAM_STARTING, /* pcm streaming requested, waiting to become ready */
	STREAM_RUNNING,  /* pcm streaming running */
	STREAM_STOPPING
};

struct pcm_runtime {
	struct zoom_chip *chip;
	struct snd_pcm *instance;

	struct pcm_substream playback;
	struct pcm_substream capture;
	bool panic; /* if set driver won't do anymore pcm on device */

	struct pcm_urb out_urbs[PCM_N_URBS];
	struct pcm_urb in_urbs[PCM_N_URBS];

	struct mutex stream_mutex;
	u8 stream_state; /* one of STREAM_XXX */
	wait_queue_head_t stream_wait_queue;
	bool stream_wait_cond;
};

static const struct snd_pcm_hardware pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH,

	.formats = SNDRV_PCM_FMTBIT_S32_LE,

	// rates/min/max are set from zoom_chip
	.channels_min = 2,
	.channels_max = 4,
	.buffer_bytes_max = 1024 * 1024,
	.period_bytes_min = PCM_PACKET_SIZE * 2,
	.period_bytes_max = 512 * 1024,
	.periods_min = 2,
	.periods_max = 1024
};

static const struct snd_pcm_hardware pcm_hw_rec = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH,

	.formats = SNDRV_PCM_FMTBIT_S32_LE,

	// rates/min/max are set from zoom_chip
	.channels_min = 1,
	.channels_max = 12,
	.buffer_bytes_max = 1024 * 1024,
	.period_bytes_min = PCM_PACKET_SIZE * 1,
	.period_bytes_max = 512 * 1024,
	.periods_min = 2,
	.periods_max = 1024
};

static struct pcm_substream *zoom_pcm_get_substream(
	struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct device *device = &rt->chip->dev->dev;

	if (alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return &rt->playback;

	if (alsa_sub->stream == SNDRV_PCM_STREAM_CAPTURE)
		return &rt->capture;

	dev_err(device, "Error getting pcm substream slot.\n");
	return NULL;
}

/* call with stream_mutex locked */
static void zoom_pcm_stream_stop(struct pcm_runtime *rt)
{
	int i;

	if (READ_ONCE(rt->stream_state) != STREAM_DISABLED) {
		WRITE_ONCE(rt->stream_state, STREAM_STOPPING);

		for (i = 0; i < PCM_N_URBS; i++) {
			usb_kill_urb(&rt->out_urbs[i].instance);
			usb_kill_urb(&rt->in_urbs[i].instance);
		}

		WRITE_ONCE(rt->stream_state, STREAM_DISABLED);
	}
}

/*
 * Resets the USB interfaces as observed from the proprietary driver.
 *
 * The windows driver resets the interfaces in this exact order.
 *
 * call with stream_mutex locked
 */
static int zoom_interface_init(struct pcm_runtime *rt)
{
	int ret;

	ret = usb_set_interface(rt->chip->dev, 1, 0); /* EP1 OUT =    OFF */
	if (ret != 0)
		goto error;
	ret = usb_set_interface(rt->chip->dev, 2, 0); /* EP2  IN =    OFF */
	if (ret != 0)
		goto error;
	ret = usb_set_interface(rt->chip->dev, 2, 3); /* EP2  IN = 32 bit */
	if (ret != 0)
		goto error;
	ret = usb_set_interface(rt->chip->dev, 1, 3); /* EP1 OUT = 32 bit */
	if (ret != 0)
		goto error;

	return 0;

error:
	dev_err(&rt->chip->dev->dev, "can't set interface for device (ret=%d)\n", ret);
	return ret;
}

/* call with stream_mutex locked */
static int zoom_pcm_stream_start(struct pcm_runtime *rt)
{
	int ret = 0;
	int i;

	if (READ_ONCE(rt->stream_state) == STREAM_DISABLED) {
		/* reset panic and wait condition when starting a new stream */
		WRITE_ONCE(rt->panic, false);
		rt->stream_wait_cond = false;

		ret = zoom_interface_init(rt);
		if (ret)
			return ret;

		/* submit our out urbs zero init */
		WRITE_ONCE(rt->stream_state, STREAM_STARTING);
		for (i = 0; i < PCM_N_URBS; i++) {
			memset(rt->out_urbs[i].buffer, 0, PCM_URB_SIZE);
			ret = usb_submit_urb(&rt->out_urbs[i].instance,
					     GFP_ATOMIC);
			if (ret) {
				zoom_pcm_stream_stop(rt);
				return ret;
			}

			ret = usb_submit_urb(&rt->in_urbs[i].instance,
					     GFP_ATOMIC);
			if (ret) {
				zoom_pcm_stream_stop(rt);
				return ret;
			}
		}

		/* wait for first out urb to return (sent in in urb handler) */
		wait_event_timeout(rt->stream_wait_queue, rt->stream_wait_cond,
				   HZ);
		if (rt->stream_wait_cond) {
			struct device *device = &rt->chip->dev->dev;
			dev_dbg(device, "%s: Stream is running wakeup event\n",
				__func__);
			WRITE_ONCE(rt->stream_state, STREAM_RUNNING);
		} else {
			zoom_pcm_stream_stop(rt);
			return -EIO;
		}
	}
	return ret;
}

static void memcpy_pcm_capture(u8 *dest, u8 *src, u8 ch_sz, unsigned int skip,
			       unsigned int len)
{
	unsigned int frame, b, o = 0;

	for (frame = 0; frame < PCM_URB_SIZE / 128; frame++) {
		unsigned int base = frame * 128;

		for (b = 0; b < ch_sz; b++) {
			if (skip) {
				skip--;
				continue;
			}
			if (len && o >= len)
				return;
			dest[o++] = src[base + b];
		}
	}
}

static void memcpy_pcm_playback(u8 *dest, u8 *src, u8 ch_sz, unsigned int skip,
				unsigned int len)
{
	unsigned int frame, b, o = 0;

	for (frame = 0; frame < PCM_URB_SIZE / 128; frame++) {
		unsigned int base = frame * 128;

		for (b = 0; b < ch_sz; b++) {
			if (skip) {
				skip--;
				continue;
			}
			if (len && o >= len)
				return;
			dest[base + b] = src[o++];
		}
	}
}

/* call with substream locked */
/* returns true if a period elapsed */
static bool zoom_pcm_capture(struct pcm_substream *sub, struct pcm_urb *urb)
{
	struct snd_pcm_runtime *alsa_rt = sub->instance->runtime;
	struct device *device = &urb->chip->dev->dev;
	u8 *dest;
	u8 ch_sz = alsa_rt->channels * 4; /* 32Bit */
	unsigned int pcm_buffer_size, pcm_len, len;

	pcm_buffer_size = snd_pcm_lib_buffer_bytes(sub->instance);

	pcm_len = ch_sz * 4; /* Channel size * 4 Frames */

	if (sub->dma_off + pcm_len <= pcm_buffer_size) {
		dev_dbg(device,
			"%s: (1) buffer_size %#x dma_offset %#x\n", __func__,
			(unsigned int)pcm_buffer_size,
			(unsigned int)sub->dma_off);

		dest = alsa_rt->dma_area + sub->dma_off;
		memcpy_pcm_capture(dest, urb->buffer, ch_sz, 0, 0);
	} else {
		/* wrap around at end of ring buffer */
		dev_dbg(device,
			"%s: (2) buffer_size %#x dma_offset %#x\n", __func__,
			(unsigned int)pcm_buffer_size,
			(unsigned int)sub->dma_off);

		len = pcm_buffer_size - sub->dma_off;
		dest = alsa_rt->dma_area + sub->dma_off;
		memcpy_pcm_capture(dest, urb->buffer, ch_sz, 0, len);

		dest = alsa_rt->dma_area;
		memcpy_pcm_capture(dest, urb->buffer, ch_sz, len,
				   pcm_len - len);
	}
	sub->dma_off += pcm_len;
	if (sub->dma_off >= pcm_buffer_size)
		sub->dma_off -= pcm_buffer_size;

	sub->period_off += pcm_len;
	if (sub->period_off >= alsa_rt->period_size) {
		sub->period_off %= alsa_rt->period_size;
		return true;
	}
	return false;
}

/* call with substream locked */
/* returns true if a period elapsed */
static bool zoom_pcm_playback(struct pcm_substream *sub, struct pcm_urb *urb)
{
	struct snd_pcm_runtime *alsa_rt = sub->instance->runtime;
	struct device *device = &urb->chip->dev->dev;
	u8 *source;
	u8 ch_sz = alsa_rt->channels * 4; /* 32Bit */
	unsigned int pcm_buffer_size, pcm_len, len;

	pcm_buffer_size = snd_pcm_lib_buffer_bytes(sub->instance);

	pcm_len = ch_sz * 4; /* Channel size * 4 Frames */

	if (sub->dma_off + pcm_len <= pcm_buffer_size) {
		dev_dbg(device, "%s: (1) buffer_size %#x dma_offset %#x\n", __func__,
			(unsigned int)pcm_buffer_size,
			(unsigned int)sub->dma_off);

		source = alsa_rt->dma_area + sub->dma_off;
		memcpy_pcm_playback(urb->buffer, source, ch_sz, 0, 0);
	} else {
		/* wrap around at end of ring buffer */
		dev_dbg(device, "%s: (2) buffer_size %#x dma_offset %#x\n", __func__,
			(unsigned int)pcm_buffer_size,
			(unsigned int)sub->dma_off);

		len = pcm_buffer_size - sub->dma_off;
		source = alsa_rt->dma_area + sub->dma_off;
		memcpy_pcm_playback(urb->buffer, source, ch_sz, 0, len);

		source = alsa_rt->dma_area;
		memcpy_pcm_playback(urb->buffer, source, ch_sz, len,
				    pcm_len - len);
	}
	sub->dma_off += pcm_len;
	if (sub->dma_off >= pcm_buffer_size)
		sub->dma_off -= pcm_buffer_size;

	sub->period_off += pcm_len;
	if (sub->period_off >= alsa_rt->period_size) {
		sub->period_off %= alsa_rt->period_size;
		return true;
	}
	return false;
}

static void zoom_pcm_in_urb_handler(struct urb *usb_urb)
{
	struct pcm_urb *in_urb = usb_urb->context;
	struct pcm_runtime *rt = in_urb->chip->pcm;
	struct device *device = &rt->chip->dev->dev;
	struct pcm_substream *sub;
	bool do_period_elapsed = false;
	int ret;

	if (READ_ONCE(rt->panic) ||
	    READ_ONCE(rt->stream_state) == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||	/* unlinked */
		     usb_urb->status == -ENODEV ||	/* device removed */
		     usb_urb->status == -ECONNRESET ||	/* unlinked */
		     usb_urb->status == -ESHUTDOWN)) {	/* device disabled */
		dev_err(device, "%s: panicking on urb status=%d\n",
			__func__, usb_urb->status);
		goto out_fail;
	}

	if (unlikely(usb_urb->status))
		dev_warn_ratelimited(device, "%s: urb status=%d (continuing)\n",
				     __func__, usb_urb->status);

	sub = &rt->capture;
	scoped_guard(spinlock_irqsave, &sub->lock) {
		if (sub->active)
			do_period_elapsed = zoom_pcm_capture(sub, in_urb);
	}
	if (do_period_elapsed)
		snd_pcm_period_elapsed(sub->instance);

	ret = usb_submit_urb(&in_urb->instance, GFP_ATOMIC);
	if (ret < 0) {
		dev_err(device, "%s: resubmit failed ret=%d\n", __func__, ret);
		goto out_fail;
	}

	return;

out_fail:
	WRITE_ONCE(rt->panic, true);
}

static void zoom_pcm_out_urb_handler(struct urb *usb_urb)
{
	struct pcm_urb *out_urb = usb_urb->context;
	struct pcm_runtime *rt = out_urb->chip->pcm;
	struct device *device = &rt->chip->dev->dev;
	struct pcm_substream *sub;
	bool do_period_elapsed = false;
	int ret;

	if (READ_ONCE(rt->panic) ||
	    READ_ONCE(rt->stream_state) == STREAM_STOPPING)
		return;

	if (unlikely(usb_urb->status == -ENOENT ||	/* unlinked */
		     usb_urb->status == -ENODEV ||	/* device removed */
		     usb_urb->status == -ECONNRESET ||	/* unlinked */
		     usb_urb->status == -ESHUTDOWN)) {	/* device disabled */
		dev_err(device, "%s: panicking on urb status=%d\n",
			__func__, usb_urb->status);
		goto out_fail;
	}

	if (unlikely(usb_urb->status))
		dev_warn_ratelimited(device, "%s: urb status=%d (continuing)\n",
				     __func__, usb_urb->status);

	if (READ_ONCE(rt->stream_state) == STREAM_STARTING) {
		rt->stream_wait_cond = true;
		wake_up(&rt->stream_wait_queue);
	}

	/* now send our playback data (if a free out urb was found) */
	sub = &rt->playback;
	scoped_guard(spinlock_irqsave, &sub->lock) {
		if (sub->active)
			do_period_elapsed = zoom_pcm_playback(sub, out_urb);
		else
			memset(out_urb->buffer, 0, PCM_URB_SIZE);
	}

	if (do_period_elapsed)
		snd_pcm_period_elapsed(sub->instance);

	ret = usb_submit_urb(&out_urb->instance, GFP_ATOMIC);
	if (ret < 0) {
		dev_err(device, "%s: resubmit failed ret=%d\n", __func__, ret);
		goto out_fail;
	}

	return;

out_fail:
	WRITE_ONCE(rt->panic, true);
}

static int zoom_pcm_hw_channel_rule(struct snd_pcm_hw_params *params,
				     struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *period_bytes =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
	const struct snd_interval *channels =
		hw_param_interval_c(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval t;

	snd_interval_any(&t);
	t.min = PCM_PACKET_SIZE * channels->min;
	t.integer = 1;

	return snd_interval_refine(period_bytes, &t);
}

static int zoom_pcm_open(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct pcm_substream *sub = NULL;
	struct snd_pcm_runtime *alsa_rt = alsa_sub->runtime;

	/* panic is recovered in prepare via stream_start; don't block opens */

	guard(mutex)(&rt->stream_mutex);

	if (alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		alsa_rt->hw = pcm_hw;
		sub = &rt->playback;
	}

	if (alsa_sub->stream == SNDRV_PCM_STREAM_CAPTURE) {
		alsa_rt->hw = pcm_hw_rec;
		sub = &rt->capture;
	}

	if (!sub) {
		struct device *device = &rt->chip->dev->dev;
		dev_err(device, "Invalid stream type\n");
		return -EINVAL;
	}

	alsa_rt->hw.rates = snd_pcm_rate_to_rate_bit(rt->chip->rate);
	alsa_rt->hw.rate_min = rt->chip->rate;
	alsa_rt->hw.rate_max = rt->chip->rate;

	sub->instance = alsa_sub;
	sub->active = false;

	snd_pcm_hw_rule_add(alsa_rt, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
			    zoom_pcm_hw_channel_rule, NULL,
			    SNDRV_PCM_HW_PARAM_CHANNELS, -1);

	return 0;
}

static int zoom_pcm_close(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct pcm_substream *sub = zoom_pcm_get_substream(alsa_sub);

	guard(mutex)(&rt->stream_mutex);
	if (sub) {
		struct pcm_substream *other =
			(sub == &rt->playback) ? &rt->capture : &rt->playback;

		/* only tear down URBs once the last substream is closed */
		if (!other->instance)
			zoom_pcm_stream_stop(rt);

		/* deactivate substream */
		guard(spinlock_irqsave)(&sub->lock);
		sub->instance = NULL;
		sub->active = false;
	}
	return 0;
}

static int zoom_pcm_prepare(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	struct pcm_substream *sub = zoom_pcm_get_substream(alsa_sub);
	int ret;

	if (!sub)
		return -ENODEV;

	guard(mutex)(&rt->stream_mutex);

	/* recover from panic: drain the zombie URB chain so the
	 * stream_start below can re-init the device cleanly */
	if (READ_ONCE(rt->panic))
		zoom_pcm_stream_stop(rt);

	scoped_guard(spinlock_irqsave, &sub->lock) {
		sub->dma_off = 0;
		sub->period_off = 0;
		sub->active = false;
	}

	if (READ_ONCE(rt->stream_state) == STREAM_DISABLED) {
		ret = zoom_pcm_stream_start(rt);
		if (ret)
			return ret;
	}
	return 0;
}

static int zoom_pcm_trigger(struct snd_pcm_substream *alsa_sub, int cmd)
{
	struct pcm_substream *sub = zoom_pcm_get_substream(alsa_sub);
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);

	if (READ_ONCE(rt->panic))
		return -EPIPE;
	if (!sub)
		return -ENODEV;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		scoped_guard(spinlock_irqsave, &sub->lock) {
			sub->active = true;
		}
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		scoped_guard(spinlock_irqsave, &sub->lock) {
			sub->active = false;
		}
		return 0;

	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t zoom_pcm_pointer(struct snd_pcm_substream *alsa_sub)
{
	struct pcm_substream *sub = zoom_pcm_get_substream(alsa_sub);
	struct pcm_runtime *rt = snd_pcm_substream_chip(alsa_sub);
	snd_pcm_uframes_t dma_offset;

	if (READ_ONCE(rt->panic) || !sub)
		return SNDRV_PCM_POS_XRUN;

	guard(spinlock_irqsave)(&sub->lock);
	dma_offset = sub->dma_off;
	return bytes_to_frames(alsa_sub->runtime, dma_offset);
}

static const struct snd_pcm_ops pcm_ops = {
	.open = zoom_pcm_open,
	.close = zoom_pcm_close,
	.prepare = zoom_pcm_prepare,
	.trigger = zoom_pcm_trigger,
	.pointer = zoom_pcm_pointer,
};

static int zoom_pcm_init_urb(struct pcm_urb *urb, struct zoom_chip *chip,
			     unsigned int pipe,
			     void (*handler)(struct urb *))
{
	urb->chip = chip;
	usb_init_urb(&urb->instance);

	urb->buffer = kzalloc(PCM_URB_SIZE, GFP_KERNEL);
	if (!urb->buffer)
		return -ENOMEM;

	usb_fill_bulk_urb(&urb->instance, chip->dev, pipe, urb->buffer,
			  PCM_URB_SIZE, handler, urb);
	if (usb_urb_ep_type_check(&urb->instance))
		return -EINVAL;

	return 0;
}

void zoom_pcm_abort(struct zoom_chip *chip)
{
	struct pcm_runtime *rt = chip->pcm;
	int i;

	if (rt) {
		WRITE_ONCE(rt->panic, true);

		/* poison instead of kill so any racing handler that tries
		 * to resubmit gets -EPERM and gives up */
		for (i = 0; i < PCM_N_URBS; i++) {
			usb_poison_urb(&rt->out_urbs[i].instance);
			usb_poison_urb(&rt->in_urbs[i].instance);
		}
	}
}

static void zoom_pcm_destroy(struct zoom_chip *chip)
{
	struct pcm_runtime *rt = chip->pcm;
	int i;

	/* defensive: ensure no URB is in flight before freeing its buffer */
	for (i = 0; i < PCM_N_URBS; i++) {
		usb_kill_urb(&rt->out_urbs[i].instance);
		usb_kill_urb(&rt->in_urbs[i].instance);
	}

	for (i = 0; i < PCM_N_URBS; i++) {
		kfree(rt->out_urbs[i].buffer);
		kfree(rt->in_urbs[i].buffer);
	}

	kfree(chip->pcm);
	chip->pcm = NULL;
}

static void zoom_pcm_free(struct snd_pcm *pcm)
{
	struct pcm_runtime *rt = pcm->private_data;

	if (rt)
		zoom_pcm_destroy(rt->chip);
}

int zoom_pcm_init(struct zoom_chip *chip)
{
	int i;
	int ret;
	struct snd_pcm *pcm;
	struct pcm_runtime *rt;

	rt = kzalloc_obj(*rt);
	if (!rt)
		return -ENOMEM;

	rt->chip = chip;
	rt->stream_state = STREAM_DISABLED;

	init_waitqueue_head(&rt->stream_wait_queue);
	mutex_init(&rt->stream_mutex);
	spin_lock_init(&rt->playback.lock);
	spin_lock_init(&rt->capture.lock);

	ret = zoom_interface_init(rt);
	if (ret)
		goto error_free_rt;

	for (i = 0; i < PCM_N_URBS; i++) {
		ret = zoom_pcm_init_urb(&rt->out_urbs[i], chip,
					usb_sndbulkpipe(chip->dev, OUT_EP),
					zoom_pcm_out_urb_handler);
		if (ret < 0)
			goto error_free_urbs;
		ret = zoom_pcm_init_urb(&rt->in_urbs[i], chip,
					usb_rcvbulkpipe(chip->dev, IN_EP),
					zoom_pcm_in_urb_handler);
		if (ret < 0)
			goto error_free_urbs;
	}

	ret = snd_pcm_new(chip->card, "USB Audio", 0, 1, 1, &pcm);
	if (ret < 0) {
		dev_err(&chip->dev->dev, "Cannot create pcm instance\n");
		goto error_free_urbs;
	}

	pcm->private_data = rt;
	pcm->private_free = zoom_pcm_free;

	strscpy(pcm->name, "USB Audio", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &pcm_ops);
	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL, 0, 0);

	rt->instance = pcm;

	chip->pcm = rt;
	return 0;

error_free_urbs:
	for (i = 0; i < PCM_N_URBS; i++) {
		kfree(rt->out_urbs[i].buffer);
		kfree(rt->in_urbs[i].buffer);
	}
error_free_rt:
	kfree(rt);
	return ret;
}
