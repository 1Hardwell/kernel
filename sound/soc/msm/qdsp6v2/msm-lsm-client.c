/*
 * Copyright (c) 2013, Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/timer.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/q6lsm.h>
#include <sound/lsm_params.h>
#include "msm-pcm-routing-v2.h"

struct lsm_priv {
	struct snd_pcm_substream *substream;
	struct lsm_client *lsm_client;

	struct snd_lsm_event_status *event_status;
	spinlock_t event_lock;
	wait_queue_head_t event_wait;
	unsigned long event_avail;
	atomic_t event_wait_stop;

	struct mutex lsm_api_lock;
};

static void lsm_event_handler(uint32_t opcode, uint32_t token,
			      void *payload, void *priv)
{
	unsigned long flags;
	struct snd_lsm_event_status *event_status;
	struct lsm_priv *prtd = priv;
	struct snd_pcm_substream *substream = prtd->substream;

	pr_debug("%s: enter opcode 0x%x\n", __func__, opcode);
	switch (opcode) {
	case LSM_SESSION_EVENT_DETECTION_STATUS:
		event_status = payload;

		spin_lock_irqsave(&prtd->event_lock, flags);
		prtd->event_status = krealloc(prtd->event_status,
					      sizeof(*event_status) +
					      event_status->payload_size,
					      GFP_ATOMIC);
		if (likely(prtd->event_status)) {
			memcpy(prtd->event_status, event_status,
			       sizeof(*event_status) +
			       event_status->payload_size);
			prtd->event_avail = 1;
			spin_unlock_irqrestore(&prtd->event_lock, flags);
			wake_up(&prtd->event_wait);
		} else {
			spin_unlock_irqrestore(&prtd->event_lock, flags);
			pr_err("%s: Couldn't allocate %d bytes of memory\n",
			       __func__, event_status->payload_size);
		}
		if (substream->timer_running)
			snd_timer_interrupt(substream->timer, 1);
		break;
	default:
		pr_debug("%s: Unsupported Event opcode 0x%x\n", __func__,
			 opcode);
		break;
	}
}

static int msm_lsm_ioctl(struct snd_pcm_substream *substream,
			 unsigned int cmd, void *arg)
{
	unsigned long flags;
	int ret;
	struct snd_lsm_sound_model snd_model;
	int rc = 0;
	int xchg = 0;
	int size = 0;
	struct snd_lsm_event_status *event_status = NULL;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lsm_priv *prtd = runtime->private_data;
	struct snd_lsm_event_status *user = arg;

	pr_debug("%s: enter cmd %x\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_LSM_REG_SND_MODEL:
		pr_debug("%s: Registering sound model\n", __func__);
		if (copy_from_user(&snd_model, (void *)arg,
				   sizeof(struct snd_lsm_sound_model))) {
			rc = -EFAULT;
			pr_err("%s: copy from user failed, size %d\n", __func__,
			       sizeof(struct snd_lsm_sound_model));
			break;
		}

		rc = q6lsm_snd_model_buf_alloc(prtd->lsm_client,
					       snd_model.data_size);
		if (rc) {
			pr_err("%s: q6lsm buffer alloc failed, size %d\n",
			       __func__, snd_model.data_size);
			break;
		}

		if (copy_from_user(prtd->lsm_client->sound_model.data,
				   snd_model.data, snd_model.data_size)) {
			pr_err("%s: copy from user data failed data %p size %d\n",
			       __func__, snd_model.data, snd_model.data_size);
			rc = -EFAULT;
			break;
		}

		rc = q6lsm_register_sound_model(prtd->lsm_client,
						snd_model.detection_mode,
						snd_model.min_keyw_confidence,
						snd_model.min_user_confidence,
						snd_model.detect_failure);
		if (rc < 0) {
			pr_err("%s: q6lsm_register_sound_model failed =%d\n",
			       __func__, rc);
			q6lsm_snd_model_buf_free(prtd->lsm_client);
		}

		break;

	case SNDRV_LSM_DEREG_SND_MODEL:
		pr_debug("%s: Deregistering sound model\n", __func__);
		rc = q6lsm_deregister_sound_model(prtd->lsm_client);
		break;

	case SNDRV_LSM_EVENT_STATUS:
		pr_debug("%s: Get event status\n", __func__);
		atomic_set(&prtd->event_wait_stop, 0);

		/*
		 * Release the api lock before wait to allow
		 * other IOCTLs to be invoked while waiting
		 * for event
		 */
		mutex_unlock(&prtd->lsm_api_lock);
		rc = wait_event_interruptible(prtd->event_wait,
				(cmpxchg(&prtd->event_avail, 1, 0) ||
				 (xchg = atomic_cmpxchg(&prtd->event_wait_stop,
							1, 0))));
		mutex_lock(&prtd->lsm_api_lock);
		pr_debug("%s: wait_event_interruptible %d event_wait_stop %d\n",
			 __func__, rc, xchg);
		if (!rc && !xchg) {
			pr_debug("%s: New event available %ld\n", __func__,
				 prtd->event_avail);
			spin_lock_irqsave(&prtd->event_lock, flags);
			if (prtd->event_status) {
				size = sizeof(*event_status) +
				       prtd->event_status->payload_size;
				event_status = kmemdup(prtd->event_status, size,
						       GFP_ATOMIC);
			}
			spin_unlock_irqrestore(&prtd->event_lock, flags);
			if (!event_status) {
				pr_err("%s: Couldn't allocate %d bytes\n",
				       __func__, size);
				/*
				 * Don't use -ENOMEM as userspace will check
				 * it for increasing buffer
				 */
				rc = -EFAULT;
			} else {
				if (!access_ok(VERIFY_READ, user,
					sizeof(struct snd_lsm_event_status)))
					rc = -EFAULT;
				if (user->payload_size <
				    event_status->payload_size) {
					pr_debug("%s: provided %dbytes isn't enough, needs %dbytes\n",
						 __func__, user->payload_size,
						 size);
					rc = -ENOMEM;
				} else if (!access_ok(VERIFY_WRITE, arg,
						      size)) {
					rc = -EFAULT;
				} else {
					rc = copy_to_user(arg, event_status,
							  size);
					if (rc)
						pr_err("%s: copy to user failed %d\n",
						       __func__, rc);
				}
				kfree(event_status);
			}
		} else if (xchg) {
			pr_debug("%s: Wait aborted\n", __func__);
			rc = 0;
		}
		break;

	case SNDRV_LSM_ABORT_EVENT:
		pr_debug("%s: Aborting event status wait\n", __func__);
		atomic_set(&prtd->event_wait_stop, 1);
		wake_up(&prtd->event_wait);
		break;

	case SNDRV_LSM_START:
		pr_debug("%s: Starting LSM client session\n", __func__);
		if (!prtd->lsm_client->started) {
			ret = q6lsm_start(prtd->lsm_client, true);
			if (!ret) {
				prtd->lsm_client->started = true;
				pr_debug("%s: LSM client session started\n",
					 __func__);
			}
		}
		break;

	case SNDRV_LSM_STOP:
		pr_debug("%s: Stopping LSM client session\n", __func__);
		if (prtd->lsm_client->started) {
			ret = q6lsm_stop(prtd->lsm_client, true);
			if (!ret)
				pr_debug("%s: LSM client session stopped %d\n",
					 __func__, ret);
			prtd->lsm_client->started = false;
		}
		break;

	default:
		pr_debug("%s: Falling into default snd_lib_ioctl cmd 0x%x\n",
			 __func__, cmd);
		rc = snd_pcm_lib_ioctl(substream, cmd, arg);
		break;
	}

	if (!rc)
		pr_debug("%s: leave (%d)\n", __func__, rc);
	else
		pr_err("%s: cmd 0x%x failed %d\n", __func__, cmd, rc);

	return rc;
}

#ifdef CONFIG_COMPAT
struct snd_lsm_sound_model32 {
	compat_uptr_t data;
	u32 data_size;
	enum lsm_detection_mode detection_mode;
	u16 min_keyw_confidence;
	u16 min_user_confidence;
	bool detect_failure;
};

struct snd_lsm_event_status32 {
	u16 status;
	u16 payload_size;
	u8 payload[0];
};

struct snd_lsm_sound_model_v2_32 {
	compat_uptr_t data;
	compat_uptr_t confidence_level;
	u32 data_size;
	enum lsm_detection_mode detection_mode;
	u8 num_confidence_levels;
	bool detect_failure;
};

enum {
	SNDRV_LSM_REG_SND_MODEL32 =
	_IOW('U', 0x00, struct snd_lsm_sound_model32),
	SNDRV_LSM_EVENT_STATUS32 =
	_IOW('U', 0x02, struct snd_lsm_event_status32),
	SNDRV_LSM_REG_SND_MODEL_V2_32 = _IOW('U', 0x07,
	struct snd_lsm_sound_model_v2_32),
};

static int msm_lsm_ioctl_compat(struct snd_pcm_substream *substream,
			  unsigned int cmd, void __user *arg)
{
	struct snd_pcm_runtime *runtime;
	int err = 0;
	u32 size = 0;

	if (PCM_RUNTIME_CHECK(substream))
		return -ENXIO;
	runtime = substream->runtime;
	switch (cmd) {
	case SNDRV_LSM_REG_SND_MODEL32: {
		struct snd_lsm_sound_model32 snd_model32;
		struct snd_lsm_sound_model snd_model;
		if (copy_from_user(&snd_model32, arg,
			sizeof(struct snd_lsm_sound_model32))) {
			err = -EFAULT;
			pr_err("%s: copy user failed ioctl %s, size %zd\n",
				__func__, "SNDRV_LSM_REG_SND_MODEL32",
				sizeof(struct snd_lsm_sound_model32));
		} else {
			snd_model.data = compat_ptr(snd_model32.data);
			snd_model.data_size = snd_model32.data_size;
			snd_model.detect_failure = snd_model32.detect_failure;
			snd_model.detection_mode = snd_model32.detection_mode;
			snd_model.min_keyw_confidence =
			snd_model32.min_keyw_confidence;
			snd_model.min_user_confidence =
			snd_model32.min_user_confidence;
			cmd = SNDRV_LSM_REG_SND_MODEL;
			err = msm_lsm_ioctl_shared(substream, cmd, &snd_model);
			if (err)
				pr_err("%s ioctl %s failed err %d\n",
				__func__, "SNDRV_LSM_REG_SND_MODEL32", err);
		}
		break;
	}
	case SNDRV_LSM_EVENT_STATUS32: {
		struct snd_lsm_event_status32 userarg32, *user32 = NULL;
		struct snd_lsm_event_status *user = NULL;
		if (copy_from_user(&userarg32, arg, sizeof(userarg32))) {
			pr_err("%s: err copyuser ioctl %s\n",
			__func__, "SNDRV_LSM_EVENT_STATUS32");
			return -EFAULT;
		}
		size = sizeof(*user) + userarg32.payload_size;
		user = kmalloc(size, GFP_KERNEL);
		if (!user) {
			pr_err("%s: Allocation failed event status size %d\n",
			__func__, size);
			return -EFAULT;
		} else {
			cmd = SNDRV_LSM_EVENT_STATUS;
			user->payload_size = userarg32.payload_size;
			err = msm_lsm_ioctl_shared(substream, cmd, user);
		}
		/* Update size with actual payload size */
		size = sizeof(userarg32) + user->payload_size;
		if (!err && !access_ok(VERIFY_WRITE, arg, size)) {
			pr_err("%s: write verify failed size %d\n",
			__func__, size);
			err = -EFAULT;
		}
		if (!err) {
			user32 = kmalloc(size, GFP_KERNEL);
			if (!user32) {
				pr_err("%s: Allocation event user status size %d\n"
				, __func__, size);
				err = -EFAULT;
			} else {
				user32->status = user->status;
				user32->payload_size = user->payload_size;
				memcpy(user32->payload,
				user->payload, user32->payload_size);
			}
		}
		if (!err && (copy_to_user(arg, user32, size))) {
			pr_err("%s: failed to copy payload %d",
			__func__, size);
			err = -EFAULT;
		}
		kfree(user);
		kfree(user32);
		if (err)
			pr_err("%s: lsmevent failed %d", __func__, err);
		break;
	}
	case SNDRV_LSM_REG_SND_MODEL_V2_32: {
		struct snd_lsm_sound_model_v2_32 snd_modelv232;
		struct snd_lsm_sound_model_v2 snd_modelv2;
		if (copy_from_user(&snd_modelv232, arg,
			sizeof(snd_modelv232))) {
			err = -EFAULT;
			pr_err("%s: copy user failed, size %zd %s\n", __func__,
			sizeof(struct snd_lsm_sound_model_v2_32),
			"SNDRV_LSM_REG_SND_MODEL_V2_32");
		} else {
			snd_modelv2.confidence_level =
			compat_ptr(snd_modelv232.confidence_level);
			snd_modelv2.data = compat_ptr(snd_modelv232.data);
			snd_modelv2.data_size = snd_modelv232.data_size;
			snd_modelv2.detect_failure =
			snd_modelv232.detect_failure;
			snd_modelv2.detection_mode =
			snd_modelv232.detection_mode;
			snd_modelv2.num_confidence_levels =
			snd_modelv232.num_confidence_levels;
			cmd = SNDRV_LSM_REG_SND_MODEL_V2;
			err = msm_lsm_ioctl_shared(substream, cmd,
				&snd_modelv2);
			if (err)
				pr_err("%s: ioctl %s failed\n", __func__,
				"SNDDRV_LSM_REG_SND_MODEL_V2_32");
		}
		break;
	}
	default:
		err = msm_lsm_ioctl_shared(substream, cmd, arg);
		break;
	}
	return err;
}
#else
#define msm_lsm_ioctl_compat NULL
#endif

static int msm_lsm_ioctl(struct snd_pcm_substream *substream,
			 unsigned int cmd, void *arg)
{
	struct snd_pcm_runtime *runtime;
	struct lsm_priv *prtd;
	struct snd_soc_pcm_runtime *rtd;
	int err = 0;
	u32 size = 0;
	struct snd_lsm_session_data session_data;

	if (!substream || !substream->private_data) {
		pr_err("%s: Invalid params\n", __func__);
		return -EINVAL;
	}
	runtime = substream->runtime;
	rtd = substream->private_data;
	prtd = runtime->private_data;
	mutex_lock(&prtd->lsm_api_lock);
	switch (cmd) {
	case SNDRV_LSM_SET_SESSION_DATA:
		pr_debug("%s: SNDRV_LSM_SET_SESSION_DATA\n", __func__);
		if (copy_from_user(&session_data, (void *)arg,
				   sizeof(struct snd_lsm_session_data))) {
			err = -EFAULT;
			pr_err("%s: copy from user failed, size %d\n",
			       __func__, sizeof(struct snd_lsm_session_data));
			break;
		}
		if (!err)
			err = msm_lsm_ioctl_shared(substream,
						   cmd, &session_data);
		if (err)
			pr_err("%s REG_SND_MODEL failed err %d\n",
			__func__, err);
		break;
	case SNDRV_LSM_REG_SND_MODEL_V2: {
		struct snd_lsm_sound_model_v2 snd_model_v2;
		if (!arg) {
			pr_err("%s: Invalid params snd_model\n", __func__);
			return -EINVAL;
		}
		if (copy_from_user(&snd_model_v2, arg, sizeof(snd_model_v2))) {
			err = -EFAULT;
			pr_err("%s: copy from user failed, size %zd\n",
			__func__, sizeof(struct snd_lsm_sound_model_v2));
		}
		if (!err)
			err = msm_lsm_ioctl_shared(substream, cmd,
						   &snd_model_v2);
		if (err)
			pr_err("%s REG_SND_MODEL failed err %d\n",
			__func__, err);
		return err;
		}
		break;
	case SNDRV_LSM_REG_SND_MODEL: {
		struct snd_lsm_sound_model snd_model;
		pr_debug("%s: SNDRV_LSM_REG_SND_MODEL\n", __func__);
		if (!arg) {
			pr_err("%s: Invalid params snd_model\n", __func__);
			err = -EINVAL;
			goto done;
		}
		if (copy_from_user(&snd_model, arg, sizeof(snd_model))) {
			err = -EFAULT;
			pr_err("%s: copy from user failed, size %zd\n",
			__func__, sizeof(struct snd_lsm_sound_model));
		}
		if (!err)
			err = msm_lsm_ioctl_shared(substream, cmd, &snd_model);
		if (err)
			pr_err("%s REG_SND_MODEL failed err %d\n",
			__func__, err);
		goto done;
	}
	case SNDRV_LSM_EVENT_STATUS: {
		struct snd_lsm_event_status *user = NULL, userarg;
		pr_debug("%s: SNDRV_LSM_EVENT_STATUS\n", __func__);
		if (!arg) {
			pr_err("%s: Invalid params event status\n", __func__);
			err = -EINVAL;
			goto done;
		}
		if (copy_from_user(&userarg, arg, sizeof(userarg))) {
			pr_err("%s: err copyuser event_status\n",
			__func__);
			err = -EFAULT;
			goto done;
		}
		size = sizeof(struct snd_lsm_event_status) +
		userarg.payload_size;
		user = kmalloc(size, GFP_KERNEL);
		if (!user) {
			pr_err("%s: Allocation failed event status size %d\n",
			__func__, size);
			return -EFAULT;
		} else {
			user->payload_size = userarg.payload_size;
			err = msm_lsm_ioctl_shared(substream, cmd, user);
		}
		/* Update size with actual payload size */
		size = sizeof(*user) + user->payload_size;
		if (!err && !access_ok(VERIFY_WRITE, arg, size)) {
			pr_err("%s: write verify failed size %d\n",
			__func__, size);
			err = -EFAULT;
		}
		if (!err && (copy_to_user(arg, user, size))) {
			pr_err("%s: failed to copy payload %d",
			__func__, size);
			err = -EFAULT;
		}
		kfree(user);
		if (err)
			pr_err("%s: lsmevent failed %d", __func__, err);
		goto done;
	}
	default:
		err = msm_lsm_ioctl_shared(substream, cmd, arg);
	break;
	}
done:
	mutex_unlock(&prtd->lsm_api_lock);
	return err;
}

static int msm_lsm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lsm_priv *prtd;
	int ret = 0;

	pr_debug("%s\n", __func__);
	prtd = kzalloc(sizeof(struct lsm_priv), GFP_KERNEL);
	if (!prtd) {
		pr_err("%s: Failed to allocate memory for lsm_priv\n",
		       __func__);
		return -ENOMEM;
	}
	prtd->substream = substream;
	prtd->lsm_client = q6lsm_client_alloc(
				(lsm_app_cb)lsm_event_handler, prtd);
	if (!prtd->lsm_client) {
		pr_err("%s: Could not allocate memory\n", __func__);
		kfree(prtd);
		return -ENOMEM;
	}
	ret = q6lsm_open(prtd->lsm_client);
	if (ret < 0) {
		pr_err("%s: lsm open failed, %d\n", __func__, ret);
		kfree(prtd);
		return ret;
	}
	prtd->lsm_client->opened = true;

	pr_debug("%s: Session ID %d\n", __func__, prtd->lsm_client->session);
	prtd->lsm_client->started = false;
	spin_lock_init(&prtd->event_lock);
	init_waitqueue_head(&prtd->event_wait);
	mutex_init(&prtd->lsm_api_lock);
	runtime->private_data = prtd;

	prtd->lsm_client->opened = false;
	return 0;
}

static int msm_lsm_close(struct snd_pcm_substream *substream)
{
	unsigned long flags;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lsm_priv *prtd = runtime->private_data;
	int ret = 0;

	pr_debug("%s\n", __func__);
	if (prtd->lsm_client->started) {
		ret = q6lsm_stop(prtd->lsm_client, true);
		if (ret)
			pr_err("%s: session stop failed, err = %d\n",
				__func__, ret);
		else
			pr_debug("%s: LSM client session stopped %d\n",
				 __func__, ret);

		/*
		 * Go Ahead and try de-register sound model,
		 * even if stop failed
		 */
		prtd->lsm_client->started = false;

		ret = q6lsm_deregister_sound_model(prtd->lsm_client);
		if (ret)
			pr_err("%s: dereg_snd_model failed, err = %d\n",
				__func__, ret);
		else
			pr_debug("%s: dereg_snd_model succesful\n",
				 __func__);
	}

	if (prtd->lsm_client->opened) {
		q6lsm_close(prtd->lsm_client);
		prtd->lsm_client->opened = false;
	}
	q6lsm_client_free(prtd->lsm_client);

	spin_lock_irqsave(&prtd->event_lock, flags);
	kfree(prtd->event_status);
	prtd->event_status = NULL;
	spin_unlock_irqrestore(&prtd->event_lock, flags);
	mutex_destroy(&prtd->lsm_api_lock);
	kfree(prtd);
	runtime->private_data = NULL;

	return 0;
}

static struct snd_pcm_ops msm_lsm_ops = {
	.open           = msm_lsm_open,
	.close          = msm_lsm_close,
	.ioctl          = msm_lsm_ioctl,
};

static int msm_asoc_lsm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	return 0;
}

static int msm_asoc_lsm_probe(struct snd_soc_platform *platform)
{
	pr_debug("enter %s\n", __func__);

	return 0;
}

static struct snd_soc_platform_driver msm_soc_platform = {
	.ops		= &msm_lsm_ops,
	.pcm_new	= msm_asoc_lsm_new,
	.probe		= msm_asoc_lsm_probe,
};

static __devinit int msm_lsm_probe(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", "msm-lsm-client");

	return snd_soc_register_platform(&pdev->dev, &msm_soc_platform);
}

static int msm_lsm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);

	return 0;
}

static const struct of_device_id msm_lsm_client_dt_match[] = {
	{.compatible = "qcom,msm-lsm-client" },
	{ }
};

static struct platform_driver msm_lsm_driver = {
	.driver = {
		.name = "msm-lsm-client",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(msm_lsm_client_dt_match),
	},
	.probe = msm_lsm_probe,
	.remove = __devexit_p(msm_lsm_remove),
};

static int __init msm_soc_platform_init(void)
{
	return platform_driver_register(&msm_lsm_driver);
}
module_init(msm_soc_platform_init);

static void __exit msm_soc_platform_exit(void)
{
	platform_driver_unregister(&msm_lsm_driver);
}
module_exit(msm_soc_platform_exit);

MODULE_DESCRIPTION("LSM client platform driver");
MODULE_DEVICE_TABLE(of, msm_lsm_client_dt_match);
MODULE_LICENSE("GPL v2");
