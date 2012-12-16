/*
 * OpenFIMG Kernel Interface
 *
 * Copyright 2012 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>

#include <video/openfimg.h>

/*
 * Various definitions
 */
#define G3D_AUTOSUSPEND_DELAY		(100)

/*
 * Registers
 */
#define G3D_FGGB_PIPESTAT_REG		(0x00)
#define G3D_FGGB_PIPESTAT_MSK		(0x0005171f)

#define G3D_FGGB_CACHECTL_REG		(0x04)
#define G3D_FGGB_FLUSH_MSK		(0x00000033)
#define G3D_FGGB_INVAL_MSK		(0x00001300)

#define G3D_FGGB_RESET_REG		(0x08)
#define G3D_FGGB_VERSION		(0x10)
#define G3D_FGGB_INTPENDING_REG		(0x40)
#define G3D_FGGB_INTMASK_REG		(0x44)
#define G3D_FGGB_PIPEMASK_REG		(0x48)
#define G3D_FGGB_PIPETGTSTATE_REG	(0x4c)

/*
 * Private data types
 */
struct g3d_context;

struct g3d_drvdata {
	struct miscdevice	mdev;
	void __iomem		*base;
	int			irq;
	struct resource 	*mem;
	struct clk		*clock;
	struct device		*dev;
	struct task_struct	*thread;

	spinlock_t		request_lock;
	struct list_head	request_queue;
	struct kmem_cache	*request_cache;
	wait_queue_head_t	request_wq;

	struct g3d_context	*last_ctx;
	struct kset		ctx_kset;
};

struct g3d_context {
	struct kobject		kobj;

	struct g3d_drvdata	*g3d;

	spinlock_t		fence_lock;
	struct list_head	fence_queue;
	wait_queue_head_t	fence_wq;

	struct list_head	state_buf_queue;
};

struct g3d_request {
	struct g3d_user_request usr;
	struct g3d_context	*ctx;
	struct list_head	list;
};

typedef void *g3d_state_buffer_t;
typedef void *g3d_command_buffer_t;

typedef int (*g3d_request_handler_t)(struct g3d_request *);

struct g3d_request_info {
	g3d_request_handler_t handler;
};

/*
 * Register accessors
 */
static inline void g3d_write(struct g3d_drvdata *d, uint32_t b, uint32_t r)
{
	writel(b, d->base + r);
}

static inline uint32_t g3d_read(struct g3d_drvdata *d, uint32_t r)
{
	return readl(d->base + r);
}

/*
 * Hardware operations
 */
static inline void g3d_soft_reset(struct g3d_drvdata *g3d)
{
	g3d_write(g3d, 1, G3D_FGGB_RESET_REG);
	udelay(1);
	g3d_write(g3d, 0, G3D_FGGB_RESET_REG);
}

static void g3d_flush_caches(struct g3d_drvdata *g3d)
{
}

static void g3d_flush_pipeline(struct g3d_drvdata *g3d)
{
}

static void g3d_process_state_buffer(struct g3d_drvdata *g3d,
					const g3d_state_buffer_t *buf)
{
}

static void g3d_process_command_buffer(struct g3d_drvdata *g3d,
					const g3d_command_buffer_t *buf)
{
}

/*
 * Command thread
 */
static int g3d_handle_state_buffer(struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;

	list_add_tail(&req->list, &ctx->state_buf_queue);

	return 0;
}

static int g3d_handle_command_buffer(struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_request *st_req;

	if (g3d->last_ctx != ctx || !list_empty(&ctx->state_buf_queue))
		g3d_flush_pipeline(g3d);

	list_for_each_entry(st_req, &ctx->state_buf_queue, list) {
		g3d_process_state_buffer(g3d,
				(const g3d_state_buffer_t *)st_req->usr.priv);
		kmem_cache_free(g3d->request_cache, st_req);
		kobject_put(&ctx->kobj);
	}
	INIT_LIST_HEAD(&ctx->state_buf_queue);

	g3d_process_command_buffer(g3d,
			(const g3d_command_buffer_t *)st_req->usr.priv);

	kmem_cache_free(g3d->request_cache, req);
	kobject_put(&ctx->kobj);
	g3d->last_ctx = ctx;

	return 0;
}

static int g3d_handle_fence(struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;

	spin_lock(&ctx->fence_lock);
	list_add_tail(&req->list, &ctx->fence_queue);
	spin_unlock(&ctx->fence_lock);

	wake_up(&ctx->fence_wq);

	return 0;
}

static const struct g3d_request_info g3d_requests[] = {
	[OPENFIMG_REQUEST_STATE_BUFFER] = {
		.handler = g3d_handle_state_buffer,
	},
	[OPENFIMG_REQUEST_COMMAND_BUFFER] = {
		.handler = g3d_handle_command_buffer,
	},
	[OPENFIMG_REQUEST_FENCE] = {
		.handler = g3d_handle_fence,
	},
};

static int g3d_command_thread(void *data)
{
	struct g3d_drvdata *g3d = (struct g3d_drvdata *)data;
	const struct g3d_request_info *req_info;
	struct g3d_request *req;
	int ret;

	allow_signal(SIGTERM);

	while (!kthread_should_stop()) {
		spin_lock(&g3d->request_lock);

		while (list_empty(&g3d->request_queue)) {
			spin_unlock(&g3d->request_lock);

			ret = wait_event_interruptible(g3d->request_wq,
					!list_empty(&g3d->request_queue));
			if (ret && kthread_should_stop())
				goto finish;

			spin_lock(&g3d->request_lock);
		}

		req = list_first_entry(&g3d->request_queue,
						struct g3d_request, list);
		list_del(&req->list);

		spin_unlock(&g3d->request_lock);

		if (req->ctx->terminate) {
			kobject_put(&req->ctx->kobj);
			kmem_cache_free(g3d->request_cache, req);
			continue;
		}

		req_info = &g3d_requests[req->usr.type];

		ret = req_info->handler(req);
		if (ret)
			dev_err(g3d->dev, "request %p (type %d) failed\n",
							req, req->usr.type);
	}

finish:
	return 0;
}

/*
 * IRQ handler
 */
static irqreturn_t g3d_handle_irq(int irq, void *dev_id)
{
	struct g3d_drvdata *data = (struct g3d_drvdata *)dev_id;

	g3d_write(data, 0, G3D_FGGB_INTPENDING_REG);

	/* TODO: Implement interrupt handler */

	return IRQ_HANDLED;
}

/*
 * User request handlers
 */
static int g3d_alloc(struct g3d_context *ctx, struct g3d_user_request *urq)
{
	return 0;
}

static int g3d_submit(struct g3d_context *ctx, struct g3d_user_request *urq)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_request *req;

	if (urq->type >= ARRAY_SIZE(g3d_requests)
	    || g3d_requests[urq->type].handler == NULL) {
		dev_err(g3d->dev,
			"invalid request type %d, ignoring\n", urq->type);
		return -EINVAL;
	}

	req = kmem_cache_alloc(g3d->request_cache, GFP_KERNEL | __GFP_ZERO);
	if (!req)
		return -ENOMEM;

	req->usr = *urq;
	req->ctx = ctx;

	kobject_get(&ctx->kobj);

	spin_lock(&g3d->request_lock);
	list_add_tail(&req->list, &g3d->request_queue);
	spin_unlock(&g3d->request_lock);

	wake_up(&g3d->request_wq);

	return 0;
}

static int g3d_fence_wait(struct g3d_context *ctx, struct g3d_user_request *urq)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_request *req;
	int ret;

	spin_lock(&ctx->fence_lock);

	while (list_empty(&ctx->fence_queue)) {
		spin_unlock(&ctx->fence_lock);

		ret = wait_event_interruptible(ctx->fence_wq,
					!list_empty(&ctx->fence_queue));
		if (ret)
			return ret;

		spin_lock(&ctx->fence_lock);
	}

	req = list_first_entry(&ctx->fence_queue, struct g3d_request, list);
	list_del(&req->list);

	spin_unlock(&ctx->fence_lock);

	*urq = req->usr;
	kmem_cache_free(g3d->request_cache, req);
	kobject_put(&ctx->kobject);

	return 0;
}

/*
 * File operations
 */
static long g3d_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	struct g3d_context *ctx = file->private_data;
	struct g3d_user_request req;
	int ret = 0;

	switch(cmd) {
	case OPENFIMG_ALLOC:
		ret = copy_from_user(&req, (void __user *)arg, sizeof(req));
		if (ret)
			return -EFAULT;

		ret = g3d_alloc(ctx, &req);
		if (ret)
			return ret;

		ret = copy_to_user((void __user *)arg, &req, sizeof(req));
		if (ret)
			return -EFAULT;
		break;

	case OPENFIMG_SUBMIT:
		ret = copy_from_user(&req, (void __user *)arg, sizeof(req));
		if (ret)
			return -EFAULT;

		ret = g3d_submit(ctx, &req);
		if (ret)
			return ret;
		break;

	case OPENFIMG_FENCE_WAIT:
		ret = g3d_fence_wait(ctx, &req);
		if (ret)
			return ret;

		ret = copy_to_user((void __user *)arg, &req, sizeof(req));
		if (ret)
			return -EFAULT;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void g3d_ctx_release(struct kobject *kobj)
{
	struct g3d_context *ctx = container_of(kobj, struct g3d_context, kobj);

	kfree(ctx);
}

static struct kobj_type g3d_ctx_kobject = {
	.release	= g3d_ctx_release,
};

static int g3d_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct g3d_drvdata *g3d = container_of(mdev, struct g3d_drvdata, mdev);
	struct g3d_context *ctx;
	int ret;

	ctx = kmalloc(sizeof(struct g3d_context), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->g3d = g3d;

	spin_lock_init(&ctx->fence_lock);
	INIT_LIST_HEAD(&ctx->fence_queue);
	init_waitqueue_head(&ctx->fence_wq);
	INIT_LIST_HEAD(&ctx->state_buf_queue);

	kobject_init(&ctx->kobj, &g3d_ctx_kobject);
	ret = kobject_add(&ctx->kobj, &g3d->dev->kobj, "g3d_context.%p", ctx);
	if (ret) {
		kfree(ctx);
		return ret;
	}

	file->private_data = ctx;

	dev_dbg(g3d->dev, "device opened\n");

	return 0;
}

static int g3d_release(struct inode *inode, struct file *file)
{
	struct g3d_context *ctx = file->private_data;
	struct g3d_drvdata *g3d = ctx->g3d;

	ctx->terminate = true;
	kobject_put(&ctx->kobj);

	dev_dbg(g3d->dev, "device released\n");

	return 0;
}

static struct file_operations g3d_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= g3d_ioctl,
	.open		= g3d_open,
	.release	= g3d_release,
};

/*
 * Platform device operations
 */
static int __init g3d_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct g3d_drvdata *g3d;
	uint32_t version;
	int ret;

	if (pdev->id != -1) {
		dev_err(&pdev->dev, "only single instance is allowed.\n");
		return -EINVAL;
	}

	g3d = kzalloc(sizeof(*g3d), GFP_KERNEL);
	if(!g3d) {
		dev_err(&pdev->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

	/* initialize the miscdevice struct */
	g3d->mdev.minor	= MISC_DYNAMIC_MINOR;
	g3d->mdev.name		= "s3c-g3d";
	g3d->mdev.fops		= &g3d_fops;

	/* get device clock */
	g3d->clock = clk_get(&pdev->dev, "3dse");
	if (g3d->clock == NULL) {
		dev_err(&pdev->dev, "failed to find g3d clock source\n");
		ret = -ENOENT;
		goto err_clock;
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev, G3D_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	/* get the memory region for the post processor driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource.\n");
		ret = -ENOENT;
		goto err_mem;
	}

	/* reserve the memory */
	g3d->mem = request_mem_region(res->start, resource_size(res),
								pdev->name);
	if (g3d->mem == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_mem;
	}

	/* map the memory */
	g3d->base = ioremap(g3d->mem->start, resource_size(g3d->mem));
	if (g3d->base == NULL) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOENT;
		goto err_ioremap;
	}

	/* get the IRQ */
	g3d->irq = platform_get_irq(pdev, 0);
	if (g3d->irq <= 0) {
		dev_err(&pdev->dev,
			"failed to get irq resource (%d).\n", g3d->irq);
		ret = g3d->irq;
		goto err_irq;
	}

	/* request the IRQ */
	ret = request_irq(g3d->irq, g3d_handle_irq, 0, pdev->name, g3d);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed (%d).\n", ret);
		goto err_irq;
	}

	g3d->dev = &pdev->dev;
	spin_lock_init(&g3d->request_lock);
	INIT_LIST_HEAD(&g3d->request_queue);
	init_waitqueue_head(&g3d->request_wq);

	g3d->request_cache = KMEM_CACHE(g3d_request, 0);
	if (!g3d->request_cache) {
		dev_err(&pdev->dev, "failed to create request slab cache\n");
		ret = -ENOMEM;
		goto err_request_cache;
	}

	platform_set_drvdata(pdev, g3d);

	pm_runtime_get_sync(&pdev->dev);

	version = g3d_read(g3d, G3D_FGGB_VERSION);
	dev_info(&pdev->dev, "detected FIMG-3DSE version %d.%d.%d\n",
		version >> 24, (version >> 16) & 0xff, (version >> 8) & 0xff);

	pm_runtime_put_sync(&pdev->dev);

	g3d->thread = kthread_run(g3d_command_thread, g3d, "kfimgd");
	if (IS_ERR(g3d->thread)) {
		ret = PTR_ERR(g3d->thread);
		dev_err(&pdev->dev,
				"failed to create command thread (%d)\n", ret);
		goto err_thread;
	}

	ret = misc_register(&g3d->mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register miscdev (%d)\n", ret);
		goto err_misc_register;
	}

	return 0;

err_misc_register:
	kthread_stop(g3d->thread);
err_thread:
	kmem_cache_destroy(g3d->request_cache);
err_request_cache:
	free_irq(g3d->irq, pdev);
err_irq:
	iounmap(g3d->base);
err_ioremap:
	release_resource(g3d->mem);
err_mem:
	pm_runtime_disable(&pdev->dev);
	clk_put(g3d->clock);
err_clock:
	kfree(g3d);

	return ret;
}

static int __devexit g3d_remove(struct platform_device *pdev)
{
	struct g3d_drvdata *g3d = platform_get_drvdata(pdev);

	misc_deregister(&g3d->mdev);

	send_sig(SIGTERM, g3d->thread, 1);
	kthread_stop(g3d->thread);
	kmem_cache_destroy(g3d->request_cache);

	pm_runtime_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	free_irq(g3d->irq, g3d);
	iounmap(g3d->base);
	release_resource(g3d->mem);
	clk_put(g3d->clock);
	kfree(g3d);

	return 0;
}

static int g3d_runtime_suspend(struct device *dev)
{
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);

	clk_enable(g3d->clock);
	g3d_flush_caches(g3d);
	clk_disable(g3d->clock);

	return 0;
}

static int g3d_runtime_resume(struct device *dev)
{
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);

	clk_enable(g3d->clock);
	g3d_soft_reset(g3d);
	clk_disable(g3d->clock);

	return 0;
}

static int g3d_suspend(struct device *dev)
{
	if (pm_runtime_suspended(dev))
		return 0;

	return g3d_runtime_suspend(dev);
}

static int g3d_resume(struct device *dev)
{
	if (pm_runtime_suspended(dev))
		return 0;

	return g3d_runtime_resume(dev);
}

static struct dev_pm_ops g3d_pm_ops = {
	.suspend		= g3d_suspend,
	.resume			= g3d_resume,
	.runtime_suspend	= g3d_runtime_suspend,
	.runtime_resume		= g3d_runtime_resume,
};

static struct platform_driver g3d_driver = {
	.remove	= __devexit_p(g3d_remove),
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "s3c-g3d",
		.pm	= &g3d_pm_ops,
	},
};

/*
 * Module operations
 */
int __init  g3d_init(void)
{
	return platform_driver_probe(&g3d_driver, g3d_probe);
}

void __exit g3d_exit(void)
{
	platform_driver_unregister(&g3d_driver);
}

module_init(g3d_init);
module_exit(g3d_exit);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("OpenFIMG Kernel Interface");
MODULE_LICENSE("GPL");
