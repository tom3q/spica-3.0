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
 *
 * TODO: (random order)
 * - shaders (load, switch, storage)
 * - textures (objects, backing storage, switch, configuration)
 * - framebuffers (objects, backing storage, switch, configuration)
 * - advanced scheduling (instead of simple list)
 * - access to protected registers (some registers of per-fragment unit)
 * - better request buffer allocation (limits, pools)
 * - more intelligent handling of fences (flushing only when necessary)
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <video/openfimg.h>

/*
 * Various definitions
 */
#define G3D_AUTOSUSPEND_DELAY		100
#define G3D_FLUSH_TIMEOUT		1000

/*
 * Registers
 */
#define G3D_FGGB_PIPESTAT_REG		0x00
#define G3D_FGGB_PIPESTAT_MSK		0x0005171f

#define G3D_FGGB_CACHECTL_REG		0x04
#define G3D_FGGB_FLUSH_MSK		0x00000033
#define G3D_FGGB_INVAL_MSK		0x00001300

#define G3D_FGGB_RESET_REG		0x08
#define G3D_FGGB_VERSION		0x10
#define G3D_FGGB_INTPENDING_REG		0x40
#define G3D_FGGB_INTMASK_REG		0x44
#define G3D_FGGB_PIPEMASK_REG		0x48
#define G3D_FGGB_PIPETGTSTATE_REG	0x4c

#define G3D_FGHI_VBADDR_REG		0x8010
#define G3D_FGHI_FIFO_ENTRY_REG		0xc000
#define G3D_FGHI_VB_ENTRY_REG		0xe000

/*
 * Private data types
 */
struct g3d_context;

enum g3d_state_bits {
	G3D_STATE_BUSY,
	G3D_STATE_DISABLED
};

struct g3d_drvdata {
	struct miscdevice	mdev;
	void __iomem		*base;
	int			irq;
	struct resource 	*mem;
	struct clk		*clock;
	struct device		*dev;
	struct task_struct	*thread;

	struct mutex		stall_mutex;

	spinlock_t		request_lock;
	struct list_head	request_queue;
	struct kmem_cache	*request_cache;
	wait_queue_head_t	request_wq;

	struct g3d_context	*last_ctx;

	unsigned long		state;
};

struct g3d_context {
	struct g3d_drvdata *g3d;
	struct file *file;

	spinlock_t fence_lock;
	struct list_head fence_queue;
	wait_queue_head_t fence_wq;

	struct list_head state_buf_queue;

	struct list_head request_list;

	u32 registers[G3D_NUM_REGISTERS];
};

struct g3d_request {
	struct g3d_user_request usr;
	struct g3d_context	*ctx;
	struct list_head	list;
	struct list_head	ctx_list;
};

typedef void *g3d_command_buffer_t;

struct g3d_request_info {
	int (*allocate)(struct g3d_request *);
	void (*free)(struct g3d_request *);
	int (*handle)(struct g3d_request *);
};

static const u32 g3d_registers[G3D_NUM_REGISTERS] = {
	/* Host interface */
	[FGHI_ATTRIB0] = 0x8040,
	[FGHI_ATTRIB1] = 0x8044,
	[FGHI_ATTRIB2] = 0x8048,
	[FGHI_ATTRIB3] = 0x804c,
	[FGHI_ATTRIB4] = 0x8050,
	[FGHI_ATTRIB5] = 0x8054,
	[FGHI_ATTRIB6] = 0x8058,
	[FGHI_ATTRIB7] = 0x805c,
	[FGHI_ATTRIB8] = 0x8060,
	[FGHI_ATTRIB9] = 0x8064,
	[FGHI_ATTRIB_VBCTRL0] = 0x8080,
	[FGHI_ATTRIB_VBCTRL1] = 0x8084,
	[FGHI_ATTRIB_VBCTRL2] = 0x8088,
	[FGHI_ATTRIB_VBCTRL3] = 0x808c,
	[FGHI_ATTRIB_VBCTRL4] = 0x8090,
	[FGHI_ATTRIB_VBCTRL5] = 0x8094,
	[FGHI_ATTRIB_VBCTRL6] = 0x8098,
	[FGHI_ATTRIB_VBCTRL7] = 0x809c,
	[FGHI_ATTRIB_VBCTRL8] = 0x80a0,
	[FGHI_ATTRIB_VBCTRL9] = 0x80a4,
	[FGHI_ATTRIB_VBBASE0] = 0x80c0,
	[FGHI_ATTRIB_VBBASE1] = 0x80c4,
	[FGHI_ATTRIB_VBBASE2] = 0x80c8,
	[FGHI_ATTRIB_VBBASE3] = 0x80cc,
	[FGHI_ATTRIB_VBBASE4] = 0x80d0,
	[FGHI_ATTRIB_VBBASE5] = 0x80d4,
	[FGHI_ATTRIB_VBBASE6] = 0x80d8,
	[FGHI_ATTRIB_VBBASE7] = 0x80dc,
	[FGHI_ATTRIB_VBBASE8] = 0x80e0,
	[FGHI_ATTRIB_VBBASE9] = 0x80e4,

	/* Primitive engine */
	[FGPE_VERTEX_CONTEXT] = 0x30000,
	[FGPE_VIEWPORT_OX] = 0x30004,
	[FGPE_VIEWPORT_OY] = 0x30008,
	[FGPE_VIEWPORT_HALF_PX] = 0x3000c,
	[FGPE_VIEWPORT_HALF_PY] = 0x30010,
	[FGPE_DEPTHRANGE_HALF_F_SUB_N] = 0x30014,
	[FGPE_DEPTHRANGE_HALF_F_ADD_N] = 0x30018,

	/* Raster engine */
	[FGRA_PIX_SAMP] = 0x38000,
	[FGRA_D_OFF_EN] = 0x38004,
	[FGRA_D_OFF_FACTOR] = 0x38008,
	[FGRA_D_OFF_UNITS] = 0x3800c,
	[FGRA_D_OFF_R_IN] = 0x38010,
	[FGRA_BFCULL] = 0x38014,
	[FGRA_YCLIP] = 0x38018,
	[FGRA_LODCTL] = 0x3c000,
	[FGRA_XCLIP] = 0x3c004,
	[FGRA_PWIDTH] = 0x3801c,
	[FGRA_PSIZE_MIN] = 0x38020,
	[FGRA_PSIZE_MAX] = 0x38024,
	[FGRA_COORDREPLACE] = 0x38028,
	[FGRA_LWIDTH] = 0x3802c,

	/* Per-fragment unit */
	[FGPF_ALPHAT] = 0x70008,
	[FGPF_CCLR] = 0x70018,
	[FGPF_BLEND] = 0x7001c,
	[FGPF_LOGOP] = 0x70020,
};

static const struct g3d_request_info g3d_requests[];

/*
 * Register accessors
 */
static inline void g3d_write(struct g3d_drvdata *g3d,
						uint32_t val, uint32_t reg)
{
	writel(val, g3d->base + reg);
}

static inline void g3d_write_burst(struct g3d_drvdata *g3d, uint32_t reg,
					const uint32_t *buf, uint32_t len)
{
	writesl(g3d->base + reg, buf, len);
}

static inline uint32_t g3d_read(struct g3d_drvdata *g3d, uint32_t reg)
{
	return readl(g3d->base + reg);
}

/*
 * Hardware operations
 */
static inline void g3d_soft_reset(struct g3d_drvdata *g3d)
{
	g3d_write(g3d, 0, G3D_FGGB_INTMASK_REG);
	g3d_write(g3d, 0, G3D_FGGB_INTPENDING_REG);

	g3d_write(g3d, 1, G3D_FGGB_RESET_REG);
	udelay(1);
	g3d_write(g3d, 0, G3D_FGGB_RESET_REG);
	udelay(1);

	clear_bit(G3D_STATE_BUSY, &g3d->state);
	wake_up(&g3d->request_wq);
}

static inline void g3d_idle_irq_enable(struct g3d_drvdata *g3d)
{
	g3d_write(g3d, 1, G3D_FGGB_INTMASK_REG);
}

static inline void g3d_idle_irq_ack_and_disable(struct g3d_drvdata *g3d)
{
	g3d_write(g3d, 0, G3D_FGGB_INTMASK_REG);
	g3d_write(g3d, 0, G3D_FGGB_INTPENDING_REG);
}

static inline int g3d_wait_for_flush(struct g3d_drvdata *g3d)
{
	int ret;

	while (test_bit(G3D_STATE_BUSY, &g3d->state)) {
		ret = wait_event_interruptible_timeout(g3d->request_wq,
					!test_bit(G3D_STATE_BUSY, &g3d->state),
					msecs_to_jiffies(G3D_FLUSH_TIMEOUT));
		if (ret) {
			dev_err(g3d->dev, "pipeline flush timed out\n");
			g3d_soft_reset(g3d);
			return ret;
		}
	}

	return 0;
}

static int g3d_flush_pipeline(struct g3d_drvdata *g3d, bool flush_caches)
{
	int ret;

	ret = g3d_wait_for_flush(g3d);
	if (ret)
		return ret;

	if (!flush_caches)
		return 0;

	g3d_write(g3d, G3D_FGGB_FLUSH_MSK, G3D_FGGB_CACHECTL_REG);

	set_bit(G3D_STATE_BUSY, &g3d->state);
	g3d_idle_irq_enable(g3d);

	return g3d_wait_for_flush(g3d);
}

static bool g3d_process_state_buffer(struct g3d_context *ctx,
				struct g3d_request *req, bool partial_update)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	const struct g3d_state_buffer *buf = req->usr.state.buf;
	unsigned int i;

	memcpy(ctx->registers, buf->registers, G3D_NUM_REGISTERS * sizeof(u32));

	if (!partial_update)
		return false;

	if (!req->usr.state.flags & G3D_STATE_NO_PARTIAL)
		return true;

	for_each_set_bit(i, buf->dirty, G3D_NUM_REGISTERS)
		g3d_write(g3d, ctx->registers[i], g3d_registers[i]);

	return false;
}

static void g3d_process_command_buffer(struct g3d_context *ctx,
						struct g3d_request *req)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	const void *vertex_buf = req->usr.command.buf;
	unsigned int vertex_cnt = req->usr.command.count;
	size_t vertex_len = req->usr.command.len;

	g3d_write(g3d, 0, G3D_FGHI_VBADDR_REG);
	g3d_write_burst(g3d, G3D_FGHI_VB_ENTRY_REG, vertex_buf, vertex_len);

	g3d_write(g3d, vertex_cnt, G3D_FGHI_FIFO_ENTRY_REG);
	g3d_write(g3d, 0, G3D_FGHI_FIFO_ENTRY_REG);
}

/*
 * Command thread
 */
static int g3d_allocate_state_buffer(struct g3d_request *req)
{
	struct g3d_state_buffer __user *src = req->usr.state.buf;
	int ret;

	/* TODO: Use better allocation method */
	req->usr.state.buf = kmalloc(sizeof(*src), GFP_KERNEL);
	if (!req->usr.state.buf)
		return -ENOMEM;

	ret = copy_from_user(req->usr.state.buf, src, sizeof(*src));
	if (ret)
		return -EFAULT;

	return 0;
}

static void g3d_free_state_buffer(struct g3d_request *req)
{
	kfree(req->usr.state.buf);
}

static int g3d_handle_state_buffer(struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;

	list_add_tail(&req->list, &ctx->state_buf_queue);

	return 0;
}

static int g3d_allocate_command_buffer(struct g3d_request *req)
{
	void __user *src = req->usr.command.buf;
	int ret;

	/* TODO: Use better allocation method */
	req->usr.command.buf = kmalloc(req->usr.command.len, GFP_KERNEL);
	if (!req->usr.command.buf)
		return -ENOMEM;

	ret = copy_from_user(req->usr.command.buf, src, req->usr.command.len);
	if (ret)
		return -EFAULT;

	return 0;
}

static void g3d_free_command_buffer(struct g3d_request *req)
{
	kfree(req->usr.command.buf);
}

static void g3d_restore_context(struct g3d_drvdata *g3d,
						struct g3d_context *ctx)
{
	unsigned int i;

	for (i = 0; i < G3D_NUM_REGISTERS; ++i)
		g3d_write(g3d, ctx->registers[i], g3d_registers[i]);
}

static void g3d_free_request(struct g3d_drvdata *g3d, struct g3d_request *req)
{
	if (g3d_requests[req->usr.type].free)
		g3d_requests[req->usr.type].free(req);
	kmem_cache_free(g3d->request_cache, req);
}

static int g3d_handle_command_buffer(struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_request *st_req;
	bool partial_update = true;

	if (test_and_clear_bit(G3D_STATE_DISABLED, &g3d->state)) {
		pm_runtime_get_sync(g3d->dev);
		clk_enable(g3d->clock);
		partial_update = false;
	} else if (g3d->last_ctx != ctx || !list_empty(&ctx->state_buf_queue)) {
		g3d_flush_pipeline(g3d, false);
		if (g3d->last_ctx != ctx)
			partial_update = false;
	}

	list_for_each_entry(st_req, &ctx->state_buf_queue, list) {
		partial_update &= g3d_process_state_buffer(ctx,
							st_req, partial_update);
		g3d_free_request(g3d, st_req);
	}
	INIT_LIST_HEAD(&ctx->state_buf_queue);

	if (!partial_update)
		g3d_restore_context(g3d, ctx);

	g3d_process_command_buffer(ctx, req);

	g3d_free_request(g3d, req);
	g3d->last_ctx = ctx;

	set_bit(G3D_STATE_BUSY, &g3d->state);
	g3d_idle_irq_enable(g3d);

	return 0;
}

static int g3d_handle_fence(struct g3d_request *req)
{
	struct g3d_context *ctx = req->ctx;

	if (req->usr.fence.flags & G3D_FENCE_FLUSH) {
		int ret = g3d_flush_pipeline(ctx->g3d, true);
		if (ret) {
			req->usr.fence.flags |= G3D_FENCE_TIMED_OUT;
			dev_err(ctx->g3d->dev, "fence wait timed out\n");
		}
	}

	spin_lock(&ctx->fence_lock);
	list_add_tail(&req->list, &ctx->fence_queue);
	spin_unlock(&ctx->fence_lock);

	wake_up(&ctx->fence_wq);

	return 0;
}

static int g3d_handle_shader_switch(struct g3d_request *req)
{
	/* TODO: Implement shader switch */

	return 0;
}

static const struct g3d_request_info g3d_requests[] = {
	[G3D_REQUEST_STATE_BUFFER] = {
		.allocate = g3d_allocate_state_buffer,
		.free = g3d_free_state_buffer,
		.handle = g3d_handle_state_buffer,
	},
	[G3D_REQUEST_COMMAND_BUFFER] = {
		.allocate = g3d_allocate_command_buffer,
		.free = g3d_free_command_buffer,
		.handle = g3d_handle_command_buffer,
	},
	[G3D_REQUEST_FENCE] = {
		.handle = g3d_handle_fence,
	},
	[G3D_REQUEST_SHADER_SWITCH] = {
		.handle = g3d_handle_shader_switch,
	},
};

static inline bool g3d_request_ready(struct g3d_drvdata *g3d)
{
	if (test_bit(G3D_STATE_BUSY, &g3d->state))
		goto check_queue;

	if (WARN_ON(test_bit(G3D_STATE_DISABLED, &g3d->state)))
		goto check_queue;

	clk_disable(g3d->clock);
	pm_runtime_mark_last_busy(g3d->dev);
	pm_runtime_put_autosuspend(g3d->dev);
	set_bit(G3D_STATE_DISABLED, &g3d->state);

check_queue:
	return !list_empty(&g3d->request_queue);
}

static int g3d_command_thread(void *data)
{
	struct g3d_drvdata *g3d = (struct g3d_drvdata *)data;
	const struct g3d_request_info *req_info;
	struct g3d_request *req;
	int ret;

	allow_signal(SIGTERM);
	set_freezable();

	while (!kthread_should_stop()) {
		mutex_lock(&g3d->stall_mutex);

		spin_lock(&g3d->request_lock);

		while (list_empty(&g3d->request_queue)) {
			spin_unlock(&g3d->request_lock);

			mutex_unlock(&g3d->stall_mutex);

			ret = wait_event_freezable(g3d->request_wq,
							g3d_request_ready(g3d));
			if (ret && kthread_should_stop())
				goto finish;

			mutex_lock(&g3d->stall_mutex);

			spin_lock(&g3d->request_lock);
		}

		req = list_first_entry(&g3d->request_queue,
						struct g3d_request, list);
		list_del(&req->list);
		list_del(&req->ctx_list);

		spin_unlock(&g3d->request_lock);

		req_info = &g3d_requests[req->usr.type];

		ret = req_info->handle(req);
		if (ret)
			dev_err(g3d->dev, "request %p (type %d) failed\n",
							req, req->usr.type);

		mutex_unlock(&g3d->stall_mutex);
	}

finish:
	return 0;
}

/*
 * IRQ handler
 */
static irqreturn_t g3d_handle_irq(int irq, void *dev_id)
{
	struct g3d_drvdata *g3d = (struct g3d_drvdata *)dev_id;

	g3d_idle_irq_ack_and_disable(g3d);

	clear_bit(G3D_STATE_BUSY, &g3d->state);
	wake_up(&g3d->request_wq);

	return IRQ_HANDLED;
}

/*
 * User request handlers
 */
static struct g3d_request *g3d_allocate_request(struct g3d_drvdata *g3d,
						struct g3d_user_request *urq)
{
	struct g3d_request *req;
	int ret;

	req = kmem_cache_alloc(g3d->request_cache, GFP_KERNEL | __GFP_ZERO);
	if (!req)
		return NULL;

	req->usr = *urq;

	if (g3d_requests[urq->type].allocate) {
		ret = g3d_requests[urq->type].allocate(req);
		if (ret) {
			dev_err(g3d->dev, "failed to allocate request data\n");
			kmem_cache_free(g3d->request_cache, req);
			return NULL;
		}
	}

	return req;
}

static int g3d_submit(struct g3d_context *ctx, struct g3d_user_request *urq)
{
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_request *req;

	if (urq->type >= ARRAY_SIZE(g3d_requests)
	    || g3d_requests[urq->type].handle == NULL) {
		dev_err(g3d->dev,
			"invalid request type %d, ignoring\n", urq->type);
		return -EINVAL;
	}

	req = g3d_allocate_request(g3d, urq);
	if (!req)
		return -ENOMEM;

	req->ctx = ctx;

	spin_lock(&g3d->request_lock);
	list_add_tail(&req->list, &g3d->request_queue);
	list_add_tail(&req->ctx_list, &ctx->request_list);
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

		if (ctx->file->f_flags & O_NONBLOCK)
			return -EAGAIN;

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
	g3d_free_request(g3d, req);

	return 0;
}

static int g3d_shader_load(struct g3d_context *ctx,
						struct g3d_user_request *urq)
{
	/* TODO: Implement shader load */

	return 0;
}

/*
 * File operations
 */
static long g3d_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct g3d_context *ctx = file->private_data;
	struct g3d_user_request req;
	int ret = 0;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = copy_from_user(&req, (void __user *)arg, sizeof(req));
		if (ret)
			return -EFAULT;
	}

	switch(cmd) {
	case _REQUEST_SUBMIT_NR:
		ret = g3d_submit(ctx, &req);
		if (ret)
			return ret;
		break;

	case G3D_FENCE_WAIT:
		ret = g3d_fence_wait(ctx, &req);
		if (ret)
			return ret;
		break;

	case G3D_SHADER_LOAD:
		ret = g3d_shader_load(ctx, &req);
		if (ret)
			return ret;

	default:
		dev_err(ctx->g3d->dev, "invalid IOCTL %x\n", cmd);
		return -EINVAL;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		ret = copy_to_user((void __user *)arg, &req, sizeof(req));
		if (ret)
			return -EFAULT;
	}

	return 0;
}

static unsigned int g3d_poll(struct file *file, poll_table *wait)
{
	struct g3d_context *ctx = file->private_data;
	bool avail;

	poll_wait(file, &ctx->fence_wq, wait);

	spin_lock(&ctx->fence_lock);
	avail = !list_empty(&ctx->fence_queue);
	spin_unlock(&ctx->fence_lock);

	return avail ? POLLIN | POLLRDNORM : 0;
}

static int g3d_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct g3d_drvdata *g3d = container_of(mdev, struct g3d_drvdata, mdev);
	struct g3d_context *ctx;

	ctx = kmalloc(sizeof(struct g3d_context), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->g3d = g3d;
	ctx->file = file;

	spin_lock_init(&ctx->fence_lock);
	INIT_LIST_HEAD(&ctx->fence_queue);
	init_waitqueue_head(&ctx->fence_wq);
	INIT_LIST_HEAD(&ctx->state_buf_queue);
	INIT_LIST_HEAD(&ctx->request_list);

	file->private_data = ctx;

	dev_dbg(g3d->dev, "device opened\n");

	return 0;
}

static int g3d_release(struct inode *inode, struct file *file)
{
	struct g3d_context *ctx = file->private_data;
	struct g3d_drvdata *g3d = ctx->g3d;
	struct g3d_request *req, *n;

	mutex_lock(&g3d->stall_mutex);

	list_for_each_entry(req, &ctx->request_list, ctx_list)
		list_del(&req->list);

	mutex_unlock(&g3d->stall_mutex);

	list_for_each_entry_safe(req, n, &ctx->request_list, ctx_list) {
		list_del(&req->ctx_list);
		g3d_free_request(g3d, req);
	}

	list_for_each_entry_safe(req, n, &ctx->state_buf_queue, list) {
		list_del(&req->list);
		g3d_free_request(g3d, req);
	}

	list_for_each_entry_safe(req, n, &ctx->fence_queue, list) {
		list_del(&req->list);
		g3d_free_request(g3d, req);
	}

	kfree(ctx);

	dev_dbg(g3d->dev, "device released\n");

	return 0;
}

static struct file_operations g3d_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= g3d_ioctl,
	.poll		= g3d_poll,
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
	mutex_init(&g3d->stall_mutex);
	set_bit(G3D_STATE_DISABLED, &g3d->state);

	g3d->request_cache = KMEM_CACHE(g3d_request, 0);
	if (!g3d->request_cache) {
		dev_err(&pdev->dev, "failed to create request slab cache\n");
		ret = -ENOMEM;
		goto err_request_cache;
	}

	platform_set_drvdata(pdev, g3d);

	pm_runtime_get_sync(&pdev->dev);

	clk_enable(g3d->clock);
	version = g3d_read(g3d, G3D_FGGB_VERSION);
	dev_info(&pdev->dev, "detected FIMG-3DSE version %d.%d.%d\n",
		version >> 24, (version >> 16) & 0xff, (version >> 8) & 0xff);
	clk_disable(g3d->clock);

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
	int ret;

	clk_enable(g3d->clock);
	ret = g3d_flush_pipeline(g3d, true);
	clk_disable(g3d->clock);

	return ret;
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
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);
	int ret;

	if (pm_runtime_suspended(dev))
		return 0;

	if (!test_bit(G3D_STATE_DISABLED, &g3d->state)) {
		ret = g3d_flush_pipeline(g3d, true);
		if (ret)
			return ret;
		clk_disable(g3d->clock);
	}

	return g3d_runtime_suspend(dev);
}

static int g3d_resume(struct device *dev)
{
	struct g3d_drvdata *g3d = dev_get_drvdata(dev);

	if (pm_runtime_suspended(dev))
		return 0;

	if (!test_bit(G3D_STATE_DISABLED, &g3d->state))
		clk_enable(g3d->clock);

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
