/*
 * OneDRAM Phone Interface Device Driver
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *	Complete rewrite including built-in multipdp function.
 *
 * Copyright (C) 2006-2010 Samsung Electronics Co.Ltd, all rights reserved
 *	Original driver.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/mtd/mtd.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/circ_buf.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/platform_data/spica_dpram.h>
#include <asm/ioctls.h>

extern struct class *sec_class;

#define DRIVER_PROC_ENTRY	"driver/dpram"

enum dpram_status {
	DPRAM_PHONE_OFF = 0,
	DPRAM_PHONE_CRASHED,
	DPRAM_PHONE_RAMDUMP,
	DPRAM_PHONE_POWER_ON,
	DPRAM_PHONE_BOOTLOADER,
	DPRAM_PHONE_IMAGE_LOADED,
	DPRAM_PHONE_BOOTING_NORMAL,
	DPRAM_PHONE_BOOTING_RAMDUMP,
	DPRAM_PHONE_DUMPING,
	DPRAM_PHONE_INITIALIZING,
	DPRAM_PHONE_RUNNING
};

#define dpram_phone_offline(dpr) ((dpr)->status < DPRAM_PHONE_POWER_ON)
#define dpram_phone_bootloader(dpr) ((dpr)->status == DPRAM_PHONE_BOOTLOADER)
#define dpram_phone_running(dpr) ((dpr)->status == DPRAM_PHONE_RUNNING)

/* onedram shared memory map */
#define OFF_IMAGE		0x00000000
#define OFF_FLASH_IMAGE		0x00005000
#define SIZ_IMAGE		0x00ffb000

#define OFF_MAGIC		0x00000000
#define OFF_ACCESS		0x00000002

#define OFF_TX_FMT_START	(OFF_MAGIC + 0x0004)
#define OFF_TX_FMT_HEAD		(OFF_TX_FMT_START)
#define OFF_TX_FMT_TAIL		(OFF_TX_FMT_HEAD + 2)
#define OFF_TX_FMT_DATA		(OFF_TX_FMT_TAIL + 2)
#define SIZ_TX_FMT_DATA		1020

#define OFF_TX_RAW_START	(OFF_TX_FMT_DATA + SIZ_TX_FMT_DATA)
#define OFF_TX_RAW_HEAD		(OFF_TX_RAW_START)
#define OFF_TX_RAW_TAIL		(OFF_TX_RAW_HEAD + 2)
#define OFF_TX_RAW_DATA		(OFF_TX_RAW_TAIL + 2)
#define SIZ_TX_RAW_DATA		15344

#define OFF_RX_FMT_START	(OFF_TX_RAW_DATA + SIZ_TX_RAW_DATA)
#define OFF_RX_FMT_HEAD		(OFF_RX_FMT_START)
#define OFF_RX_FMT_TAIL		(OFF_RX_FMT_HEAD + 2)
#define OFF_RX_FMT_DATA		(OFF_RX_FMT_TAIL + 2)
#define SIZ_RX_FMT_DATA		1020

#define OFF_RX_RAW_START	(OFF_RX_FMT_DATA + SIZ_RX_FMT_DATA)
#define OFF_RX_RAW_HEAD		(OFF_RX_RAW_START)
#define OFF_RX_RAW_TAIL		(OFF_RX_RAW_HEAD + 2)
#define OFF_RX_RAW_DATA		(OFF_RX_RAW_TAIL + 2)
#define SIZ_RX_RAW_DATA		15344

#define SIZ_DPRAM		0x00007ffc

#define OFF_NVDATA		0x00f7b000
#define OFF_DGS_INFO		0x00fff000
#define SIZ_DGS_INFO		0x00000100

#define OFF_ERROR_MSG		0x00000408
#define SIZ_ERROR_MSG		62
#define SIZ_ERROR_HDR		2

#define BIT_TX_FMT		INT_MASK_SEND_F
#define BIT_TX_RAW		INT_MASK_SEND_R
#define BIT_RX_FMT		INT_MASK_RES_ACK_F
#define BIT_RX_RAW		INT_MASK_RES_ACK_R

#define DPRAM_SFR		(0xFFF800)
#define	DPRAM_SMP		(DPRAM_SFR)
#define DPRAM_MBX_AB		(DPRAM_SFR + 0x20)
#define DPRAM_MBX_BA		(DPRAM_SFR + 0x40)
#define DPRAM_CHECK_AB		(DPRAM_SFR + 0xA0)
#define DPRAM_CHECK_BA		(DPRAM_SFR + 0xC0)

#define DPRAM_MSG_SBL_DONE	0x12341234
#define DPRAM_CMD_BINARY_LOAD	0x45674567
#define DPRAM_MSG_BINARY_DONE	0xabcdabcd

#define DPRAM_CMD_RAMDUMP_START	0xDEADDEAD
#define DPRAM_MSG_RAMDUMP_LARGE	0x0ADD0ADD // 16MB - 2KB
#define DPRAM_CMD_RAMDUMP_MORE	0xEDEDEDED
#define DPRAM_MSG_RAMDUMP_SMALL	0xFADEFADE // 5MB + 4KB

#define RAMDUMP_LARGE_SIZE	(16*1024*1024 - 2*1024)
#define RAMDUMP_SMALL_SIZE	(5*1024*1024 + 4*1024)

/*
 * interrupt masks.
 */
#define INT_MASK_VALID		0x0080
#define INT_MASK_COMMAND	0x0040
#define INT_MASK_REQ_ACK_F	0x0020
#define INT_MASK_REQ_ACK_R	0x0010
#define INT_MASK_RES_ACK_F	0x0008
#define INT_MASK_RES_ACK_R	0x0004
#define INT_MASK_SEND_F		0x0002
#define INT_MASK_SEND_R		0x0001

#define CMD_INIT_START		0x0001
#define CMD_INIT_END		0x0002
#define CMD_REQ_ACTIVE		0x0003
#define CMD_RES_ACTIVE		0x0004
#define CMD_REQ_TIME_SYNC	0x0005
#define CMD_POWER_OFF		0x0006
#define CMD_RESET		0x0007
#define CMD_PHONE_START 	0x0008
#define CMD_ERR_DISPLAY		0x0009
#define CMD_SUSPEND		0x000A
#define CMD_NV_REBUILDING	0x000B
#define CMD_EMER_DOWN		0x000C
#define CMD_SMP_REQ		0x000D
#define CMD_SMP_REP		0x000E

#define INT_COMMAND(x)		(INT_MASK_VALID | INT_MASK_COMMAND | x)
#define INT_NON_COMMAND(x)	(INT_MASK_VALID | x)

/* ioctl command definitions. */
#define IOC_MZ_MAGIC			(0x6f)
#define IOCTL_DPRAM_PHONE_POWON		_IO(IOC_MZ_MAGIC, 0xd0)
#define IOCTL_DPRAM_PHONEIMG_LOAD	_IO(IOC_MZ_MAGIC, 0xd1)
#define IOCTL_DPRAM_NVDATA_LOAD		_IO(IOC_MZ_MAGIC, 0xd2)
#define IOCTL_DPRAM_PHONE_BOOTSTART	_IO(IOC_MZ_MAGIC, 0xd3)

#define IOC_SEC_MAGIC			(0xf0)
#define IOCTL_DPRAM_PHONE_ON		_IO(IOC_SEC_MAGIC, 0xc0)
#define IOCTL_DPRAM_PHONE_OFF		_IO(IOC_SEC_MAGIC, 0xc1)
#define IOCTL_DPRAM_PHONE_GETSTATUS	_IOR(IOC_SEC_MAGIC, 0xc2, unsigned int)
#define IOCTL_DPRAM_PHONE_MDUMP		_IO(IOC_SEC_MAGIC, 0xc3)
#define IOCTL_DPRAM_PHONE_BATTERY	_IO(IOC_SEC_MAGIC, 0xc4)
#define IOCTL_DPRAM_PHONE_RESET		_IO(IOC_SEC_MAGIC, 0xc5)
#define IOCTL_DPRAM_PHONE_RAMDUMP_ON	_IO(IOC_SEC_MAGIC, 0xc6)
#define IOCTL_DPRAM_PHONE_RAMDUMP_OFF	_IO(IOC_SEC_MAGIC, 0xc7)
#define IOCTL_DPRAM_GET_DGS_INFO	_IOR(IOC_SEC_MAGIC, 0xc8, unsigned char [SIZ_DGS_INFO])

#define IOC_MZ2_MAGIC			(0xC1)
#define IOCTL_PDP_ACTIVATE		_IOWR(IOC_MZ2_MAGIC, 0xe0, struct pdp_arg)
#define IOCTL_PDP_DEACTIVATE		_IOW(IOC_MZ2_MAGIC, 0xe1, struct pdp_arg)
#define IOCTL_PDP_ADJUST		_IOW(IOC_MZ2_MAGIC, 0xe2, int)
#define IOCTL_PDP_TXSTART		_IO(IOC_MZ2_MAGIC, 0xe3)
#define IOCTL_PDP_TXSTOP		_IO(IOC_MZ2_MAGIC, 0xe4)
#define IOCTL_PDP_CSDSTART		_IO(IOC_MZ2_MAGIC, 0xe5)
#define IOCTL_PDP_CSDSTOP		_IO(IOC_MZ2_MAGIC, 0xe6)

/*
 * structure definitions.
 */
struct pdp_arg {
	unsigned char	id;
	char		ifname[16];
} __attribute__ ((packed));

struct dpram_user_data {
	void __user *addr;
	unsigned int size;
};

struct pdp_header {
	u16	len;
	u8	id;
	u8	ctrl;
} __attribute__ ((packed));

#define PDP_START_BYTE		(0x7f)
#define PDP_STOP_BYTE		(0x7e)
#define PDP_ID_MAX		(0xff)

struct m_fifo {
	u16 *head;
	u16 *tail;
	u16 size;
	u16 avail;
	void *data;
} __attribute__ ((packed));

static inline void init_m_fifo(struct m_fifo *fifo, void *base,
			u16 head, u16 tail, u16 data, u16 size)
{
	fifo->head = base + head;
	fifo->tail = base + tail;
	fifo->data = base + data;
	fifo->size = size;
}

#define INIT_M_FIFO(fifo, type, dir, base) \
	init_m_fifo((fifo), (base), (OFF_##dir##_##type##_HEAD), \
		(OFF_##dir##_##type##_TAIL), (OFF_##dir##_##type##_DATA), \
		(SIZ_##dir##_##type##_DATA))

enum dpram_vdev_type {
	DPRAM_VTTY,
	DPRAM_VNET
};

struct dpram_pdp_vdev {
	struct dpram_device	*dev;
	unsigned int		id;
	enum dpram_vdev_type	type;
	void (*handle_rx)(struct dpram_pdp_vdev *, size_t);
	void 			*priv;
	struct list_head	list;
};

#define VTTY_TX_BUF_LEN		(PAGE_SIZE)
struct dpram_vtty_priv {
	struct dpram_pdp_vdev	vdev;
	struct tty_driver	*drv;
	struct tty_struct	*tty;
	struct circ_buf		tx_buf;
	int			open_count;
	struct mutex		lock;
	struct list_head	list;
};

struct dpram_device {
	struct m_fifo rx;
	struct m_fifo tx;

	struct wake_lock wakelock;
	struct miscdevice mdev;

	unsigned int rx_bytes;
	unsigned int tx_bytes;

	atomic_t opened;

	int ack_req;
	u32 ack_bits;
	u32 ack_req_bits;
	u32 send_bits;

	unsigned int idx;
};

enum dpram_index {
	DPRAM_FMT = 0,
	DPRAM_RAW,
	DPRAM_NUM
};

enum mailbox {
	MAILBOX_AB = 0,
	MAILBOX_BA,
	MAILBOX_NUM
};

struct dpram {
	struct device			*dev;
	struct device			*sec_dpram_dev;
	struct dpram_platform_data	*pdata;
	struct proc_dir_entry		*proc_entry;

	wait_queue_head_t	wq;
	struct mutex		ctl_lock;

	unsigned int onedram_irq;
	unsigned int phone_active_irq;
	unsigned int sim_detect_irq;

	struct resource *mem;
	struct resource *res;
	void __iomem	*base;
	phys_addr_t	phys_base;
	size_t		size;

	volatile u32	*onedram_sem;
	volatile u32	*onedram_mailbox[MAILBOX_NUM];

	enum dpram_status status;
	bool sim_state;
	u32 last_irq;
	u32 last_cmd;

	/* used for ramdump mode */
	int ramdump;
	unsigned ramdump_size;
	loff_t ramdump_pos;

	/* shared memory semaphore management */
	unsigned sem_req_count;
	unsigned sem_req_active;
	unsigned sem_owner;
	unsigned sem_signal_bits;

	char *dgs_buf;

	struct dpram_device	device[DPRAM_NUM];

	struct list_head	pdp_vdev_list;
	struct sk_buff_head	pdp_vnet_queue;
	struct list_head	pdp_vtty_queue;
	int			pdp_priority;
	struct mutex		pdp_ctl_lock;
	wait_queue_head_t	pdp_tx_wq;
	int			pdp_adjust;
	struct task_struct	*pdp_tx_task;
	struct task_struct	*pdp_rx_task;

	struct miscdevice	err_mdev;
	struct fasync_struct	*err_async;
	atomic_t		err_opened;
	char			*err_buf;
	size_t			err_avail;
	wait_queue_head_t	err_wq;
};

struct dpram_vdev_template {
	struct dpram_pdp_vdev *(*add_dev)(struct dpram_device *,
						unsigned int, const char *);
	void (*rm_dev)(struct dpram_pdp_vdev *);
	void (*get_name)(struct dpram_pdp_vdev *vdev, char *buf, size_t len);
};

char fw_path[2048];
module_param_string(fw_path, fw_path, 2048, 0);

#define DEFAULT_FW_PATH	"/radio/modem.bin"

/*
 * Utility functions
 */
static inline struct dpram *dev_to_dpr(struct dpram_device *dev)
{
	return container_of(dev, struct dpram, device[dev->idx]);
}

/*
 * OneDRAM access
 */
static inline unsigned int onedram_read_mailbox(struct dpram *dpr)
{
	unsigned int val;
	val = readl(dpr->onedram_mailbox[MAILBOX_AB]);
#ifdef DEBUG_MAILBOX
	dev_dbg(dpr->dev, "%s: %08x\n", __func__, val);
#endif
	return val;
}

static inline void onedram_write_mailbox(struct dpram *dpr, u32 cmd)
{
#ifdef DEBUG_MAILBOX
	dev_dbg(dpr->dev, "%s: %08x\n", __func__, cmd);
#endif
	dpr->last_cmd = cmd;
	writel(cmd, dpr->onedram_mailbox[MAILBOX_BA]);
}

static inline int onedram_semaphore_held(struct dpram *dpr)
{
	return readl(dpr->onedram_sem) != 0;
}

static inline void dpram_phone_request_sem(struct dpram *dpr)
{
	onedram_write_mailbox(dpr, INT_COMMAND(CMD_SMP_REQ));
}

/* Forward declaration */
static void dpram_update_state(struct dpram *dpr);

static int onedram_semaphore_request(struct dpram *dpr)
{
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	dpr->sem_req_count++;
	ret = dpr->sem_owner;
	if (!ret) {
		if (onedram_semaphore_held(dpr)) {
			/* surprise! we already have control */
			ret = dpr->sem_owner = 1;
			wake_up(&dpr->wq);
			dpram_update_state(dpr);
		} else {
			/* ask the modem for mmio access */
			if (dpram_phone_running(dpr))
				dpram_phone_request_sem(dpr);
		}
	}

	local_irq_restore(flags);

	return ret;
}

static inline void onedram_semaphore_release(struct dpram *dpr)
{
	writel(0, dpr->onedram_sem);
}

static void onedram_semaphore_put(struct dpram *dpr, unsigned bits)
{
	unsigned long flags;

	local_irq_save(flags);

	dpr->sem_req_count--;
	dpr->sem_signal_bits |= bits;
	if ((dpr->sem_req_count == 0) && dpram_phone_running(dpr)) {
		if (dpr->sem_signal_bits) {
			onedram_semaphore_release(dpr);
			onedram_write_mailbox(dpr,
					INT_NON_COMMAND(dpr->sem_signal_bits));
		} else if (dpr->sem_req_active) {
			onedram_semaphore_release(dpr);
			onedram_write_mailbox(dpr, INT_COMMAND(CMD_RES_ACTIVE));
		} else {
			onedram_semaphore_release(dpr);
			onedram_write_mailbox(dpr, INT_COMMAND(CMD_SMP_REP));
		}
		dpr->sem_owner = 0;
		dpr->sem_signal_bits = 0;
		dpr->sem_req_active = 0;
	}

	local_irq_restore(flags);
}

static int onedram_semaphore_owner_p(struct dpram *dpr)
{
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	ret = dpr->sem_owner || dpram_phone_offline(dpr);

	local_irq_restore(flags);

	return ret;
}

static int _onedram_semaphore_get(struct dpram *dpr)
{
	int retry = 20;
	int ret;

	do {
		if (onedram_semaphore_request(dpr))
			break;

		ret = wait_event_interruptible_timeout(dpr->wq,
				onedram_semaphore_owner_p(dpr), HZ / 10);
		if (!ret && !onedram_semaphore_held(dpr)) {
			onedram_semaphore_put(dpr, 0);
			continue;
		}

		break;
	} while (--retry);

	if (!onedram_semaphore_held(dpr) || !dpram_phone_running(dpr)) {
		onedram_semaphore_put(dpr, 0);
		return -ENODEV;
	}

	return 0;
}

#ifdef DEBUG
static inline int __onedram_semaphore_get(const char *func, struct dpram *dpr)
{
	int ret;
	ret = _onedram_semaphore_get(dpr);
	if (ret)
		dev_err(dpr->dev,
			"Failed to get onedram semaphore in %s (ret=%d)\n",
			func, ret);
	return ret;
}

#define onedram_semaphore_get(dpr)	__onedram_semaphore_get(__func__, dpr)
#else
#define onedram_semaphore_get(dpr)	_onedram_semaphore_get(dpr)
#endif

static inline void *onedram_addr(struct dpram *dpr, u32 offset)
{
	BUG_ON(!onedram_semaphore_held(dpr));

	return dpr->base + offset;
}

/*
 * general purpose fifo access routines
 */
typedef void * (*copyfunc)(void *, const void *, __kernel_size_t);

static void *x_copy_to_user(void *dst, const void *src, __kernel_size_t sz)
{
	if (copy_to_user((void __user *) dst, src, sz) != 0)
		pr_err("dpram: copy_to_user\n");
	return dst;
}

static void *x_copy_from_user(void *dst, const void *src, __kernel_size_t sz)
{
	if (copy_from_user(dst, (const void __user *) src, sz) != 0)
		pr_err("dpram: copy_from_user\n");
	return dst;
}

static void copy_to_tty(struct tty_struct *tty,
					const void *src, __kernel_size_t sz)
{
	int written;

	if (unlikely(sz == 1)) {
		tty_insert_flip_char(tty, *(const char *)src, TTY_NORMAL);
		return;
	}

	while (sz > 0) {
		written = tty_insert_flip_string(tty, (const char *)src, sz);
		sz -= written;
		src += written;
	}
}

/* Return count in buffer.  */
static inline u32 circ_cnt(u32 head, u32 tail, u32 size)
{
	u32 cnt;

	if (tail > head)
		cnt = size - (tail - head);
	else
		cnt = head - tail;

	return cnt;
}

/* Return space available, 0..size-1.  We always leave one free char
   as a completely full buffer has head == tail, which is the same as
   empty.  */
static inline u32 circ_space(u32 head, u32 tail, u32 size)
{
	return circ_cnt(tail, head + 1, size);
}

/* Return count up to the end of the buffer.  Carefully avoid
   accessing head and tail more than once, so they can change
   underneath us without returning inconsistent results.  */
static inline u32 circ_cnt_to_end(u32 head, u32 tail, u32 size)
{
	u32 cnt;

	if (tail > head)
		cnt = size - tail;
	else
		cnt = head - tail;

	return cnt;
}

/* Return space available up to the end of the buffer.  */
static inline u32 circ_space_to_end(u32 head, u32 tail, u32 size)
{
	u32 space;

	if (tail > head)
		space = tail - head - 1;
	else
		space = size - head;

	return space;
}

static unsigned _fifo_read(struct m_fifo *q, void *dst,
			   unsigned count, copyfunc copy)
{
	unsigned n;
	unsigned head = *q->head;
	unsigned tail = *q->tail;
	unsigned size = q->size;

	if (circ_cnt(head, tail, size) < count)
		return 0;

	n = circ_cnt_to_end(head, tail, size);

	if (likely(n >= count)) {
		copy(dst, q->data + tail, count);
	} else {
		copy(dst, q->data + tail, n);
		copy(dst + n, q->data, count - n);
	}
	*q->tail = (tail + count) % size;

	return count;
}

static unsigned _fifo_write(struct m_fifo *q, const void *src,
			    unsigned count, copyfunc copy)
{
	unsigned n;
	unsigned head = *q->head;
	unsigned tail = *q->tail;
	unsigned size = q->size;

	if (circ_space(head, tail, size) < count)
		return 0;

	n = circ_space_to_end(head, tail, size);

	if (likely(n >= count)) {
		copy(q->data + head, src, count);
	} else {
		copy(q->data + head, src, n);
		copy(q->data, src + n, count - n);
	}
	*q->head = (head + count) % size;

	return count;
}

static unsigned fifo_read_tty(struct m_fifo *q,
					struct tty_struct *dst, unsigned count)
{
	unsigned n;
	unsigned head = *q->head;
	unsigned tail = *q->tail;
	unsigned size = q->size;

	if (circ_cnt(head, tail, size) < count)
		return 0;

	n = circ_cnt_to_end(head, tail, size);

	if (likely(n >= count)) {
		copy_to_tty(dst, q->data + tail, count);
	} else {
		copy_to_tty(dst, q->data + tail, n);
		copy_to_tty(dst, q->data, count - n);
	}
	*q->tail = (tail + count) % size;

	return count;
}

static void __maybe_unused fifo_purge(struct m_fifo *q)
{
	*q->head = 0;
	*q->tail = 0;
}

static unsigned __maybe_unused fifo_skip(struct m_fifo *q, unsigned count)
{
	if (circ_cnt(*q->head, *q->tail, q->size) < count)
		return 0;
	*q->tail = (*q->tail + count) % q->size;
	return count;
}

#define fifo_read(q, dst, count) \
	_fifo_read(q, dst, count, memcpy)
#define fifo_read_user(q, dst, count) \
	_fifo_read(q, dst, count, x_copy_to_user)

#define fifo_write(q, src, count) \
	_fifo_write(q, src, count, memcpy)
#define fifo_write_user(q, src, count) \
	_fifo_write(q, src, count, x_copy_from_user)

#define fifo_count(mf) circ_cnt(*(mf)->head, *(mf)->tail, (mf)->size)
#define fifo_space(mf) circ_space(*(mf)->head, *(mf)->tail, (mf)->size)

static void __maybe_unused fifo_dump(const char *tag, struct m_fifo *q,
		      unsigned start, unsigned count)
{
	if (count > 64)
		count = 64;

	if ((start + count) <= q->size) {
		print_hex_dump_bytes(tag, DUMP_PREFIX_ADDRESS,
				     q->data + start, count);
	} else {
		print_hex_dump_bytes(tag, DUMP_PREFIX_ADDRESS,
				     q->data + start, q->size - start);
		print_hex_dump_bytes(tag, DUMP_PREFIX_ADDRESS,
				     q->data, count - (q->size - start));
	}
}

/*
 * State processing
 */
void dpram_update_state(struct dpram *dpr)
{
	BUG_ON(!onedram_semaphore_held(dpr));

	/* update our idea of space available in fifos */
	dpr->device[DPRAM_FMT].tx.avail =
				fifo_space(&dpr->device[DPRAM_FMT].tx);
	dpr->device[DPRAM_FMT].rx.avail =
				fifo_count(&dpr->device[DPRAM_FMT].rx);

	if (dpr->device[DPRAM_FMT].rx.avail)
		wake_lock(&dpr->device[DPRAM_FMT].wakelock);
	else
		wake_unlock(&dpr->device[DPRAM_FMT].wakelock);

	dpr->device[DPRAM_RAW].tx.avail =
				fifo_space(&dpr->device[DPRAM_RAW].tx);
	dpr->device[DPRAM_RAW].rx.avail =
				fifo_count(&dpr->device[DPRAM_RAW].rx);

	if (dpr->device[DPRAM_RAW].rx.avail)
		wake_lock(&dpr->device[DPRAM_RAW].wakelock);
	else
		wake_unlock(&dpr->device[DPRAM_RAW].wakelock);

	/* wake up blocked or polling read/write operations */
	wake_up(&dpr->wq);
}

void dpram_update_device(struct dpram_device *device)
{
	unsigned long flags;

	BUG_ON(!onedram_semaphore_held(dev_to_dpr(device)));

	local_irq_save(flags);

	device->tx.avail = fifo_space(&device->tx);
	device->rx.avail = fifo_count(&device->rx);

	if (device->rx.avail)
		wake_lock(&device->wakelock);
	else
		wake_unlock(&device->wakelock);

	local_irq_restore(flags);
}

/*
 * MTD and NVDATA
 */
static int dpram_phone_image_read(struct dpram *dpr)
{
	char buf[512];
	struct file *filp;
	int ret;
	void *addr;
	int len;
	int pos;

	filp = filp_open(fw_path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		dev_err(dpr->dev, "Failed to open modem firmware.\n");
		return PTR_ERR(filp);
	}

	addr = onedram_addr(dpr, OFF_IMAGE);
	len = SIZ_IMAGE;
	pos = OFF_FLASH_IMAGE;
	while (len > 0) {
		int to_copy = (sizeof(buf) > len) ? len : sizeof(buf);
		ret = kernel_read(filp, pos, buf, to_copy);
		if (ret < 0) {
			dev_err(dpr->dev,
				"Failed to read modem firmware (%d).\n", ret);
			goto error;
		}
		memcpy(addr, buf, ret);
		pos += ret;
		addr += ret;
		len -= ret;
	}

	ret = 0;

error:
	filp_close(filp, 0);

	return ret;
}

static int dpram_nvdata_load(struct dpram *dpr, struct dpram_user_data *param)
{
	struct mtd_info *mtd;
	size_t retlen = 0;
	int ret;

	mtd = get_mtd_device_nm("dgs");
	if (IS_ERR(mtd)) {
		dev_err(dpr->dev, "Cannot find dgs partition.\n");
		return PTR_ERR(mtd);
	}

	ret = mtd->read(mtd, 5*mtd->writesize,
					SIZ_DGS_INFO, &retlen, dpr->dgs_buf);
	if (ret || retlen != SIZ_DGS_INFO) {
		dev_err(dpr->dev, "Failed to read dgs block.\n");
		put_mtd_device(mtd);
		return (ret) ? ret : -EIO;
	}

	put_mtd_device(mtd);

	if (!onedram_semaphore_held(dpr)) {
		dev_err(dpr->dev, "%s: semaphore owned by CP.\n", __func__);
		return -EBUSY;
	}

	ret = copy_from_user(onedram_addr(dpr, OFF_NVDATA),
						param->addr, param->size);
	if (ret)
		return -EFAULT;

	memcpy(onedram_addr(dpr, OFF_DGS_INFO), dpr->dgs_buf, SIZ_DGS_INFO);
	dev_dbg(dpr->dev, "nvdata downloaded.\n");

	return 0;
}

/*
 * Phone power and init
 */
static int dpram_phone_image_load(struct dpram *dpr)
{
	int ret;

	gpio_set_value(dpr->pdata->gpio_cp_boot_sel, 0);
	gpio_set_value(dpr->pdata->gpio_usim_boot, 0);

	ret = wait_event_interruptible_timeout(dpr->wq,
					dpram_phone_bootloader(dpr), 5 * HZ);
	if (!ret) {
		dev_err(dpr->dev, "Waiting for bootloader timed out.\n");
		return -ENODEV;
	}
	if (ret < 0)
		return ret;

	if(!onedram_semaphore_held(dpr)) {
		dev_err(dpr->dev, "%s: semaphore owned by CP.\n", __func__);
		return -EBUSY;
	}

	ret = dpram_phone_image_read(dpr);
	if (ret) {
		dev_err(dpr->dev, "image read failed\n");
		return ret;
	}

	dpr->status = DPRAM_PHONE_IMAGE_LOADED;
	dev_dbg(dpr->dev, "image downloaded.\n");

	return 0;
}

static void dpram_phone_power_off(struct dpram *dpr)
{
	dev_info(dpr->dev, "Phone power off.\n");

	dpr->status = DPRAM_PHONE_OFF;
	wake_up(&dpr->wq);

	gpio_set_value(dpr->pdata->gpio_phone_on, 0);
	gpio_set_value(dpr->pdata->gpio_phone_rst_n, 0);
}

static void dpram_phone_power_on(struct dpram *dpr)
{
	unsigned long flags;

	dev_info(dpr->dev, "Phone power on...\n");

	local_irq_save(flags);

	dpr->sem_owner = 0;
	dpr->status = DPRAM_PHONE_OFF;
	wake_up(&dpr->wq);

	gpio_set_value(dpr->pdata->gpio_pda_active, 0);

	(void) onedram_read_mailbox(dpr);
	onedram_write_mailbox(dpr, 0);

	local_irq_restore(flags);

	gpio_set_value(dpr->pdata->gpio_cp_boot_sel, 1);
	gpio_set_value(dpr->pdata->gpio_usim_boot, 1);

	gpio_set_value(dpr->pdata->gpio_phone_on, 0);
	gpio_set_value(dpr->pdata->gpio_phone_rst_n, 0);
	msleep(400);

	gpio_set_value(dpr->pdata->gpio_phone_on, 1);
	msleep(60);

	gpio_set_value(dpr->pdata->gpio_phone_rst_n, 1);
	msleep(1000);

	gpio_set_value(dpr->pdata->gpio_phone_on, 0);
	msleep(200);

	gpio_set_value(dpr->pdata->gpio_pda_active, 1);

	dpr->status = DPRAM_PHONE_POWER_ON;

	dev_dbg(dpr->dev, "Phone powered on.\n");
}

static int dpram_phone_boot_start(struct dpram *dpr, int ramdump)
{
	int ret;

	dev_dbg(dpr->dev, "dpram_phone_boot_start() %s\n",
					ramdump ? "ramdump" : "normal");

	if (dpr->status != DPRAM_PHONE_IMAGE_LOADED) {
		dev_err(dpr->dev, "image not loaded\n");
		return -EINVAL;
	}

	if (!onedram_semaphore_held(dpr)) {
		dev_err(dpr->dev, "%s: semaphore owned by CP.\n", __func__);
		return -EBUSY;
	}

	onedram_semaphore_release(dpr);

	if (ramdump) {
		dpr->status = DPRAM_PHONE_BOOTING_RAMDUMP;
		dpr->ramdump_size = 0;
		dpr->ramdump_pos = 0;
		onedram_write_mailbox(dpr, DPRAM_CMD_RAMDUMP_START);
		ret = wait_event_timeout(dpr->wq,
				dpr->status == DPRAM_PHONE_DUMPING, 25 * HZ);
		if (ret == 0)
			return -ENODEV;
	} else {
		dpr->status = DPRAM_PHONE_BOOTING_NORMAL;
		onedram_write_mailbox(dpr, DPRAM_CMD_BINARY_LOAD);
		ret = wait_event_timeout(dpr->wq,
			dpr->status == DPRAM_PHONE_INITIALIZING, 25 * HZ);
		if (ret == 0)
			return -ENODEV;
	}

	dev_dbg(dpr->dev, "dpram_phone_boot_start() DONE\n");
	return 0;
}

static int dpram_phone_getstatus(struct dpram *dpr)
{
	return gpio_get_value(dpr->pdata->gpio_phone_active);
}

static void dpram_phone_reset(struct dpram *dpr)
{
	dev_info(dpr->dev, "Phone reset.\n");

	dpr->status = DPRAM_PHONE_POWER_ON;
	wake_up(&dpr->wq);

	gpio_set_value(dpr->pdata->gpio_pda_active, 0);

	(void) onedram_read_mailbox(dpr);
	onedram_write_mailbox(dpr, 0);

	gpio_set_value(dpr->pdata->gpio_cp_boot_sel, 1);
	gpio_set_value(dpr->pdata->gpio_usim_boot, 1);

	gpio_set_value(dpr->pdata->gpio_phone_rst_n, 0);
	msleep(100);

	gpio_set_value(dpr->pdata->gpio_phone_rst_n, 1);
	msleep(200);

	gpio_set_value(dpr->pdata->gpio_pda_active, 1);

	dpr->status = DPRAM_PHONE_POWER_ON;
}

/*
 * Interrupts
 */
static void dpram_handle_offline(struct dpram *dpr, unsigned cmd)
{
	switch (dpr->status) {
	case DPRAM_PHONE_POWER_ON:
	case DPRAM_PHONE_BOOTLOADER:
		if (cmd == DPRAM_MSG_SBL_DONE) {
			dev_info(dpr->dev, "waiting for image\n");
			dpr->status = DPRAM_PHONE_BOOTLOADER;
			wake_up(&dpr->wq);
			return;
		}
		break;
	case DPRAM_PHONE_BOOTING_NORMAL:
		if (cmd == DPRAM_MSG_BINARY_DONE) {
			dev_info(dpr->dev, "binary load done\n");
			dpr->status = DPRAM_PHONE_INITIALIZING;
			wake_up(&dpr->wq);
			return;
		}
		break;
	case DPRAM_PHONE_BOOTING_RAMDUMP:
	case DPRAM_PHONE_DUMPING:
		if (cmd == DPRAM_MSG_RAMDUMP_LARGE) {
			dpr->status = DPRAM_PHONE_DUMPING;
			dpr->ramdump_size = RAMDUMP_LARGE_SIZE;
			wake_up(&dpr->wq);
			dev_info(dpr->dev, "ramdump - %d bytes available\n",
							dpr->ramdump_size);
			return;
		}
		if (cmd == DPRAM_MSG_RAMDUMP_SMALL) {
			dpr->status = DPRAM_PHONE_DUMPING;
			dpr->ramdump_size = RAMDUMP_SMALL_SIZE;
			wake_up(&dpr->wq);
			dev_info(dpr->dev, "ramdump - %d bytes available\n",
							dpr->ramdump_size);
			return;
		}
		break;
	case DPRAM_PHONE_INITIALIZING:
		if ((cmd & 0xffff) == INT_COMMAND(CMD_PHONE_START)) {
			dev_info(dpr->dev, "Phone started.\n");
                        if (onedram_semaphore_held(dpr)) {
				volatile unsigned short *reg;
				memset(onedram_addr(dpr, 0), 0, SIZ_DPRAM);
				reg = onedram_addr(dpr, OFF_MAGIC);
				*reg = 0x00aa;
				reg = onedram_addr(dpr, OFF_ACCESS);
				*reg = 0x0001;
			} else {
				dev_dbg(dpr->dev, "Semaphore not held, could not initialize dpram memory.\n");
			}
			onedram_write_mailbox(dpr, INT_COMMAND(CMD_INIT_END));
			dpr->status = DPRAM_PHONE_RUNNING;
			wake_up(&dpr->wq);
			return;
		}
		break;
	default:
		break;
	}

	dev_dbg(dpr->dev,"invalid offline interrupt %08x in state %d\n",
							cmd, dpr->status);
}

static void dpram_handle_command(struct dpram *dpr, u32 cmd)
{
	if (unlikely(!dpram_phone_running(dpr))) {
		dpram_handle_offline(dpr, cmd);
		return;
	}

	if (!(cmd & INT_MASK_VALID)) {
		dev_warn(dpr->dev, "Unhandled offline command %08x\n", cmd);
		return;
	}

	if (!(cmd & INT_MASK_COMMAND)) {
		if (cmd & INT_MASK_REQ_ACK_F)
			dpr->device[DPRAM_FMT].ack_req = 1;
		if (cmd & INT_MASK_REQ_ACK_R)
			dpr->device[DPRAM_RAW].ack_req = 1;
		return;
	}

	switch (cmd & 15) {
	case CMD_SMP_REQ:
		dev_dbg(dpr->dev, "Phone requested semaphore.\n");
		if (!onedram_semaphore_held(dpr)) {
			/* Sometimes the modem may ask for the
				* sem when it already owns it.  Humor
				* it and ack that request.
				*/
			onedram_write_mailbox(dpr,
					INT_COMMAND(CMD_SMP_REP));
		} else if (dpr->sem_req_count == 0) {
			/* No references? Give it to the modem. */
			dpram_update_state(dpr);
			dpr->sem_owner = 0;
			onedram_semaphore_release(dpr);
			onedram_write_mailbox(dpr,
					INT_COMMAND(CMD_SMP_REP));
			break;
		} else {
			/* Busy now, will be returned later. */
			break;
		}
		break;
	case CMD_REQ_ACTIVE:
		dev_dbg(dpr->dev, "Phone requested semaphore (CMD_REQ_ACTIVE).\n");
		if (!onedram_semaphore_held(dpr)) {
			/* Sometimes the modem may ask for the
				* sem when it already owns it.  Humor
				* it and ack that request.
				*/
			onedram_write_mailbox(dpr,
					INT_COMMAND(CMD_RES_ACTIVE));
		} else if (dpr->sem_req_count == 0) {
			/* No references? Give it to the modem. */
			dpram_update_state(dpr);
			dpr->sem_owner = 0;
			onedram_semaphore_release(dpr);
			onedram_write_mailbox(dpr,
					INT_COMMAND(CMD_RES_ACTIVE));
			break;
		} else {
			/* Busy now, remember the modem needs it. */
			dpr->sem_req_active = 1;
		}
		break;
	case CMD_SMP_REP:
		dev_dbg(dpr->dev, "Phone returned semaphore.\n");
		break;
	case CMD_RESET:
		dev_err(dpr->dev, "Phone reset.\n");
		dpr->status = DPRAM_PHONE_CRASHED;
		wake_up(&dpr->wq);
		break;
	case CMD_ERR_DISPLAY: {
		dev_err(dpr->dev, "Phone error.\n");
		dpr->status = DPRAM_PHONE_CRASHED;
		if (onedram_semaphore_held(dpr)) {
			dpr->err_buf[0] = '1';
			dpr->err_buf[1] = ' ';
			memcpy(dpr->err_buf + 2,
				onedram_addr(dpr, OFF_RX_FMT_DATA),
				SIZ_ERROR_MSG);
			dpr->err_avail = 1;
			dev_err(dpr->dev, "$$$ %s $$$\n", dpr->err_buf);
			wake_up(&dpr->err_wq);
			kill_fasync(&dpr->err_async, SIGIO, POLL_IN);
		}
		wake_up(&dpr->wq);
		break;
	}
	case CMD_SUSPEND:
		dev_dbg(dpr->dev, "Phone suspended.\n");
		break;
	default:
		dev_dbg(dpr->dev,
				"Unhandled command: %08x\n", cmd);
	}
}

static irqreturn_t dpram_mailbox_irq(int irq, void *dev_id)
{
	struct dpram *dpr = dev_id;

	if (gpio_get_value(dpr->pdata->gpio_onedram_int_n))
		return IRQ_NONE;

	do {
		u32 cmd = onedram_read_mailbox(dpr);
		dpram_handle_command(dpr, cmd);
		dpr->last_irq = cmd;
	} while (!gpio_get_value(dpr->pdata->gpio_onedram_int_n));

	if (unlikely(!dpram_phone_running(dpr)) || !onedram_semaphore_held(dpr))
		return IRQ_HANDLED;

	/*
	 * On *any* interrupt from the modem it may have given
	 * us ownership of the mmio hw semaphore.  If that
	 * happens, we should claim the semaphore if we have
	 * threads waiting for it and we should process any
	 * messages that the modem has enqueued in its fifos.
	 */
	if (!dpr->sem_owner) {
		dpram_update_state(dpr);
		if (dpr->sem_req_count) {
			dpr->sem_owner = 1;
			wake_up(&dpr->wq);
		}
	}

	/*
	* If we have a signal to send and we're not
	* hanging on to the mmio hw semaphore, give
	* it back to the modem and send the signal.
	* Otherwise this will happen when we give up
	* the mmio hw sem in onedram_semaphore_put().
	*/
	if (dpr->sem_signal_bits && !dpr->sem_owner) {
		onedram_semaphore_release(dpr);
		onedram_write_mailbox(dpr,
				INT_NON_COMMAND(dpr->sem_signal_bits));
		dpr->sem_signal_bits = 0;
	}

	return IRQ_HANDLED;
}

static irqreturn_t dpram_phone_active_irq(int irq, void *dev_id)
{
	struct dpram *dpr = dev_id;
	unsigned long flags;

	if (gpio_get_value(dpr->pdata->gpio_phone_active)) {
		dev_dbg(dpr->dev, "Phone activated in state %d.\n", dpr->status);
		return IRQ_HANDLED;
	}

	if (!dpram_phone_running(dpr)) {
		dev_dbg(dpr->dev, "Phone reset in state %d.\n", dpr->status);
		return IRQ_HANDLED;
	}

	dev_err(dpr->dev, "Phone reset while running, needs restart.\n");

	local_irq_save(flags);

	memset(dpr->err_buf, 0, SIZ_ERROR_HDR + SIZ_ERROR_MSG);
	strcpy(dpr->err_buf, "9 $PHONE-RESET");
	dpr->err_avail = 1;
	dpr->status = DPRAM_PHONE_CRASHED;

	local_irq_restore(flags);

	wake_up(&dpr->wq);
	wake_up(&dpr->err_wq);
	kill_fasync(&dpr->err_async, SIGIO, POLL_IN);

	return IRQ_HANDLED;
}

static irqreturn_t dpram_sim_irq(int irq, void *dev_id)
{
	struct dpram *dpr = dev_id;
	unsigned long flags;
	int state, new_state;
	int debounce_count = 10;
	int debounce_retries = 10;

	state = gpio_get_value(dpr->pdata->gpio_sim_detect_n);

	if (state == dpr->sim_state)
		return IRQ_HANDLED;

	do {
		msleep(20);

		new_state = gpio_get_value(dpr->pdata->gpio_sim_detect_n);
		if (new_state != state) {
			state = new_state;
			debounce_count = 10;
			--debounce_retries;
			continue;
		}
	} while (--debounce_count && debounce_retries);

	if (debounce_count && !debounce_retries) {
		dev_err(dpr->dev, "SIM detect signal failed to stabilize. Disabling SIM card detection.\n");
		disable_irq_nosync(irq);
		return IRQ_HANDLED;
	}

	dev_dbg(dpr->dev, "SIM state = %d.\n", state);

	local_irq_save(flags);

	memset(dpr->err_buf, 0, SIZ_ERROR_HDR + SIZ_ERROR_MSG);
	strcpy(dpr->err_buf, (state) ? "2 $SIM-DETACHED" : "3 $SIM-ATTACHED");

	dpr->err_avail = 1;
	dpr->sim_state = state;

	local_irq_restore(flags);

	wake_up(&dpr->err_wq);
	kill_fasync(&dpr->err_async, SIGIO, POLL_IN);

	return IRQ_HANDLED;
}

/*
 * Error device
 */
static int dpram_err_read(struct file *filp, char *buf,
						size_t count, loff_t *ppos)
{
	struct dpram *dpr = filp->private_data;
	unsigned long flags;
	int ncopy;

	local_irq_save(flags);

	while (!dpr->err_avail) {
		local_irq_restore(flags);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ncopy = wait_event_interruptible(dpr->err_wq, dpr->err_avail);
		if (ncopy < 0)
			return ncopy;

		local_irq_save(flags);
	}

	ncopy = SIZ_ERROR_MSG + SIZ_ERROR_HDR;
	if (count < ncopy)
		ncopy = count;

	if (copy_to_user(buf, dpr->err_buf, ncopy))
		ncopy = -EFAULT;
	else
		dpr->err_avail = 0;

	local_irq_restore(flags);

	return ncopy;
}

static int dpram_err_fasync(int fd, struct file *filp, int mode)
{
	struct dpram *dpr = filp->private_data;

	return fasync_helper(fd, filp, mode, &dpr->err_async);
}

static unsigned int dpram_err_poll(struct file *filp, poll_table *wait)
{
	struct dpram *dpr = filp->private_data;

	poll_wait(filp, &dpr->err_wq, wait);

	return ((dpr->err_avail) ? (POLLIN | POLLRDNORM) : 0);
}

static int dpram_err_open(struct inode *in, struct file *filp)
{
	struct dpram *dpr =
		container_of(filp->private_data, struct dpram, err_mdev);

	if (atomic_xchg(&dpr->err_opened, 1))
		return -EBUSY;

	filp->private_data = dpr;

	return 0;
}

static int dpram_err_release(struct inode *in, struct file *filp)
{
	struct dpram *dpr = filp->private_data;

	dpram_err_fasync(-1, filp, 0);
	atomic_dec(&dpr->err_opened);

	return 0;
}

static struct file_operations dpram_err_fops = {
	.owner		= THIS_MODULE,
	.open		= dpram_err_open,
	.release	= dpram_err_release,
	.read		= dpram_err_read,
	.fasync		= dpram_err_fasync,
	.poll		= dpram_err_poll,
};

static void dpram_error_device_unregister(struct dpram *dpr)
{
	misc_deregister(&dpr->err_mdev);
}

static int dpram_error_device_register(struct dpram *dpr)
{
	int ret = 0;

	dpr->err_mdev.name = "dpramerr";
	dpr->err_mdev.minor = MISC_DYNAMIC_MINOR;
	dpr->err_mdev.fops = &dpram_err_fops;

	dpr->err_buf = kzalloc(SIZ_ERROR_HDR + SIZ_ERROR_MSG + 1, GFP_KERNEL);
	if (!dpr->err_buf) {
		dev_err(dpr->dev, "Failed to allocate dpram error buffer.\n");
		return -ENOMEM;
	}

	ret = misc_register(&dpr->err_mdev);
	if (ret) {
		dev_err(dpr->dev, "Failed to register error misc device.\n");
		goto err_dev;
	}

	return 0;

err_dev:
	kfree(dpr->err_buf);
	return ret;
}

/*
 * Character device
 */
static int dpram_char_open(struct inode *inode, struct file *filp)
{
	struct dpram_device *device =
		container_of(filp->private_data, struct dpram_device, mdev);

	if (atomic_xchg(&device->opened, 1))
		return -EBUSY;

	filp->private_data = device;

	return 0;
}

static int dpram_char_release(struct inode *inode, struct file *filp)
{
	struct dpram_device *device = filp->private_data;

	atomic_dec(&device->opened);

	return 0;
}

static long dpram_char_ioctl(struct file *filp,
					unsigned int cmd, unsigned long arg)
{
	struct dpram_device *dev = filp->private_data;
	struct dpram *dpr = dev_to_dpr(dev);
	unsigned int val;
	int ret = 0;

	if((ret = mutex_lock_interruptible(&dpr->ctl_lock)) < 0)
		return ret;

	switch (cmd) {
	case IOCTL_DPRAM_PHONE_GETSTATUS:
		val = dpram_phone_getstatus(dpr);
		ret = copy_to_user((void __user *)arg, &val, sizeof(val));
		if (ret)
			ret = -EFAULT;
		break;

	case IOCTL_DPRAM_PHONE_POWON:
		dpram_phone_power_on(dpr);
		break;

	case IOCTL_DPRAM_PHONEIMG_LOAD:
		dpram_phone_image_load(dpr);
		break;

	case IOCTL_DPRAM_PHONE_BOOTSTART:
		ret = dpram_phone_boot_start(dpr, dpr->ramdump);
		break;

	case IOCTL_DPRAM_NVDATA_LOAD: {
		struct dpram_user_data param;
		ret = copy_from_user(&param, (void *)arg, sizeof(param));
		if (ret) {
			ret = -EFAULT;
			break;
		}
		dpram_nvdata_load(dpr, &param);
		break;
	}
	case IOCTL_DPRAM_GET_DGS_INFO: {
		ret = copy_to_user((void __user *)arg,
						dpr->dgs_buf, SIZ_DGS_INFO);
		if (ret)
			ret = -EFAULT;
		break;
	}
	case IOCTL_DPRAM_PHONE_RESET:
		dpram_phone_reset(dpr);
		break;

	case IOCTL_DPRAM_PHONE_OFF:
		dpram_phone_power_off(dpr);
		break;

	case IOCTL_DPRAM_PHONE_RAMDUMP_ON:
		dpr->ramdump = 1;
		break;

	case IOCTL_DPRAM_PHONE_RAMDUMP_OFF:
		dpr->ramdump = 0;
		break;

	/* Dummies for broken Samsung RIL */
	case TCGETS:
	case TCSETS:
	case TIOCMGET:
	case TIOCMSET:
	case IOCTL_DPRAM_PHONE_ON:
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&dpr->ctl_lock);

	return ret;
}

static ssize_t dpram_char_read(struct file *filp, char __user *buf,
						size_t count, loff_t *ppos)
{
	struct dpram_device *dev = filp->private_data;
	struct dpram *dpr = dev_to_dpr(dev);
	unsigned long flags;
	size_t to_copy;
	int ret;

	ret = onedram_semaphore_get(dpr);
	if (ret)
		return ret;

	dpram_update_device(dev);

	dev_dbg(dpr->dev, "%s: dev %d, requested %d, available %d\n",
				__func__, dev->idx, count, dev->rx.avail);

	while (!dev->rx.avail) {
		onedram_semaphore_put(dpr, 0);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		dev_dbg(dpr->dev, "%s: dev %d, waiting for data\n",
							__func__, dev->idx);

		ret = wait_event_interruptible(dpr->wq,
				dev->rx.avail || dpram_phone_offline(dpr));
		if (ret < 0)
			return ret;

		ret = onedram_semaphore_get(dpr);
		if (ret)
			return ret;
	}

	to_copy = (dev->rx.avail > count) ? count : dev->rx.avail;

	fifo_read_user(&dev->rx, buf, to_copy);

	dpram_update_device(dev);

	count -= to_copy;
	dev->rx_bytes += to_copy;

	local_irq_save(flags);
	if (dev->ack_req) {
		onedram_semaphore_put(dpr, dev->ack_bits);
		dev->ack_req = 0;
	} else {
		onedram_semaphore_put(dpr, 0);
	}
	local_irq_restore(flags);

	return to_copy;
}

static ssize_t dpram_char_write(struct file *filp, const char __user *buf,
						size_t count, loff_t *ppos)
{
	struct dpram_device *dev = filp->private_data;
	struct dpram *dpr = dev_to_dpr(dev);
	size_t to_copy;
	int ret;

	ret = onedram_semaphore_get(dpr);
	if (ret)
		return ret;

	dpram_update_device(dev);

	dev_dbg(dpr->dev, "%s: dev %d, requested %d, available %d\n",
				__func__, dev->idx, count, dev->tx.avail);

	while (!dev->tx.avail) {
		onedram_semaphore_put(dpr, 0);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		dev_dbg(dpr->dev, "%s: dev %d, waiting for space\n",
							__func__, dev->idx);

		ret = wait_event_interruptible(dpr->wq,
				dev->tx.avail || dpram_phone_offline(dpr));
		if (ret < 0)
			return ret;

		ret = onedram_semaphore_get(dpr);
		if (ret)
			return ret;
	}

	to_copy = (dev->tx.avail > count) ? count : dev->tx.avail;

	fifo_write_user(&dev->tx, buf, to_copy);

	dpram_update_device(dev);

	count -= to_copy;
	dev->tx_bytes += to_copy;

	if (!count)
		onedram_semaphore_put(dpr, dev->send_bits);
	else
		onedram_semaphore_put(dpr, dev->ack_req_bits);

	return to_copy;
}

static unsigned int dpram_char_poll(struct file *filp, poll_table *wait)
{
	struct dpram_device *dev = filp->private_data;
	struct dpram *dpr = dev_to_dpr(dev);
	unsigned long flags;
	int ret = 0;

	dev_dbg(dpr->dev, "%s: dev %d\n", __func__, dev->idx);

	poll_wait(filp, &dpr->wq, wait);

	local_irq_save(flags);

	if (dev->rx.avail)
		ret |= POLLIN | POLLRDNORM;

	if (dev->tx.avail)
		ret |= POLLOUT | POLLWRNORM;

	local_irq_restore(flags);

	return ret;
}

static struct file_operations dpram_char_fops = {
	.owner		= THIS_MODULE,
	.open		= dpram_char_open,
	.release	= dpram_char_release,
	.read		= dpram_char_read,
	.write		= dpram_char_write,
	.poll		= dpram_char_poll,
	.unlocked_ioctl	= dpram_char_ioctl,
};

static int dpram_char_register(struct dpram_device *dev)
{
	dev->mdev.name = "dpram";
	dev->mdev.minor = MISC_DYNAMIC_MINOR;
	dev->mdev.fops = &dpram_char_fops;

	return misc_register(&dev->mdev);
}

static void dpram_char_unregister(struct dpram_device *dev)
{
	misc_deregister(&dev->mdev);
}

/*
 * PDP virtual device helpers
 */
static struct dpram_pdp_vdev *dpram_pdp_get_vdev(struct dpram *dpr,
								unsigned int id)
{
	struct dpram_pdp_vdev *vdev;

	list_for_each_entry(vdev, &dpr->pdp_vdev_list, list) {
		if (vdev->id == id)
			return vdev;
	}

	return 0;
}

/*
 * PDP packet router
 */
static int dpram_wait_for_fifo(struct dpram *dpr, struct dpram_device *dev,
			struct m_fifo *fifo, unsigned int bits, size_t len)
{
	int ret;

	dpram_update_device(dev);

	while (fifo->avail < len) {
		onedram_semaphore_put(dpr, bits);

		ret = wait_event_interruptible(dpr->wq, fifo->avail >= len);
		if (ret < 0)
			return ret;

		ret = onedram_semaphore_get(dpr);
		if (ret)
			return ret;

		bits = 0;
	}

	return 0;
}

static void dpram_pdp_send_hdr(struct dpram_device *dev,
					struct dpram_pdp_vdev *vdev, size_t len)
{
	struct pdp_header hdr;
	struct dpram *dpr = dev_to_dpr(dev);

	hdr.len		= len + sizeof(struct pdp_header);
	hdr.id		= vdev->id;
	hdr.ctrl	= 0;

	dev_dbg(dpr->dev, "Transmitting PDP packet with ID %d and LEN %d.\n",
							hdr.id, hdr.len);

	fifo_write(&dev->tx, &hdr, sizeof(struct pdp_header));
}

static int dpram_pdp_process_vnet_tx(struct dpram *dpr,
						struct dpram_device *dev)
{
	const unsigned char start = PDP_START_BYTE;
	const unsigned char stop = PDP_STOP_BYTE;
	struct dpram_pdp_vdev *vdev;
	struct sk_buff *skb;
	int ret;

	/* Starts with OneDRAM semaphore held */
	skb = skb_dequeue(&dpr->pdp_vnet_queue);

	/* Releases the semaphore and sleeps if no space available */
	ret = dpram_wait_for_fifo(dpr, dev, &dev->tx, 0,
				1 + sizeof(struct pdp_header) + skb->len + 1);
	if (ret) {
		dev_kfree_skb(skb);
		return ret;
	}

	/* Semaphore is held here */
	vdev = netdev_priv(skb->dev);

	fifo_write(&dev->tx, &start, 1);
	++dev->tx_bytes;
	dpram_pdp_send_hdr(dev, vdev, skb->len);
	dev->tx_bytes += sizeof(struct pdp_header);
	fifo_write(&dev->tx, skb->data, skb->len);
	dev->tx_bytes += skb->len;
	fifo_write(&dev->tx, &stop, 1);
	++dev->tx_bytes;

	++skb->dev->stats.tx_packets;
	skb->dev->stats.tx_bytes += skb->len;

	dev_kfree_skb(skb);

	/* Exits with OneDRAM semaphore held */
	return 0;
}

static int dpram_pdp_process_vtty_tx(struct dpram *dpr,
						struct dpram_device *dev)
{
	const unsigned char start = PDP_START_BYTE;
	const unsigned char stop = PDP_STOP_BYTE;
	struct dpram_vtty_priv *vtty;
	unsigned long flags;
	unsigned int len, to_end;
	struct circ_buf *circ;
	int head, tail;
	int ret;

	/* Starts with OneDRAM semaphore held */
	local_irq_save(flags);

	vtty = list_first_entry(&dpr->pdp_vtty_queue,
						struct dpram_vtty_priv, list);
	circ = &vtty->tx_buf;
	head = circ->head;
	tail = circ->tail;

	local_irq_restore(flags);

	len = CIRC_CNT(head, tail, VTTY_TX_BUF_LEN);
	to_end = CIRC_CNT_TO_END(head, tail, VTTY_TX_BUF_LEN);

	if (len > ETH_DATA_LEN)
		len = ETH_DATA_LEN;

	/* Releases the semaphore and sleeps if no space available */
	ret = dpram_wait_for_fifo(dpr, dev, &dev->tx, 0,
				1 + sizeof(struct pdp_header) + len + 1);
	if (ret)
		return ret;

	fifo_write(&dev->tx, &start, 1);
	dpram_pdp_send_hdr(dev, &vtty->vdev, len);
	if (likely(to_end >= len)) {
		fifo_write(&dev->tx, circ->buf + tail, len);
	} else {
		fifo_write(&dev->tx, circ->buf + tail, to_end);
		fifo_write(&dev->tx, circ->buf, len - to_end);
	}
	fifo_write(&dev->tx, &stop, 1);

	tail = (tail + len) % VTTY_TX_BUF_LEN;
	dev->tx_bytes += 1 + sizeof(struct pdp_header) + len + 1;

	local_irq_save(flags);

	list_del_init(&vtty->list);

	circ->tail = tail;
	len = CIRC_CNT(circ->head, circ->tail, VTTY_TX_BUF_LEN);
	if (len)
		list_add_tail(&vtty->list, &dpr->pdp_vtty_queue);

	local_irq_restore(flags);

	/* Exits with OneDRAM semaphore held */
	return 0;
}

static int dpram_pdp_process_tx(struct dpram *dpr, struct dpram_device *dev)
{
	int ret;

	if (!list_empty(&dpr->pdp_vtty_queue)
				&& !skb_queue_empty(&dpr->pdp_vnet_queue)) {
		if (dpr->pdp_priority)
			ret = dpram_pdp_process_vnet_tx(dpr, dev);
		else
			ret = dpram_pdp_process_vtty_tx(dpr, dev);

		dpr->pdp_priority ^= 1;
		return ret;
	}

	if (!list_empty(&dpr->pdp_vtty_queue))
		return dpram_pdp_process_vtty_tx(dpr, dev);

	return dpram_pdp_process_vnet_tx(dpr, dev);
}

static int dpram_pdp_tx_thread(void *data)
{
	struct dpram_device *dev = data;
	struct dpram *dpr = dev_to_dpr(dev);
	unsigned long flags;
	int ret;

	allow_signal(SIGTERM);

wait_for_phone:
	ret = wait_event_interruptible(dpr->wq, dpram_phone_running(dpr));
	if (ret)
		return 0;

	do {
		/* Semaphore isn't held here */
		local_irq_save(flags);
		while (list_empty(&dpr->pdp_vtty_queue)
				&& skb_queue_empty(&dpr->pdp_vnet_queue)) {
			local_irq_restore(flags);
			ret = wait_event_interruptible(dpr->pdp_tx_wq,
					!list_empty(&dpr->pdp_vtty_queue) ||
					!skb_queue_empty(&dpr->pdp_vnet_queue));
			if (ret)
				goto finish;
			local_irq_save(flags);
		}
		local_irq_restore(flags);

		ret = onedram_semaphore_get(dpr);
		if (ret == -ENODEV) {
			dev_warn(dpr->dev, "%s: Phone offline.\n", __func__);
			goto wait_for_phone;
		}
		if (ret)
			goto finish;
		/* Semaphore is held here */

		/* Exits with semaphore released on error */
		ret = dpram_pdp_process_tx(dpr, dev);
		if (ret == -ENODEV) {
			dev_warn(dpr->dev, "%s: Phone offline.\n", __func__);
			goto wait_for_phone;
		}
		if (ret)
			goto finish;

		onedram_semaphore_put(dpr, dev->send_bits);
		/* Semaphore is no longer held here */
	} while(!kthread_should_stop());

finish:
	skb_queue_purge(&dpr->pdp_vnet_queue);

	return ret;
}

static void dpram_pdp_demux_packet(struct dpram_device *dev,
							struct pdp_header *hdr)
{
	struct dpram *dpr = dev_to_dpr(dev);
	struct dpram_pdp_vdev *vdev;
	unsigned long flags;

	dev_dbg(dpr->dev, "Received PDP packet with ID %d and LEN %d.\n",
							hdr->id, hdr->len);

	local_irq_save(flags);
	vdev = dpram_pdp_get_vdev(dpr, hdr->id);
	local_irq_restore(flags);
	if (!vdev || !vdev->handle_rx) {
		dev_warn(dpr->dev, "%s: Skipping unsubscribed packet ID %d (%d bytes)\n",
						__func__, hdr->id, hdr->len);
		fifo_skip(&dev->rx, hdr->len - sizeof(struct pdp_header));
		return;
	}

	vdev->handle_rx(vdev, hdr->len - sizeof(struct pdp_header));
}

static int dpram_pdp_rx_thread(void *data)
{
	struct dpram_device *dev = data;
	struct dpram *dpr = dev_to_dpr(dev);
	struct pdp_header hdr;
	unsigned char start, stop;
	unsigned int bits = 0;
	unsigned long flags;
	int ret;

	allow_signal(SIGTERM);

wait_for_phone:
	ret = wait_event_interruptible(dpr->wq, dpram_phone_running(dpr));
	if (ret)
		return 0;

	ret = onedram_semaphore_get(dpr);
	if (ret == -ENODEV) {
		dev_warn(dpr->dev, "%s: Phone offline.\n", __func__);
		goto wait_for_phone;
	}
	if (ret)
		return ret;

	/* The semaphore is held here */
	do {
		/* Releases the semaphore and sleeps if no data available */
		ret = dpram_wait_for_fifo(dpr, dev, &dev->rx, bits,
						sizeof(struct pdp_header) + 1);
		if (ret == -ENODEV) {
			dev_warn(dpr->dev, "%s: Phone offline.\n", __func__);
			goto wait_for_phone;
		}
		if (ret)
			return ret;

		/* The semaphore is held here */
		fifo_read(&dev->rx, &start, 1);
		++dev->rx_bytes;
		if (start != PDP_START_BYTE) {
			/* Something wrong happened (sync lost?) */
			dev_err(dpr->dev, "%s: Wrong start byte read. (Overrun?)\n",
								__func__);
			fifo_purge(&dev->rx);
			continue;
		}

		fifo_read(&dev->rx, &hdr, sizeof(struct pdp_header));
		dev->rx_bytes += sizeof(struct pdp_header);

		/* Releases the semaphore and sleeps if no data available */
		ret = dpram_wait_for_fifo(dpr, dev, &dev->rx, 0,
				hdr.len - sizeof(struct pdp_header) + 1);
		if (ret == -ENODEV) {
			dev_warn(dpr->dev, "%s: Phone offline.\n", __func__);
			goto wait_for_phone;
		}
		if (ret)
			return ret;

		/* The semaphore is held here */
		dpram_pdp_demux_packet(dev, &hdr);
		dev->rx_bytes += hdr.len - sizeof(struct pdp_header);

		fifo_read(&dev->rx, &stop, 1);
		++dev->rx_bytes;
		if (stop != PDP_STOP_BYTE)
			dev_dbg(dpr->dev, "%s: Wrong stop byte read, corrupted packet?\n",
								__func__);

		local_irq_save(flags);
		if (dev->ack_req)
			bits = dev->ack_bits;
		else
			bits = 0;
		dev->ack_req = 0;
		local_irq_restore(flags);
	} while(!kthread_should_stop());

	onedram_semaphore_put(dpr, bits);

	return 0;
}

/*
 * PDP virtual TTY devices
 */
static int dpram_vtty_open(struct tty_struct *tty, struct file *filp)
{
	struct dpram_vtty_priv *vtty = tty->driver->driver_state;
	int ret;

	ret = mutex_lock_interruptible(&vtty->lock);
	if (ret)
		return ret;

	tty->driver_data = vtty;
	tty->low_latency = 1;

	++vtty->open_count;
	vtty->tty = tty;

	mutex_unlock(&vtty->lock);

	return 0;
}

static void dpram_vtty_close(struct tty_struct *tty, struct file *filp)
{
	struct dpram_vtty_priv *vtty = tty->driver_data;

	mutex_lock(&vtty->lock);

	if(!--vtty->open_count)
		vtty->tty = NULL;

	mutex_unlock(&vtty->lock);
}

static inline struct dpram *vtty_to_dpr(struct dpram_vtty_priv *vtty)
{
	struct dpram_device *dev = vtty->vdev.dev;
	return dev_to_dpr(dev);
}

static int dpram_vtty_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct dpram_vtty_priv *vtty = tty->driver_data;
	struct circ_buf *circ = &vtty->tx_buf;
	unsigned long flags;
	int ret = 0;

	if (!circ->buf)
		return 0;

	local_irq_save(flags);
	if (CIRC_SPACE(circ->head, circ->tail, VTTY_TX_BUF_LEN) != 0) {
		circ->buf[circ->head] = ch;
		circ->head = (circ->head + 1) & (VTTY_TX_BUF_LEN - 1);
		ret = 1;
	}
	local_irq_restore(flags);
	return ret;
}

static void dpram_vtty_flush_chars(struct tty_struct *tty)
{
	struct dpram_vtty_priv *vtty = tty->driver_data;
	struct dpram *dpr = vtty_to_dpr(vtty);
	unsigned long flags;
	int c;

	local_irq_save(flags);
	c = CIRC_CNT(vtty->tx_buf.head, vtty->tx_buf.tail, VTTY_TX_BUF_LEN);
	if (c && list_empty(&vtty->list)) {
		list_add_tail(&vtty->list, &dpr->pdp_vtty_queue);
		wake_up(&dpr->pdp_tx_wq);
	}
	local_irq_restore(flags);
}

static int dpram_vtty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	struct dpram_vtty_priv *vtty = tty->driver_data;
	struct dpram *dpr;
	struct circ_buf *circ;
	unsigned long flags;
	int c, ret = 0;
	int head, tail;

	/*
	 * This means you called this function _after_ the port was
	 * closed.  No cookie for you.
	 */
	if (!vtty) {
		WARN_ON(1);
		return -EL3HLT;
	}

	dpr = vtty_to_dpr(vtty);
	circ = &vtty->tx_buf;
	if (!circ->buf)
		return 0;

	local_irq_save(flags);

	while (1) {
		head = circ->head;
		tail = circ->tail;

		local_irq_restore(flags);

		c = CIRC_SPACE_TO_END(head, tail, VTTY_TX_BUF_LEN);
		if (count < c)
			c = count;
		if (c <= 0) {
			local_irq_save(flags);
			break;
		}

		memcpy(circ->buf + head, buf, c);
		head = (circ->head + c) & (VTTY_TX_BUF_LEN - 1);
		buf += c;
		count -= c;
		ret += c;

		local_irq_save(flags);

		circ->head = head;
	}

	if (list_empty(&vtty->list)) {
		list_add_tail(&vtty->list, &dpr->pdp_vtty_queue);
		wake_up(&dpr->pdp_tx_wq);
	}

	local_irq_restore(flags);

	return ret;
}

static int dpram_vtty_write_room(struct tty_struct *tty)
{
	struct dpram_vtty_priv *vtty = tty->driver_data;
	unsigned long flags;
	int ret;

	local_irq_save(flags);
	ret = CIRC_SPACE(vtty->tx_buf.head, vtty->tx_buf.tail, VTTY_TX_BUF_LEN);
	local_irq_restore(flags);

	return ret;
}

static int dpram_vtty_chars_in_buffer(struct tty_struct *tty)
{
	struct dpram_vtty_priv *vtty = tty->driver_data;
	unsigned long flags;
	int ret;

	local_irq_save(flags);
	ret = CIRC_CNT(vtty->tx_buf.head, vtty->tx_buf.tail, VTTY_TX_BUF_LEN);
	local_irq_restore(flags);

	return ret;
}

static struct tty_operations dpram_vtty_ops = {
	.open 			= dpram_vtty_open,
	.close 			= dpram_vtty_close,
	.write 			= dpram_vtty_write,
	.write_room		= dpram_vtty_write_room,
	.put_char		= dpram_vtty_put_char,
	.flush_chars		= dpram_vtty_flush_chars,
	.chars_in_buffer	= dpram_vtty_chars_in_buffer,
};

static void dpram_vtty_handle_rx(struct dpram_pdp_vdev *vdev, size_t len)
{
	struct dpram_device *dev = vdev->dev;
	struct dpram *dpr = dev_to_dpr(dev);
	struct dpram_vtty_priv *vtty =
			container_of(vdev, struct dpram_vtty_priv, vdev);

	mutex_lock(&vtty->lock);

	if (unlikely(!vtty->tty)) {
		dev_warn(dpr->dev, "%s: Skipping RX to closed vtty %s (%d bytes)\n",
						__func__, vtty->drv->name, len);
		fifo_skip(&dev->rx, len);
	} else {
		fifo_read_tty(&dev->rx, vtty->tty, len);
		tty_flip_buffer_push(vtty->tty);
	}

	mutex_unlock(&vtty->lock);
}

static void dpram_vtty_get_name(struct dpram_pdp_vdev *vdev,
							char *buf, size_t len)
{
	struct tty_driver *tty = vdev->priv;

	snprintf(buf, len, "%s0", tty->name);
}

static struct dpram_pdp_vdev *dpram_vtty_add_dev(struct dpram_device *dev,
					unsigned int id, const char *name)
{
	struct tty_driver *tty;
	struct dpram_vtty_priv *vtty;
	struct dpram_pdp_vdev *vdev;
	int ret;

	if (id > PDP_ID_MAX)
		return ERR_PTR(-EINVAL);

	vtty = kzalloc(sizeof(struct dpram_vtty_priv), GFP_KERNEL);
	if (!vtty)
		return ERR_PTR(-ENOMEM);

	vtty->tx_buf.buf = (void *)get_zeroed_page(GFP_KERNEL);
	if (!vtty->tx_buf.buf) {
		ret = -ENOMEM;
		goto err_tx_buf;
	}

	tty = alloc_tty_driver(1);
	if (!tty) {
		ret = -ENOMEM;
		goto err_alloc_tty;
	}

	tty->owner		= THIS_MODULE;
	tty->driver_name	= "multipdp";
	tty->name		= name;
	tty->type		= TTY_DRIVER_TYPE_SERIAL;
	tty->subtype		= SERIAL_TYPE_NORMAL;
	tty->init_termios	= tty_std_termios;
	tty->flags		= TTY_DRIVER_REAL_RAW;
	tty->driver_state	= vtty;
	tty_set_operations(tty, &dpram_vtty_ops);

	vtty->drv = tty;
	INIT_LIST_HEAD(&vtty->list);
	mutex_init(&vtty->lock);

	vdev = &vtty->vdev;
	vdev->type	= DPRAM_VTTY;
	vdev->id 	= id;
	vdev->handle_rx	= dpram_vtty_handle_rx;
	vdev->priv	= tty;
	vdev->dev	= dev;

	ret = tty_register_driver(tty);
	if (ret)
		goto err_tty_register;

	return vdev;

err_tty_register:
	put_tty_driver(tty);
err_alloc_tty:
	free_page((unsigned long)vtty->tx_buf.buf);
err_tx_buf:
	kfree(vtty);

	return ERR_PTR(ret);
}

static void dpram_vtty_rm_dev(struct dpram_pdp_vdev *vdev)
{
	struct dpram_vtty_priv *vtty =
			container_of(vdev, struct dpram_vtty_priv, vdev);

	tty_unregister_driver(vtty->drv);
	put_tty_driver(vtty->drv);
	free_page((unsigned long)vtty->tx_buf.buf);
	kfree(vtty);
}

/*
 * PDP virtual network devices
 */
static int dpram_vnet_open(struct net_device *net)
{
	netif_start_queue(net);
	return 0;
}

static int dpram_vnet_stop(struct net_device *net)
{
	netif_stop_queue(net);
	return 0;
}

static int dpram_vnet_start_xmit(struct sk_buff *skb, struct net_device *net)
{
	struct dpram_pdp_vdev *vdev = netdev_priv(net);
	struct dpram_device *dev = vdev->dev;
	struct dpram *dpr = dev_to_dpr(dev);

	skb_queue_tail(&dpr->pdp_vnet_queue, skb);
	wake_up(&dpr->pdp_tx_wq);

	return NETDEV_TX_OK;
}

static struct net_device_ops dpram_vnet_ops = {
	.ndo_open	= dpram_vnet_open,
	.ndo_stop	= dpram_vnet_stop,
	.ndo_start_xmit	= dpram_vnet_start_xmit,
};

static void dpram_vnet_setup(struct net_device *dev)
{
	dev->netdev_ops		= &dpram_vnet_ops;
	dev->type		= ARPHRD_PPP;
	dev->flags		= IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 1000;
	dev->mtu		= ETH_DATA_LEN;
	dev->watchdog_timeo	= 5 * HZ;
}

static int dpram_vnet_start_tx(struct dpram *dpr, struct dpram_device *dev)
{
	struct dpram_pdp_vdev *vdev;
	struct net_device *net;
	unsigned long flags;

	local_irq_save(flags);
	list_for_each_entry(vdev, &dpr->pdp_vdev_list, list) {
		if (vdev->type != DPRAM_VNET)
			continue;
		net = vdev->priv;
		netif_wake_queue(net);
	}
	local_irq_restore(flags);

	return 0;
}

static int dpram_vnet_stop_tx(struct dpram *dpr, struct dpram_device *dev)
{
	struct dpram_pdp_vdev *vdev;
	struct net_device *net;
	unsigned long flags;

	local_irq_save(flags);
	list_for_each_entry(vdev, &dpr->pdp_vdev_list, list) {
		if (vdev->type != DPRAM_VNET)
			continue;
		net = vdev->priv;
		netif_tx_disable(net);
	}
	local_irq_restore(flags);

	return 0;
}

static void dpram_vnet_handle_rx(struct dpram_pdp_vdev *vdev, size_t len)
{
	struct dpram_device *dev = vdev->dev;
	struct dpram *dpr = dev_to_dpr(dev);
	struct net_device *net = vdev->priv;
	struct sk_buff *skb;

	if (!netif_running(net)) {
		if (net_ratelimit())
			dev_warn(dpr->dev, "%s: Received packet while not running, dropping.\n",
								__func__);
		fifo_skip(&dev->rx, len);
		return;
	}

	skb = alloc_skb(len + 2, GFP_KERNEL);
	if (!skb) {
		if (net_ratelimit())
			dev_warn(dpr->dev, "%s: Failed to allocate socket buffer, dropping packet.\n",
								__func__);
		fifo_skip(&dev->rx, len);
		return;
	}

	fifo_read(&dev->rx, skb_put(skb, len), len);
	skb->dev = net;
	/* Get the ethertype from the version in the IP header. */
	if (skb->data[0] >> 4 == 6)
		skb->protocol = __constant_htons(ETH_P_IPV6);
	else
		skb->protocol = __constant_htons(ETH_P_IP);
	skb_reset_mac_header(skb);

	++net->stats.rx_packets;
	net->stats.rx_bytes += skb->len;

	netif_rx_ni(skb);
}

static void dpram_vnet_get_name(struct dpram_pdp_vdev *vdev,
							char *buf, size_t len)
{
	struct net_device *net = vdev->priv;

	snprintf(buf, len, "%s", net->name);
}

static struct dpram_pdp_vdev *dpram_vnet_add_dev(struct dpram_device *dev,
					unsigned int id, const char *name)
{
	struct net_device *net;
	struct dpram_pdp_vdev *vdev;
	int ret;

	if (id > PDP_ID_MAX)
		return ERR_PTR(-EINVAL);

	net = alloc_netdev(sizeof(struct dpram_pdp_vdev),
						"pdp%d", dpram_vnet_setup);
	if (!net)
		return ERR_PTR(-ENOMEM);

	vdev = netdev_priv(net);
	vdev->type	= DPRAM_VNET;
	vdev->id	= id;
	vdev->handle_rx	= dpram_vnet_handle_rx;
	vdev->priv	= net;
	vdev->dev	= dev;

	ret = register_netdev(net);
	if (ret) {
		free_netdev(net);
		return ERR_PTR(ret);
	}

	return vdev;
}

static void dpram_vnet_rm_dev(struct dpram_pdp_vdev *vdev)
{
	struct net_device *net = vdev->priv;

	unregister_netdev(net);
	free_netdev(net);
}

/*
 * PDP virtual device management
 */
static const struct dpram_vdev_template dpram_vdev_types[] = {
	[DPRAM_VTTY] = {
		.add_dev	= dpram_vtty_add_dev,
		.rm_dev		= dpram_vtty_rm_dev,
		.get_name	= dpram_vtty_get_name,
	},
	[DPRAM_VNET] = {
		.add_dev	= dpram_vnet_add_dev,
		.rm_dev		= dpram_vnet_rm_dev,
		.get_name	= dpram_vnet_get_name,
	},
};

static int dpram_pdp_activate(struct dpram *dpr, struct dpram_device *dev,
				struct pdp_arg *arg, enum dpram_vdev_type type)
{
	struct dpram_pdp_vdev *vdev;
	unsigned long flags;

	local_irq_save(flags);
	vdev = dpram_pdp_get_vdev(dpr, arg->id);
	local_irq_restore(flags);
	if (vdev) {
		if (vdev->type != type)
			return -EBUSY;

		dpram_vdev_types[type].get_name(vdev,
					arg->ifname, sizeof(arg->ifname));
		return 0;
	}

	if (type >= ARRAY_SIZE(dpram_vdev_types))
		return -EINVAL;

	vdev = dpram_vdev_types[type].add_dev(dev, arg->id, arg->ifname);
	if (IS_ERR(vdev)) {
		dev_err(dpr->dev, "%s: Failed to register pdp vdev type=%d, id=%d.\n",
						__func__, type, arg->id);
		return PTR_ERR(vdev);
	}

	local_irq_save(flags);
	list_add_tail(&vdev->list, &dpr->pdp_vdev_list);
	local_irq_restore(flags);

	dpram_vdev_types[type].get_name(vdev, arg->ifname, sizeof(arg->ifname));

	return 0;
}

static int dpram_pdp_deactivate(struct dpram *dpr,
				struct dpram_device *dev, struct pdp_arg *arg)
{
	struct dpram_pdp_vdev *vdev;
	unsigned long flags;

	local_irq_save(flags);
	vdev = dpram_pdp_get_vdev(dpr, arg->id);
	local_irq_restore(flags);
	if (!vdev || vdev->type == DPRAM_VTTY)
		return -EINVAL;

	local_irq_save(flags);
	list_del(&vdev->list);
	local_irq_restore(flags);

	dpram_vdev_types[vdev->type].rm_dev(vdev);

	return 0;
}

/*
 * PDP character device
 */
static int dpram_pdp_open(struct inode *inode, struct file *filp)
{
	struct dpram_device *device =
		container_of(filp->private_data, struct dpram_device, mdev);

	filp->private_data = device;

	return 0;
}

static long dpram_pdp_ioctl(struct file *filp,
					unsigned int cmd, unsigned long arg)
{
	struct dpram_device *dev = filp->private_data;
	struct dpram *dpr = dev_to_dpr(dev);
	int ret = 0;

	if((ret = mutex_lock_interruptible(&dpr->pdp_ctl_lock)) < 0)
		return ret;

	switch (cmd) {
	case IOCTL_PDP_ACTIVATE: {
		struct pdp_arg parg;
		ret = copy_from_user(&parg, (void __user *)arg,
							sizeof(struct pdp_arg));
		if (ret) {
			ret = -EFAULT;
			break;
		}
		parg.id += dpr->pdp_adjust;
		ret = dpram_pdp_activate(dpr, dev, &parg, DPRAM_VNET);
		if (ret)
			break;
		ret = copy_to_user((void __user *)arg, &parg,
							sizeof(struct pdp_arg));
		if (ret) {
			ret = -EFAULT;
			break;
		}
		break;
	}

	case IOCTL_PDP_DEACTIVATE: {
		struct pdp_arg parg;
		ret = copy_from_user(&parg, (void __user *)arg,
							sizeof(struct pdp_arg));
		if (ret) {
			ret = -EFAULT;
			break;
		}
		parg.id += dpr->pdp_adjust;
		ret = dpram_pdp_deactivate(dpr, dev, &parg);
		break;
	}

	case IOCTL_PDP_ADJUST:
		ret = copy_from_user(&dpr->pdp_adjust,
					(void __user *)arg, sizeof(int));
		if (ret)
			ret = -EFAULT;
		break;

	case IOCTL_PDP_TXSTART:
		ret = dpram_vnet_start_tx(dpr, dev);
		break;

	case IOCTL_PDP_TXSTOP:
		ret = dpram_vnet_stop_tx(dpr, dev);
		break;

	/* Dummies for compatibility */
	case IOCTL_PDP_CSDSTART:
	case IOCTL_PDP_CSDSTOP:
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&dpr->pdp_ctl_lock);

	return ret;
}

static struct file_operations dpram_pdp_fops = {
	.owner		= THIS_MODULE,
	.open		= dpram_pdp_open,
	.unlocked_ioctl	= dpram_pdp_ioctl,
};

static struct pdp_arg dpram_default_vtty[] = {
	{ .id = 1, .ifname = "ttyCSD" },
	{ .id = 8, .ifname = "ttyEFS" },
	{ .id = 5, .ifname = "ttyGPS" },
	{ .id = 6, .ifname = "ttyXTRA" },
	{ .id = 25, .ifname = "ttySMD" },
};

static int dpram_pdp_register(struct dpram_device *dev)
{
	struct dpram *dpr = dev_to_dpr(dev);
	int ret, i;

	dpr->pdp_adjust = 9;

	dev->mdev.name = "multipdp";
	dev->mdev.minor = MISC_DYNAMIC_MINOR;
	dev->mdev.fops = &dpram_pdp_fops;

	ret = misc_register(&dev->mdev);
	if (ret)
		return ret;

	dpr->pdp_tx_task = kthread_run(dpram_pdp_tx_thread, dev, "kpdptxd");
	if (IS_ERR(dpr->pdp_tx_task)) {
		dev_err(dpr->dev, "%s: Failed to create PDP TX thread.\n",
								__func__);
		ret = PTR_ERR(dpr->pdp_tx_task);
		goto err_tx;
	}

	dpr->pdp_rx_task = kthread_run(dpram_pdp_rx_thread, dev, "kpdprxd");
	if (IS_ERR(dpr->pdp_rx_task)) {
		dev_err(dpr->dev, "%s: Failed to create PDP RX thread.\n",
								__func__);
		ret = PTR_ERR(dpr->pdp_rx_task);
		goto err_rx;
	}

	for (i = 0; i < ARRAY_SIZE(dpram_default_vtty); ++i)
		dpram_pdp_activate(dpr, dev,
					&dpram_default_vtty[i], DPRAM_VTTY);

	return 0;

err_rx:
	kthread_stop(dpr->pdp_tx_task);
err_tx:
	misc_deregister(&dev->mdev);

	return 0;
}

static void dpram_pdp_unregister(struct dpram_device *dev)
{
	struct dpram *dpr = dev_to_dpr(dev);
	struct dpram_pdp_vdev *vdev;

	misc_deregister(&dev->mdev);

	send_sig(SIGTERM, dpr->pdp_rx_task, 1);
	kthread_stop(dpr->pdp_rx_task);

	send_sig(SIGTERM, dpr->pdp_tx_task, 1);
	kthread_stop(dpr->pdp_tx_task);

	list_for_each_entry(vdev, &dpr->pdp_vdev_list, list)
		dpram_vdev_types[vdev->type].rm_dev(vdev);
}

/*
 * Proc entry
 */
static int dpram_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	struct dpram *dpr = data;
	char *p = page;
	int len;
	u16 in_interrupt = 0, out_interrupt = 0;
	int fmt_rx_avail, fmt_tx_free, raw_rx_avail, raw_tx_free, sem;
	unsigned int fmt_rx, fmt_tx, raw_rx, raw_tx;
	char buf[SIZ_ERROR_HDR + SIZ_ERROR_MSG + 1];
	unsigned long flags;

	local_irq_save(flags);

	fmt_rx_avail = dpr->device[DPRAM_FMT].rx.avail;
	fmt_tx_free = dpr->device[DPRAM_FMT].tx.avail;
	fmt_rx = dpr->device[DPRAM_FMT].rx_bytes;
	fmt_tx = dpr->device[DPRAM_FMT].tx_bytes;
	raw_rx_avail = dpr->device[DPRAM_RAW].rx.avail;
	raw_tx_free = dpr->device[DPRAM_RAW].tx.avail;
	raw_rx = dpr->device[DPRAM_RAW].rx_bytes;
	raw_tx = dpr->device[DPRAM_RAW].tx_bytes;

	local_irq_restore(flags);

	sem = onedram_semaphore_held(dpr);

	in_interrupt = dpr->last_irq;
	out_interrupt = dpr->last_cmd;

	memset(buf, 0, sizeof(buf));
	local_irq_save(flags);
	memcpy(buf, dpr->err_buf, SIZ_ERROR_HDR + SIZ_ERROR_MSG);
	local_irq_restore(flags);

	p += sprintf(p,
			"-------------------------------------\n"
			"| NAME\t\t\t| VALUE\n"
			"-------------------------------------\n"
			"| Phone status     \t| %d\n"
			"| Onedram Semaphore\t| %d\n"
			"| sem request count\t| %d\n"
			"| FMT RX DATA      \t| %d\n"
			"| FMT TX SPACE     \t| %d\n"
			"| FMT RX bytes     \t| %d\n"
			"| FMT TX bytes     \t| %d\n"
			"| RAW RX DATA      \t| %d\n"
			"| RAW TX SPACE     \t| %d\n"
			"| RAW RX bytes     \t| %d\n"
			"| RAW TX bytes     \t| %d\n"
			"| PHONE->PDA MAILBOX\t| 0x%04x\n"
			"| PDA->PHONE MAILBOX\t| 0x%04x\n"
			"| LAST PHONE ERR MSG\t| %s\n"
			"| PHONE ACTIVE\t\t| %s\n"
			"| DPRAM IRQ active\t| %d\n"
			"-------------------------------------\n",
			dpr->status, sem,
			dpr->sem_req_count,
			fmt_rx_avail, fmt_tx_free, fmt_rx, fmt_tx,
			raw_rx_avail, raw_tx_free, raw_rx, raw_tx,
			in_interrupt, out_interrupt,
			(buf[0] != '\0' ? buf : "NONE"),
			(dpram_phone_getstatus(dpr) ? "ACTIVE" : "INACTIVE"),
			!gpio_get_value(dpr->pdata->gpio_onedram_int_n));

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

/*
 * SIM state attr
 */
static ssize_t sim_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dpram *dpr = dev_get_drvdata(dev);

	return sprintf(buf, "%s", (dpr->sim_state) ? "DETACHED" : "ATTACHED");
}

static ssize_t sim_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	/* do noting */
	return size;
}

static DEVICE_ATTR(sim, S_IRUGO | S_IWUGO, sim_show, sim_store);

/*
 * Driver initialization
 */
static int dpram_check_pdata(struct platform_device *pdev)
{
	struct dpram_platform_data *pdata = pdev->dev.platform_data;

	if (!gpio_is_valid(pdata->gpio_phone_on)) {
		dev_err(&pdev->dev, "Invalid PHONE_ON gpio specified.\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_phone_rst_n)) {
		dev_err(&pdev->dev, "Invalid PHONE_RST_N gpio specified.\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_phone_active)) {
		dev_err(&pdev->dev, "Invalid PHONE_ACTIVE gpio specified.\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_cp_boot_sel)) {
		dev_err(&pdev->dev, "Invalid CP_BOOT_SEL gpio specified.\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_usim_boot)) {
		dev_err(&pdev->dev, "Invalid USIM_BOOT gpio specified.\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_pda_active)) {
		dev_err(&pdev->dev, "Invalid PDA_ACTIVE gpio specified.\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_onedram_int_n)) {
		dev_err(&pdev->dev, "Invalid ONEDRAM_INT_N gpio specified.\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_sim_detect_n)) {
		dev_err(&pdev->dev, "Invalid SIM_DETECT_N gpio specified.\n");
		return -EINVAL;
	}

	return 0;
}

static int dpram_init_gpio(struct dpram *dpr)
{
	struct dpram_platform_data *pdata = dpr->pdata;
	int ret = 0;

	ret = gpio_request(pdata->gpio_phone_on, "Phone on");
	if (ret) {
		dev_err(dpr->dev, "Failed to request PHONE_ON gpio.\n");
		return ret;
	}
	gpio_direction_output(pdata->gpio_phone_on, 0);

	ret = gpio_request(pdata->gpio_cp_boot_sel, "CP boot sel");
	if (ret) {
		dev_err(dpr->dev, "Failed to request CP_BOOT_SEL gpio.\n");
		goto err_cp_boot_sel;
	}
	gpio_direction_output(pdata->gpio_cp_boot_sel, 0);

	ret = gpio_request(pdata->gpio_usim_boot, "USIM boot");
	if (ret) {
		dev_err(dpr->dev, "Failed to request USIM_BOOT gpio.\n");
		goto err_usim_boot;
	}
	gpio_direction_output(pdata->gpio_usim_boot, 0);

	ret = gpio_request(pdata->gpio_phone_rst_n, "Phone reset");
	if (ret) {
		dev_err(dpr->dev, "Failed to request PHONE_RST_N gpio.\n");
		goto err_phone_rst_n;
	}
	gpio_direction_output(pdata->gpio_phone_rst_n, 0);

	ret = gpio_request(pdata->gpio_pda_active, "PDA active");
	if (ret) {
		dev_err(dpr->dev, "Failed to request PDA_ACTIVE gpio.\n");
		goto err_pda_active;
	}
	gpio_direction_output(pdata->gpio_pda_active, 1);

	ret = gpio_request(pdata->gpio_phone_active, "Phone active");
	if (ret) {
		dev_err(dpr->dev, "Failed to request PHONE_ACTIVE gpio.\n");
		goto err_phone_active;
	}
	gpio_direction_input(pdata->gpio_phone_active);

	ret = gpio_request(pdata->gpio_onedram_int_n, "ONEDRAM IRQ");
	if (ret) {
		dev_err(dpr->dev, "Failed to request ONEDRAM_INT_N gpio.\n");
		goto err_onedram_int_n;
	}
	gpio_direction_input(pdata->gpio_onedram_int_n);

	ret = gpio_request(pdata->gpio_sim_detect_n, "SIM detect");
	if (ret) {
		dev_err(dpr->dev, "Failed to request SIM_DETECT_N gpio.\n");
		goto err_sim_detect_n;
	}
	gpio_direction_input(pdata->gpio_sim_detect_n);

	return 0;

err_sim_detect_n:
	gpio_free(pdata->gpio_onedram_int_n);
err_onedram_int_n:
	gpio_free(pdata->gpio_phone_active);
err_phone_active:
	gpio_free(pdata->gpio_pda_active);
err_pda_active:
	gpio_free(pdata->gpio_phone_rst_n);
err_phone_rst_n:
	gpio_free(pdata->gpio_usim_boot);
err_usim_boot:
	gpio_free(pdata->gpio_cp_boot_sel);
err_cp_boot_sel:
	gpio_free(pdata->gpio_phone_on);

	return ret;
}

static int dpram_init_driver_data(struct dpram *dpr)
{
	dpr->dgs_buf = kzalloc(SIZ_DGS_INFO, GFP_KERNEL);
	if (!dpr->dgs_buf) {
		dev_err(dpr->dev, "Failed to allocate DGS buffer.\n");
		return -ENOMEM;
	}

	mutex_init(&dpr->ctl_lock);
	mutex_init(&dpr->pdp_ctl_lock);

	init_waitqueue_head(&dpr->wq);
	init_waitqueue_head(&dpr->err_wq);
	init_waitqueue_head(&dpr->pdp_tx_wq);

	INIT_LIST_HEAD(&dpr->pdp_vdev_list);
	skb_queue_head_init(&dpr->pdp_vnet_queue);
	INIT_LIST_HEAD(&dpr->pdp_vtty_queue);

	wake_lock_init(&dpr->device[DPRAM_FMT].wakelock,
						WAKE_LOCK_SUSPEND, "dpram_fmt");
	dpr->device[DPRAM_FMT].idx = DPRAM_FMT;
	dpr->device[DPRAM_FMT].ack_bits = INT_MASK_RES_ACK_F;
	dpr->device[DPRAM_FMT].ack_req_bits = INT_MASK_REQ_ACK_F;
	dpr->device[DPRAM_FMT].send_bits = INT_MASK_SEND_F;

	wake_lock_init(&dpr->device[DPRAM_RAW].wakelock,
						WAKE_LOCK_SUSPEND, "dpram_raw");
	dpr->device[DPRAM_RAW].idx = DPRAM_RAW;
	dpr->device[DPRAM_RAW].ack_bits = INT_MASK_RES_ACK_R;
	dpr->device[DPRAM_RAW].ack_req_bits = INT_MASK_REQ_ACK_R;
	dpr->device[DPRAM_RAW].send_bits = INT_MASK_SEND_R;

	return 0;
}

static int dpram_map(struct dpram *dpr)
{
	dpr->res = request_mem_region(dpr->mem->start,
				resource_size(dpr->mem), dev_name(dpr->dev));
	if (!dpr->res) {
		dev_err(dpr->dev, "Failed to request memory region.\n");
		return -ENOMEM;
	}

	dpr->phys_base = dpr->res->start;
	dpr->size = resource_size(dpr->res);

	dpr->base = ioremap_nocache(dpr->phys_base, dpr->size);
	if (dpr->base == NULL) {
		dev_err(dpr->dev, "ioremap_nocache failed.\n");
		release_mem_region(dpr->phys_base, dpr->size);
		return -ENOMEM;
	}

	dpr->onedram_sem = dpr->base + DPRAM_SMP;
	dpr->onedram_mailbox[MAILBOX_BA] = dpr->base + DPRAM_MBX_BA;
	dpr->onedram_mailbox[MAILBOX_AB] = dpr->base + DPRAM_MBX_AB;

	INIT_M_FIFO(&dpr->device[DPRAM_FMT].rx, FMT, RX, dpr->base);
	INIT_M_FIFO(&dpr->device[DPRAM_FMT].tx, FMT, TX, dpr->base);
	INIT_M_FIFO(&dpr->device[DPRAM_RAW].rx, RAW, RX, dpr->base);
	INIT_M_FIFO(&dpr->device[DPRAM_RAW].tx, RAW, TX, dpr->base);

	return 0;
}

static int dpram_init_irq(struct dpram *dpr)
{
	int ret = 0;

	/* OneDRAM interrupt */
	ret = gpio_to_irq(dpr->pdata->gpio_onedram_int_n);
	if (ret < 0) {
		dev_err(dpr->dev, "Failed to get OneDRAM IRQ.\n");
		return ret;
	}
	dpr->onedram_irq = ret;

	ret = request_irq(dpr->onedram_irq, dpram_mailbox_irq,
				IRQF_TRIGGER_FALLING, "OneDRAM mailbox", dpr);
	if (ret) {
		dev_err(dpr->dev, "Failed to request OneDRAM IRQ.\n");
		return ret;
	}

	/* Phone active interrupt */
	ret = gpio_to_irq(dpr->pdata->gpio_phone_active);
	if (ret < 0) {
		dev_err(dpr->dev, "Failed to get phone active IRQ.\n");
		goto err_phone_active;
	}
	dpr->phone_active_irq = ret;

	ret = request_irq(dpr->phone_active_irq, dpram_phone_active_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"Phone active", dpr);
	if (ret) {
		dev_err(dpr->dev, "Failed to request phone active IRQ.\n");
		goto err_phone_active;
	}

	/* SIM detect interrupt */
	ret = gpio_to_irq(dpr->pdata->gpio_sim_detect_n);
	if (ret < 0) {
		dev_err(dpr->dev, "Failed to get SIM detect IRQ.\n");
		goto err_sim_detect;
	}
	dpr->sim_detect_irq = ret;

	ret = request_threaded_irq(dpr->sim_detect_irq, NULL, dpram_sim_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"SIM detect", dpr);
	if (ret) {
		dev_err(dpr->dev, "Failed to request SIM detect IRQ.\n");
		goto err_sim_detect;
	}

	enable_irq_wake(dpr->phone_active_irq);
	enable_irq_wake(dpr->onedram_irq);
	enable_irq_wake(dpr->sim_detect_irq);

	return 0;

err_sim_detect:
	free_irq(dpr->phone_active_irq, dpr);
err_phone_active:
	free_irq(dpr->onedram_irq, dpr);

	return ret;
}

/*
 * Driver cleanup
 */
static void dpram_clean_irq(struct dpram *dpr)
{
	disable_irq_wake(dpr->phone_active_irq);
	disable_irq_wake(dpr->onedram_irq);
	disable_irq_wake(dpr->sim_detect_irq);

	free_irq(dpr->onedram_irq, dpr);
	free_irq(dpr->phone_active_irq, dpr);
	free_irq(dpr->sim_detect_irq, dpr);
}

static void dpram_clean_gpio(struct dpram *dpr)
{
	struct dpram_platform_data *pdata = dpr->pdata;

	gpio_free(pdata->gpio_cp_boot_sel);
	gpio_free(pdata->gpio_onedram_int_n);
	gpio_free(pdata->gpio_pda_active);
	gpio_free(pdata->gpio_phone_active);
	gpio_free(pdata->gpio_phone_on);
	gpio_free(pdata->gpio_phone_rst_n);
	gpio_free(pdata->gpio_sim_detect_n);
	gpio_free(pdata->gpio_usim_boot);
}

static void dpram_clean_driver_data(struct dpram *dpr)
{
	wake_lock_destroy(&dpr->device[DPRAM_FMT].wakelock);
	wake_lock_destroy(&dpr->device[DPRAM_RAW].wakelock);

	kfree(dpr->dgs_buf);
}

static void dpram_unmap(struct dpram *dpr)
{
	iounmap(dpr->base);
	release_mem_region(dpr->phys_base, dpr->size);
}

/*
 * Platform driver
 */
static int __init dpram_probe(struct platform_device *pdev)
{
	struct dpram_platform_data *pdata = pdev->dev.platform_data;
	struct dpram *dpr;
	struct resource *mem;
	int ret;

	if (pdev->id != -1) {
		dev_err(&pdev->dev, "Only a single instance is allowed.\n");
		return -ENOENT;
	}

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data specified.\n");
		return -EINVAL;
	}

	if ((ret = dpram_check_pdata(pdev)) < 0)
		return ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "Could not get memory resource.\n");
		return -ENOMEM;
	}

	dpr = kzalloc(sizeof(struct dpram), GFP_KERNEL);
	if (!dpr) {
		dev_err(&pdev->dev, "Failed to allocate driver data.\n");
		return -ENOMEM;
	}

	dpr->dev = &pdev->dev;
	dpr->pdata = pdata;
	dpr->mem = mem;

	dpr->sec_dpram_dev = device_create(sec_class,
						dpr->dev, 0, dpr, "dpram");
	if (IS_ERR(dpr->sec_dpram_dev)) {
		dev_err(dpr->dev, "Failed to create sec dpram device.\n");
		ret = PTR_ERR(dpr->sec_dpram_dev);
		goto err_sec_dev;
	}

	if ((ret = device_create_file(dpr->sec_dpram_dev, &dev_attr_sim)) < 0) {
		dev_err(dpr->dev, "Failed to create sec dpram sim attr.\n");
		goto err_sec_dev_attr;
	}

	if ((ret = dpram_init_gpio(dpr)) != 0) {
		dev_err(dpr->dev, "Failed to setup GPIO pins.\n");
		goto err_gpio;
	}

	if ((ret = dpram_init_driver_data(dpr)) != 0) {
		dev_err(dpr->dev, "Failed to initialize driver data.\n");
		goto err_drvdata;
	}

	if ((ret = dpram_map(dpr)) != 0) {
		dev_err(dpr->dev, "Failed to map dpram memory.\n");
		goto err_map;
	}

	platform_set_drvdata(pdev, dpr);

	if ((ret = dpram_char_register(&dpr->device[DPRAM_FMT])) != 0) {
		dev_err(dpr->dev, "Failed to register dpram char device.\n");
		goto err_dpram_char_register;
	}

	if ((ret = dpram_pdp_register(&dpr->device[DPRAM_RAW])) != 0) {
		dev_err(dpr->dev, "Failed to register dpram pdp device.\n");
		goto err_dpram_pdp_register;
	}

	if ((ret = dpram_error_device_register(dpr)) != 0) {
		dev_err(dpr->dev, "Failed to register dpram error device.\n");
		goto err_dpram_err_register;
	}

	dpr->sim_state = gpio_get_value(pdata->gpio_sim_detect_n);
	dev_dbg(dpr->dev, "SIM state: %d\n", dpr->sim_state);

	if ((ret = dpram_init_irq(dpr)) < 0) {
		dev_err(dpr->dev, "Failed to setup dpram interrupts.\n");
		goto err_irq;
	}

	dpr->proc_entry = create_proc_read_entry(DRIVER_PROC_ENTRY,
						0, NULL, dpram_read_proc, dpr);

	return 0;

err_irq:
	dpram_error_device_unregister(dpr);
err_dpram_err_register:
	dpram_pdp_unregister(&dpr->device[DPRAM_RAW]);
err_dpram_pdp_register:
	dpram_char_unregister(&dpr->device[DPRAM_FMT]);
err_dpram_char_register:
	platform_set_drvdata(pdev, NULL);
	dpram_unmap(dpr);
err_map:
	dpram_clean_driver_data(dpr);
err_drvdata:
	dpram_clean_gpio(dpr);
err_gpio:
	device_remove_file(dpr->sec_dpram_dev, &dev_attr_sim);
err_sec_dev_attr:
	device_unregister(dpr->sec_dpram_dev);
err_sec_dev:
	kfree(dpr);
	return ret;
}

static int __devexit dpram_remove(struct platform_device *pdev)
{
	struct dpram *dpr = platform_get_drvdata(pdev);

	if (dpr->proc_entry)
		remove_proc_entry(DRIVER_PROC_ENTRY, 0);

	device_remove_file(dpr->sec_dpram_dev, &dev_attr_sim);
	device_unregister(dpr->sec_dpram_dev);
	dpram_error_device_unregister(dpr);
	dpram_char_unregister(&dpr->device[DPRAM_FMT]);
	dpram_pdp_unregister(&dpr->device[DPRAM_RAW]);
	dpram_clean_irq(dpr);
	dpram_clean_gpio(dpr);
	dpram_clean_driver_data(dpr);
	dpram_unmap(dpr);

	platform_set_drvdata(pdev, NULL);

	kfree(dpr);

	return 0;
}

static void dpram_shutdown(struct platform_device *pdev)
{
	struct dpram *dpr = platform_get_drvdata(pdev);

	gpio_set_value(dpr->pdata->gpio_phone_on, 0);
	gpio_set_value(dpr->pdata->gpio_phone_rst_n, 0);
}

static int dpram_suspend(struct device *dev)
{
	struct dpram *dpr = dev_get_drvdata(dev);

	gpio_set_value(dpr->pdata->gpio_pda_active, 0);

	return 0;
}

static int dpram_resume(struct device *dev)
{
	struct dpram *dpr = dev_get_drvdata(dev);

	gpio_set_value(dpr->pdata->gpio_pda_active, 1);

	return 0;
}

static struct dev_pm_ops dpram_pm_ops = {
	.suspend	= dpram_suspend,
	.resume		= dpram_resume,
};

static struct platform_driver dpram_driver = {
	.remove		= __devexit_p(dpram_remove),
	.shutdown	= dpram_shutdown,
	.driver	= {
		.name	= "spica-dpram",
		.pm	= &dpram_pm_ops,
	},
};

/*
 * Kernel module
 */
static int __init dpram_init(void)
{
	if (!strlen(fw_path))
		strncpy(fw_path, DEFAULT_FW_PATH, sizeof(fw_path));

	return platform_driver_probe(&dpram_driver, dpram_probe);
}

static void __exit dpram_exit(void)
{
	platform_driver_unregister(&dpram_driver);
}

module_init(dpram_init);
module_exit(dpram_exit);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("OneDRAM Phone Interface Device Driver");
MODULE_LICENSE("GPL");
