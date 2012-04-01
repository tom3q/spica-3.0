/*
 * OneDRAM Phone Interface Device Driver
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *	Complete rewrite.
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

#ifndef __DPRAM_H__
#define __DPRAM_H__

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

struct dpram_platform_data {
	unsigned int gpio_phone_on;
	unsigned int gpio_phone_rst_n;
	unsigned int gpio_phone_active;
	unsigned int gpio_cp_boot_sel;
	unsigned int gpio_usim_boot;
	unsigned int gpio_pda_active;
	unsigned int gpio_onedram_int_n;
	unsigned int gpio_sim_detect_n;
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
	unsigned sem_bp_request;
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

#endif	/* __DPRAM_H__ */
