/*
 * Samsung S3C6410 OneNAND driver
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * Based on Samsung S3C64XX/S5PC1XX OneNAND driver
 * Copyright © 2008-2010 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *  Marek Szyprowski <m.szyprowski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/onenand.h>
#include <linux/mtd/partitions.h>
#include <linux/interrupt.h>

#include <asm/mach/flash.h>
#include <plat/regs-onenand.h>

#include <linux/io.h>

/*
 * OneNAND Map 10 commands
 */
#define ONENAND_ERASE_STATUS		0x00
#define ONENAND_MULTI_ERASE_SET		0x01
#define ONENAND_ERASE_START		0x03
#define ONENAND_UNLOCK_START		0x08
#define ONENAND_UNLOCK_END		0x09
#define ONENAND_LOCK_START		0x0A
#define ONENAND_LOCK_END		0x0B
#define ONENAND_LOCK_TIGHT_START	0x0C
#define ONENAND_LOCK_TIGHT_END		0x0D
#define ONENAND_UNLOCK_ALL		0x0E
#define ONENAND_OTP_ACCESS		0x12
#define ONENAND_SPARE_ACCESS_ONLY	0x13
#define ONENAND_MAIN_ACCESS_ONLY	0x14
#define ONENAND_ERASE_VERIFY		0x15
#define ONENAND_MAIN_SPARE_ACCESS	0x16
#define ONENAND_PIPELINE_READ		0x4000

/*
 * OneNAND command maps
 */
#define MAP_00				(0x0)
#define MAP_01				(0x1)
#define MAP_10				(0x2)
#define MAP_11				(0x3)

/*
 * Address bits
 */
#define S3C6410_CMD_MAP_SHIFT		24
#define S3C6410_FBA_SHIFT		12
#define S3C6410_FPA_SHIFT		6
#define S3C6410_FSA_SHIFT		4

/*
 * ONENAND Registers of S3C6410
 */
#define S3C_MEM_CFG		(0x00)	/* Memory Device Configuration Register */
#define S3C_BURST_LEN		(0x10)	/* Burst Length Register */
#define S3C_MEM_RESET		(0x20)	/* Memory Reset Register */
#define S3C_INT_ERR_STAT	(0x30)	/* Interrupt Error Status Register */
#define S3C_INT_ERR_MASK	(0x40)	/* Interrupt Error Mask Register */
#define S3C_INT_ERR_ACK		(0x50)	/* Interrupt Error Acknowledge Register */
#define S3C_ECC_ERR_STAT	(0x60)	/* ECC Error Status Register */
#define S3C_MANUFACT_ID		(0x70)	/* Manufacturer ID Register */
#define S3C_DEVICE_ID		(0x80)	/* Device ID Register */
#define S3C_DATA_BUF_SIZE	(0x90)	/* Data Buffer Size Register */
#define S3C_BOOT_BUF_SIZE	(0xA0)	/* Boot Buffer Size Register */
#define S3C_BUF_AMOUNT		(0xB0)	/* Amount of Buffer Register */
#define S3C_TECH		(0xC0)	/* Technology Register */
#define S3C_FBA_WIDTH		(0xD0)	/* FBA Width Register */
#define S3C_FPA_WIDTH		(0xE0)	/* FPA Width Register */
#define S3C_FSA_WIDTH		(0xF0)	/* FSA Width Register */
#define S3C_REVISION		(0x100)	/* Revision Register */
#define S3C_DATARAM0		(0x110)	/* DataRAMCode Register */
#define S3C_DATARAM1		(0x120)	/* DataRAM1 Code Register */
#define S3C_SYNC_MODE		(0x130)	/* Synchronous Mode Register */
#define S3C_TRANS_SPARE		(0x140)	/* Transfer Size Register */
#define S3C_DBS_DFS_WIDTH	(0x160)	/* DBS_DFS width Register */
#define S3C_PAGE_CNT		(0x170)	/* Page Count Register */
#define S3C_ERR_PAGE_ADDR	(0x180)	/* Error Page Address Register */
#define S3C_BURST_RD_LAT	(0x190)	/* Burst Read Latency Register */
#define S3C_INT_PIN_ENABLE	(0x1A0)	/* Interrupt Pin Enable Register */
#define S3C_INT_MON_CYC		(0x1B0)	/* Interrupt Monitor Cycle Count Register */
#define S3C_ACC_CLOCK		(0x1C0)	/* Access Clock Register */
#define S3C_SLOW_RD_PATH	(0x1D0)	/* Slow Read Path Register */
#define S3C_ERR_BLK_ADDR	(0x1E0)	/* Error Block Address Register */
#define S3C_FLASH_VER_ID	(0x1F0)	/* Flash Version ID Register */
#define S3C_FLASH_AUX_CNTRL	(0x300)	/* Flash Auxiliary control register */
#define S3C_FLASH_AFIFO_CNT	(0x310)	/* Number of data in asynchronous FIFO in flash controller 0. */

#define S3C_MEM_CFG_ECC		(1 << 8)

/*
 * Driver data
 */
struct s3c6410_onenand {
	struct mtd_info		*mtd;
	struct platform_device	*pdev;
	void __iomem		*base;
	struct resource 	*base_res;
	void __iomem		*ahb_addr;
	dma_addr_t		ahb_phys;
	struct resource 	*ahb_res;
	int			bootram_command;

	void		*page_buf;
	void		*oob_buf;

	unsigned int	(*mem_addr)(int fba, int fpa, int fsa);
	unsigned int	(*cmd_map)(unsigned int type, unsigned int val);

	struct mtd_partition	*parts;
};

#define CMD_MAP_00(dev, addr)		(dev->cmd_map(MAP_00, ((addr) << 1)))
#define CMD_MAP_01(dev, mem_addr)	(dev->cmd_map(MAP_01, (mem_addr)))
#define CMD_MAP_10(dev, mem_addr)	(dev->cmd_map(MAP_10, (mem_addr)))
#define CMD_MAP_11(dev, addr)		(dev->cmd_map(MAP_11, ((addr) << 2)))

/* FIXME: Use driver data instead of this global variable. */
static struct s3c6410_onenand *onenand;

static const char *part_probes[] = { "cmdlinepart", NULL, };

/*
 * I/O accessors
 */
static inline int s3c6410_onenand_read_reg(int offset)
{
	return readl(onenand->base + offset);
}

static inline void s3c6410_onenand_write_reg(int value, int offset)
{
	writel(value, onenand->base + offset);
}

static inline int s3c6410_onenand_read_cmd(unsigned int cmd)
{
	return readl(onenand->ahb_addr + cmd);
}

static inline void s3c6410_onenand_write_cmd(int value, unsigned int cmd)
{
	writel(value, onenand->ahb_addr + cmd);
}

static unsigned int s3c6410_onenand_cmd_map(unsigned type, unsigned val)
{
	return (type << S3C6410_CMD_MAP_SHIFT) | val;
}

static unsigned int s3c6410_onenand_mem_addr(int fba, int fpa, int fsa)
{
	return (fba << S3C6410_FBA_SHIFT) | (fpa << S3C6410_FPA_SHIFT) |
		(fsa << S3C6410_FSA_SHIFT);
}

/*
 * OneNAND driver
 */
static void s3c6410_onenand_reset(void)
{
	unsigned long timeout = 0x10000;
	int stat;

	s3c6410_onenand_write_reg(ONENAND_MEM_RESET_COLD, MEM_RESET_OFFSET);
	while (1 && timeout--) {
		stat = s3c6410_onenand_read_reg(INT_ERR_STAT_OFFSET);
		if (stat & RST_CMP)
			break;
	}
	stat = s3c6410_onenand_read_reg(INT_ERR_STAT_OFFSET);
	s3c6410_onenand_write_reg(stat, INT_ERR_ACK_OFFSET);

	/* Clear interrupt */
	s3c6410_onenand_write_reg(0x0, INT_ERR_ACK_OFFSET);
	/* Clear the ECC status */
	s3c6410_onenand_write_reg(0x0, ECC_ERR_STAT_OFFSET);
}

static unsigned short s3c6410_onenand_readw(void __iomem *addr)
{
	struct onenand_chip *this = onenand->mtd->priv;
	int reg = addr - this->base;
	int word_addr = reg >> 1;

	/* It's used for probing time */
	switch (reg) {
	case ONENAND_REG_MANUFACTURER_ID:
		return s3c6410_onenand_read_reg(MANUFACT_ID_OFFSET);
	case ONENAND_REG_DEVICE_ID:
		return s3c6410_onenand_read_reg(DEVICE_ID_OFFSET);
	case ONENAND_REG_VERSION_ID:
		return s3c6410_onenand_read_reg(FLASH_VER_ID_OFFSET);
	case ONENAND_REG_DATA_BUFFER_SIZE:
		return s3c6410_onenand_read_reg(DATA_BUF_SIZE_OFFSET);
	case ONENAND_REG_TECHNOLOGY:
		return s3c6410_onenand_read_reg(TECH_OFFSET);
	case ONENAND_REG_SYS_CFG1:
		return s3c6410_onenand_read_reg(MEM_CFG_OFFSET);

	/* Used at unlock all status */
	case ONENAND_REG_CTRL_STATUS:
		return 0;

	case ONENAND_REG_WP_STATUS:
		return ONENAND_WP_US;

	default:
		break;
	}

	/* BootRAM access control */
	if ((unsigned int) addr < ONENAND_DATARAM && onenand->bootram_command) {
		switch (word_addr) {
		case 0:
			return s3c6410_onenand_read_reg(MANUFACT_ID_OFFSET);
		case 1:
			return s3c6410_onenand_read_reg(DEVICE_ID_OFFSET);
		case 2:
			return s3c6410_onenand_read_reg(FLASH_VER_ID_OFFSET);
		default:
			break;
		}
	}

	return s3c6410_onenand_read_cmd(CMD_MAP_11(onenand, word_addr)) & 0xffff;
}

static void s3c6410_onenand_writew(unsigned short value, void __iomem *addr)
{
	struct onenand_chip *this = onenand->mtd->priv;
	unsigned int reg = addr - this->base;
	unsigned int word_addr = reg >> 1;

	/* It's used for probing time */
	switch (reg) {
	case ONENAND_REG_SYS_CFG1:
		s3c6410_onenand_write_reg(value, MEM_CFG_OFFSET);
		return;

	case ONENAND_REG_START_ADDRESS1:
	case ONENAND_REG_START_ADDRESS2:
		return;

	/* Lock/lock-tight/unlock/unlock_all */
	case ONENAND_REG_START_BLOCK_ADDRESS:
		return;

	default:
		break;
	}

	/* BootRAM access control */
	if ((unsigned int)addr < ONENAND_DATARAM) {
		switch (value) {
		case ONENAND_CMD_READID:
			onenand->bootram_command = 1;
			return;

		case ONENAND_CMD_RESET:
			s3c6410_onenand_write_reg(ONENAND_MEM_RESET_COLD,
							MEM_RESET_OFFSET);
			onenand->bootram_command = 0;
			return;

		default:
			break;
		}
	}

	s3c6410_onenand_write_cmd(value, CMD_MAP_11(onenand, word_addr));
}

#ifdef CONFIG_MTD_ONENAND_S3C6410_BURST_READ
static inline void s3c6410_onenand_read(struct s3c6410_onenand *onenand,
				unsigned int addr, int count, unsigned int *buf)
{
	__asm__ (
		"1:\n"
		"\tldmia %1, {r0-r7}\n"
		"\tstmia %2!, {r0-r7}\n"
		"\tsubs %0, #1\n"
		"\tbne 1b\n"
		:
		: "r"(count / 8),
			"r"(onenand->ahb_addr + addr), "r"(buf)
		: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"
	);
}
#else
static inline void s3c6410_onenand_read(struct s3c6410_onenand *onenand,
				unsigned int addr, int count, unsigned int *buf)
{
	int i;
	for (i = 0; i < count; i++)
		*buf++ = s3c6410_onenand_read_cmd(addr);
}
#endif

#ifdef CONFIG_MTD_ONENAND_S3C6410_BURST_WRITE
static inline void s3c6410_onenand_write(struct s3c6410_onenand *onenand,
				unsigned int addr, int count, unsigned int *buf)
{
	__asm__ (
		"1:\n"
		"\tldmia %1!, {r0-r7}\n"
		"\tstmia %2, {r0-r7}\n"
		"\tsubs %0, #1\n"
		"\tbne 1b\n"
		:
		: "r"(count / 8),
			"r"(buf), "r"(onenand->ahb_addr + addr)
		: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"
	);
}

static inline void s3c6410_onenand_dummy_write(
		struct s3c6410_onenand *onenand, unsigned int addr, int count)
{
	__asm__ (
		"\tmvn r0, #0\n"
		"\tmvn r1, #0\n"
		"\tmvn r2, #0\n"
		"\tmvn r3, #0\n"
		"\tmvn r4, #0\n"
		"\tmvn r5, #0\n"
		"\tmvn r6, #0\n"
		"\tmvn r7, #0\n"
		"1:\n"
		"\tstmia %1, {r0-r7}\n"
		"\tsubs %0, #1\n"
		"\tbne 1b\n"
		:
		: "r"(count / 8), "r"(onenand->ahb_addr + addr)
		: "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"
	);
}
#else
static inline void s3c6410_onenand_write(struct s3c6410_onenand *onenand,
				unsigned int addr, int count, unsigned int *buf)
{
	int i;
	for (i = 0; i < count; i++)
		s3c6410_onenand_write_cmd(*buf++, addr);
}

static inline void s3c6410_onenand_dummy_write(
		struct s3c6410_onenand *onenand, unsigned int addr, int count)
{
	int i;
	for (i = 0; i < count; i++)
		s3c6410_onenand_write_cmd(0xffffffff, addr);
}
#endif

static int s3c6410_onenand_command(struct mtd_info *mtd, int cmd, loff_t addr,
			       size_t len)
{
	struct onenand_chip *this = mtd->priv;
	unsigned int *m, *s;
	int fba, fpa, fsa = 0;
	unsigned int mem_addr, cmd_map_01, cmd_map_10;
	int mcount, scount;
	int index;

	fba = (int) (addr >> this->erase_shift);
	fpa = (int) (addr >> this->page_shift);
	fpa &= this->page_mask;

	mem_addr = onenand->mem_addr(fba, fpa, fsa);
	cmd_map_01 = CMD_MAP_01(onenand, mem_addr);
	cmd_map_10 = CMD_MAP_10(onenand, mem_addr);

	switch (cmd) {
	case ONENAND_CMD_READ:
	case ONENAND_CMD_READOOB:
	case ONENAND_CMD_BUFFERRAM:
		ONENAND_SET_NEXT_BUFFERRAM(this);
	default:
		break;
	}

	index = ONENAND_CURRENT_BUFFERRAM(this);

	/*
	 * Emulate Two BufferRAMs and access with 4 bytes pointer
	 */
	m = (unsigned int *) onenand->page_buf;
	s = (unsigned int *) onenand->oob_buf;

	if (index) {
		m += (this->writesize >> 2);
		s += (mtd->oobsize >> 2);
	}

	mcount = mtd->writesize >> 2;
	scount = mtd->oobsize >> 2;

	switch (cmd) {
	case ONENAND_CMD_READ:
		/* Main */
		s3c6410_onenand_read(onenand, cmd_map_01, mcount, m);
		return 0;

	case ONENAND_CMD_READOOB:
		s3c6410_onenand_write_reg(TSRF, TRANS_SPARE_OFFSET);

		/* Main */
		s3c6410_onenand_read(onenand, cmd_map_01, mcount, m);
		/* Spare */
		s3c6410_onenand_read(onenand, cmd_map_01, scount, s);

		s3c6410_onenand_write_reg(0, TRANS_SPARE_OFFSET);
		return 0;

	case ONENAND_CMD_PROG:
		/* Main */
		s3c6410_onenand_write(onenand, cmd_map_01, mcount, m);
		return 0;

	case ONENAND_CMD_PROGOOB:
		s3c6410_onenand_write_reg(TSRF, TRANS_SPARE_OFFSET);

		/* Main - dummy write */
		s3c6410_onenand_dummy_write(onenand, cmd_map_01, mcount);
		/* Spare */
		s3c6410_onenand_write(onenand, cmd_map_01, scount, s);

		s3c6410_onenand_write_reg(0, TRANS_SPARE_OFFSET);
		return 0;

	case ONENAND_CMD_UNLOCK_ALL:
		s3c6410_onenand_write_cmd(ONENAND_UNLOCK_ALL, cmd_map_10);
		return 0;

	case ONENAND_CMD_ERASE:
		s3c6410_onenand_write_cmd(ONENAND_ERASE_START, cmd_map_10);
		return 0;

	default:
		break;
	}

	return 0;
}

static int s3c6410_onenand_wait(struct mtd_info *mtd, int state)
{
	struct device *dev = &onenand->pdev->dev;
	unsigned int flags = INT_ACT;
	unsigned int stat, ecc;
	unsigned long timeout;

	switch (state) {
	case FL_READING:
		flags |= BLK_RW_CMP | LOAD_CMP;
		break;
	case FL_WRITING:
		flags |= BLK_RW_CMP | PGM_CMP;
		break;
	case FL_ERASING:
		flags |= BLK_RW_CMP | ERS_CMP;
		break;
	case FL_LOCKING:
		flags |= BLK_RW_CMP;
		break;
	default:
		break;
	}

	/* The 20 msec is enough */
	timeout = jiffies + msecs_to_jiffies(20);
	while (time_before(jiffies, timeout)) {
		stat = s3c6410_onenand_read_reg(INT_ERR_STAT_OFFSET);
		if (stat & flags)
			break;

		if (state != FL_READING)
			cond_resched();
	}

	/* To get correct interrupt status in timeout case */
	stat = s3c6410_onenand_read_reg(INT_ERR_STAT_OFFSET);
	s3c6410_onenand_write_reg(stat, INT_ERR_ACK_OFFSET);

	/*
	 * In the Spec. it checks the controller status first
	 * However if you get the correct information in case of
	 * power off recovery (POR) test, it should read ECC status first
	 */
	if (stat & LOAD_CMP) {
		ecc = s3c6410_onenand_read_reg(ECC_ERR_STAT_OFFSET);
		if (ecc & ONENAND_ECC_4BIT_UNCORRECTABLE) {
			dev_info(dev, "%s: ECC error = 0x%04x\n", __func__,
				 ecc);
			mtd->ecc_stats.failed++;
			return -EBADMSG;
		}
	}

	if (stat & (LOCKED_BLK | ERS_FAIL | PGM_FAIL | LD_FAIL_ECC_ERR)) {
		dev_info(dev, "%s: controller error = 0x%04x\n", __func__,
			 stat);
		if (stat & LOCKED_BLK)
			dev_info(dev, "%s: it's locked error = 0x%04x\n",
				 __func__, stat);

		return -EIO;
	}

	return 0;
}

static int s3c6410_onenand_bbt_wait(struct mtd_info *mtd, int state)
{
	unsigned int flags = INT_ACT | LOAD_CMP;
	unsigned int stat;
	unsigned long timeout;

	/* The 20 msec is enough */
	timeout = jiffies + msecs_to_jiffies(20);
	while (time_before(jiffies, timeout)) {
		stat = s3c6410_onenand_read_reg(INT_ERR_STAT_OFFSET);
		if (stat & flags)
			break;
	}
	/* To get correct interrupt status in timeout case */
	stat = s3c6410_onenand_read_reg(INT_ERR_STAT_OFFSET);
	s3c6410_onenand_write_reg(stat, INT_ERR_ACK_OFFSET);

	if (stat & LD_FAIL_ECC_ERR) {
		s3c6410_onenand_reset();
		return ONENAND_BBT_READ_ERROR;
	}

	if (stat & LOAD_CMP) {
		int ecc = s3c6410_onenand_read_reg(ECC_ERR_STAT_OFFSET);
		if (ecc & ONENAND_ECC_4BIT_UNCORRECTABLE) {
			s3c6410_onenand_reset();
			return ONENAND_BBT_READ_ERROR;
		}
	}

	return 0;
}

static unsigned char *s3c6410_get_bufferram(struct mtd_info *mtd, int area)
{
	struct onenand_chip *this = mtd->priv;
	int index = ONENAND_CURRENT_BUFFERRAM(this);
	unsigned char *p;

	if (area == ONENAND_DATARAM) {
		p = (unsigned char *) onenand->page_buf;
		if (index == 1)
			p += this->writesize;
	} else {
		p = (unsigned char *) onenand->oob_buf;
		if (index == 1)
			p += mtd->oobsize;
	}

	return p;
}

static int s3c6410_onenand_read_bufferram(struct mtd_info *mtd, int area,
				  unsigned char *buffer, int offset,
				  size_t count)
{
	unsigned char *p;

	p = s3c6410_get_bufferram(mtd, area);
	memcpy(buffer, p + offset, count);
	return 0;
}

static int s3c6410_onenand_write_bufferram(struct mtd_info *mtd, int area,
				   const unsigned char *buffer, int offset,
				   size_t count)
{
	unsigned char *p;

	p = s3c6410_get_bufferram(mtd, area);
	memcpy(p + offset, buffer, count);
	return 0;
}



static void s3c6410_onenand_check_lock_status(struct mtd_info *mtd)
{
	struct onenand_chip *this = mtd->priv;
	struct device *dev = &onenand->pdev->dev;
	unsigned int block, end;
	int stat;

	end = this->chipsize >> this->erase_shift;

	for (block = 0; block < end; block++) {
		s3c6410_onenand_write_cmd(block,
			CMD_MAP_11(onenand, ONENAND_REG_START_ADDRESS1 >> 1));

		stat = s3c6410_onenand_read_cmd(
			CMD_MAP_11(onenand, ONENAND_REG_WP_STATUS >> 1));
		if (!(stat & ONENAND_WP_US))
			dev_err(dev, "block %d is write-protected!\n", block);
	}
}

static void s3c6410_onenand_do_lock_cmd(struct mtd_info *mtd, loff_t ofs,
				    size_t len, int cmd)
{
	struct onenand_chip *this = mtd->priv;
	int start, end, start_mem_addr, end_mem_addr;

	start = ofs >> this->erase_shift;
	start_mem_addr = onenand->mem_addr(start, 0, 0);
	end = start + (len >> this->erase_shift) - 1;
	end_mem_addr = onenand->mem_addr(end, 0, 0);

	if (cmd == ONENAND_CMD_LOCK) {
		s3c6410_onenand_write_cmd(ONENAND_LOCK_START,
					CMD_MAP_10(onenand, start_mem_addr));
		s3c6410_onenand_write_cmd(ONENAND_LOCK_END,
					CMD_MAP_10(onenand, end_mem_addr));
	} else {
		s3c6410_onenand_write_cmd(ONENAND_UNLOCK_START,
					CMD_MAP_10(onenand, start_mem_addr));
		s3c6410_onenand_write_cmd(ONENAND_UNLOCK_END,
					CMD_MAP_10(onenand, end_mem_addr));
	}

	this->wait(mtd, FL_LOCKING);
}

static void s3c6410_onenand_unlock_all(struct mtd_info *mtd)
{
	struct onenand_chip *this = mtd->priv;
	loff_t ofs = 0;
	size_t len = this->chipsize;

	if (this->options & ONENAND_HAS_UNLOCK_ALL) {
		/* Write unlock command */
		this->command(mtd, ONENAND_CMD_UNLOCK_ALL, 0, 0);

		/* No need to check return value */
		this->wait(mtd, FL_LOCKING);

		/* Workaround for all block unlock in DDP */
		if (!ONENAND_IS_DDP(this)) {
			s3c6410_onenand_check_lock_status(mtd);
			return;
		}

		/* All blocks on another chip */
		ofs = this->chipsize >> 1;
		len = this->chipsize >> 1;
	}

	s3c6410_onenand_do_lock_cmd(mtd, ofs, len, ONENAND_CMD_UNLOCK);

	s3c6410_onenand_check_lock_status(mtd);
}

static void s3c6410_onenand_setup(struct mtd_info *mtd)
{
	struct onenand_chip *this = mtd->priv;

	onenand->mtd = mtd;

	onenand->mem_addr = s3c6410_onenand_mem_addr;
	onenand->cmd_map = s3c6410_onenand_cmd_map;

	this->read_word = s3c6410_onenand_readw;
	this->write_word = s3c6410_onenand_writew;

	this->wait = s3c6410_onenand_wait;
	this->bbt_wait = s3c6410_onenand_bbt_wait;
	this->unlock_all = s3c6410_onenand_unlock_all;
	this->command = s3c6410_onenand_command;

	this->read_bufferram = s3c6410_onenand_read_bufferram;
	this->write_bufferram = s3c6410_onenand_write_bufferram;
}

/*
 * Platform driver
 */
static int s3c6410_onenand_probe(struct platform_device *pdev)
{
	struct onenand_platform_data *pdata;
	struct onenand_chip *this;
	struct mtd_info *mtd;
	struct resource *r;
	int size, err;
	u32 reg;

	pdata = pdev->dev.platform_data;
	/* No need to check pdata. the platform data is optional */

	size = sizeof(struct mtd_info) + sizeof(struct onenand_chip);
	mtd = kzalloc(size, GFP_KERNEL);
	if (!mtd) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	onenand = kzalloc(sizeof(struct s3c6410_onenand), GFP_KERNEL);
	if (!onenand) {
		err = -ENOMEM;
		goto onenand_fail;
	}

	this = (struct onenand_chip *) &mtd[1];
	mtd->priv = this;
	mtd->dev.parent = &pdev->dev;
	mtd->owner = THIS_MODULE;
	onenand->pdev = pdev;

	s3c6410_onenand_setup(mtd);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		return -ENOENT;
		goto ahb_resource_failed;
	}

	onenand->base_res = request_mem_region(r->start, resource_size(r),
					       pdev->name);
	if (!onenand->base_res) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		err = -EBUSY;
		goto resource_failed;
	}

	onenand->base = ioremap(r->start, resource_size(r));
	if (!onenand->base) {
		dev_err(&pdev->dev, "failed to map memory resource\n");
		err = -EFAULT;
		goto ioremap_failed;
	}
	/* Set onenand_chip also */
	this->base = onenand->base;

	/* Use runtime badblock check */
	this->options |= ONENAND_SKIP_UNLOCK_CHECK;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!r) {
		dev_err(&pdev->dev, "no buffer memory resource defined\n");
		return -ENOENT;
		goto ahb_resource_failed;
	}

	onenand->ahb_res = request_mem_region(r->start, resource_size(r),
						pdev->name);
	if (!onenand->ahb_res) {
		dev_err(&pdev->dev, "failed to request buffer memory resource\n");
		err = -EBUSY;
		goto ahb_resource_failed;
	}

	onenand->ahb_phys = r->start;
	onenand->ahb_addr = ioremap(r->start, resource_size(r));
	if (!onenand->ahb_addr) {
		dev_err(&pdev->dev, "failed to map buffer memory resource\n");
		err = -EINVAL;
		goto ahb_ioremap_failed;
	}

	/* Allocate 4KiB BufferRAM */
	onenand->page_buf = kzalloc(SZ_4K, GFP_KERNEL);
	if (!onenand->page_buf) {
		err = -ENOMEM;
		goto page_buf_fail;
	}

	/* Allocate 128 SpareRAM */
	onenand->oob_buf = kzalloc(128, GFP_KERNEL);
	if (!onenand->oob_buf) {
		err = -ENOMEM;
		goto oob_buf_fail;
	}

	/* S3C doesn't handle subpage write */
	mtd->subpage_sft = 0;
	this->subpagesize = mtd->writesize;

	/* Enable ECC */
	reg = s3c6410_onenand_read_reg(S3C_MEM_CFG);
	reg &= ~S3C_MEM_CFG_ECC;
	s3c6410_onenand_write_reg(reg, S3C_MEM_CFG);

	/* Perform BBT scan */
	if (onenand_scan(mtd, 1)) {
		err = -EFAULT;
		goto scan_failed;
	}

	/* S3C doesn't handle subpage write */
	mtd->subpage_sft = 0;
	this->subpagesize = mtd->writesize;

	if (s3c6410_onenand_read_reg(MEM_CFG_OFFSET) & ONENAND_SYS_CFG1_SYNC_READ)
		dev_info(&onenand->pdev->dev, "OneNAND Sync. Burst Read enabled\n");

	err = parse_mtd_partitions(mtd, part_probes, &onenand->parts, 0);
	if (err > 0)
		mtd_device_register(mtd, onenand->parts, err);
	else if (err <= 0 && pdata && pdata->parts)
		mtd_device_register(mtd, pdata->parts, pdata->nr_parts);
	else
		err = mtd_device_register(mtd, NULL, 0);

	platform_set_drvdata(pdev, mtd);

	return 0;

scan_failed:
	kfree(onenand->oob_buf);
oob_buf_fail:
	kfree(onenand->page_buf);
page_buf_fail:
	iounmap(onenand->ahb_addr);
ahb_ioremap_failed:
	release_mem_region(onenand->ahb_res->start,
					resource_size(onenand->ahb_res));
ahb_resource_failed:
	iounmap(onenand->base);
ioremap_failed:
	release_mem_region(onenand->base_res->start,
				   resource_size(onenand->base_res));
resource_failed:
	kfree(onenand);
onenand_fail:
	kfree(mtd);
	return err;
}

static int __devexit s3c6410_onenand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);

	onenand_release(mtd);
	iounmap(onenand->ahb_addr);
	release_mem_region(onenand->ahb_res->start,
					resource_size(onenand->ahb_res));

	iounmap(onenand->base);
	release_mem_region(onenand->base_res->start,
			   resource_size(onenand->base_res));

	platform_set_drvdata(pdev, NULL);

	kfree(onenand->page_buf);
	kfree(onenand->oob_buf);

	kfree(onenand);
	kfree(mtd);

	return 0;
}

struct s3c6410_onenand_sleep_save {
	unsigned long	reg;
	unsigned long	val;
};

#define S3C6410_ONENAND_SAVE_ITEM(x) \
	{ .reg = (x) }

static struct s3c6410_onenand_sleep_save s3c6410_onenand_save_data[] = {
	S3C6410_ONENAND_SAVE_ITEM(S3C_MEM_CFG),
	S3C6410_ONENAND_SAVE_ITEM(S3C_BURST_LEN),
	S3C6410_ONENAND_SAVE_ITEM(S3C_INT_ERR_MASK),
	S3C6410_ONENAND_SAVE_ITEM(S3C_FBA_WIDTH),
	S3C6410_ONENAND_SAVE_ITEM(S3C_FPA_WIDTH),
	S3C6410_ONENAND_SAVE_ITEM(S3C_FSA_WIDTH),
	S3C6410_ONENAND_SAVE_ITEM(S3C_DATARAM0),
	S3C6410_ONENAND_SAVE_ITEM(S3C_DATARAM1),
	S3C6410_ONENAND_SAVE_ITEM(S3C_TRANS_SPARE),
	S3C6410_ONENAND_SAVE_ITEM(S3C_DBS_DFS_WIDTH),
	S3C6410_ONENAND_SAVE_ITEM(S3C_INT_PIN_ENABLE),
	S3C6410_ONENAND_SAVE_ITEM(S3C_INT_MON_CYC),
	S3C6410_ONENAND_SAVE_ITEM(S3C_ACC_CLOCK),
	S3C6410_ONENAND_SAVE_ITEM(S3C_SLOW_RD_PATH),
	S3C6410_ONENAND_SAVE_ITEM(S3C_FLASH_AUX_CNTRL),
};

static void s3c6410_onenand_save(struct s3c6410_onenand_sleep_save *ptr,
								int count)
{
	for (; count > 0; count--, ptr++)
		ptr->val = s3c6410_onenand_read_reg(ptr->reg);
}

static void s3c6410_onenand_restore(
			struct s3c6410_onenand_sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++)
		s3c6410_onenand_write_reg(ptr->val, ptr->reg);
}

static int s3c6410_onenand_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct onenand_chip *this = mtd->priv;

	this->wait(mtd, FL_PM_SUSPENDED);

	s3c6410_onenand_save(s3c6410_onenand_save_data,
					ARRAY_SIZE(s3c6410_onenand_save_data));

	return 0;
}

static  int s3c6410_onenand_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct onenand_chip *this = mtd->priv;

	s3c6410_onenand_restore(s3c6410_onenand_save_data,
					ARRAY_SIZE(s3c6410_onenand_save_data));

	this->unlock_all(mtd);

	return 0;
}

static const struct dev_pm_ops s3c6410_onenand_pm_ops = {
	.suspend	= s3c6410_onenand_suspend,
	.resume		= s3c6410_onenand_resume,
};

static struct platform_driver s3c6410_onenand_driver = {
	.driver         = {
		.name	= "s3c6410-onenand",
		.pm	= &s3c6410_onenand_pm_ops,
	},
	.probe          = s3c6410_onenand_probe,
	.remove         = __devexit_p(s3c6410_onenand_remove),
};

/*
 * Module init/exit
 */
static int __init s3c6410_onenand_init(void)
{
	return platform_driver_register(&s3c6410_onenand_driver);
}

static void __exit s3c6410_onenand_exit(void)
{
	platform_driver_unregister(&s3c6410_onenand_driver);
}

module_init(s3c6410_onenand_init);
module_exit(s3c6410_onenand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("S3C6410 OneNAND controller support");
