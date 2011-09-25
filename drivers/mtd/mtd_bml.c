/*
 * BML over MTD compatibility layer.
 *	Copyright © 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * Based on Direct MTD block device access (mtdblock):
 *	Copyright © 1999-2010 David Woodhouse <dwmw2@infradead.org>
 *	Copyright © 2000-2003 Nicolas Pitre <nico@fluxnic.net>
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/bitmap.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/sort.h>
#include <linux/bsearch.h>
#include <linux/suspend.h>

#include <linux/mtd/partitions.h>
#include <linux/mtd/bml.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>
#include <linux/mutex.h>

/*
 * Structures used by BML
 */

#define BML_SECT_SIZE		(512)
#define SECT_PER_BMS		(2)
#define SECT_PER_BMI		(4)

#define TYPE_INIT_CREATE	(0xFCFE)
#define TYPE_UPDATE_PIEXT	(0xFAFE)
#define TYPE_WRITE_ERR		(0xFDFE)
#define TYPE_ERASE_ERR		(0xFEFE)

#define BML_UPCB_HDR		"ULOCKPCH"

struct bml_pcb_header {
	char sig[8];
	u16 age;
	u16 alt_pcb;
	char erase_sig[8];
} __attribute__((packed));

struct bml_bms_entry {
	u16 sbn;
	u16 rbn;
} __attribute__((packed));

struct bml_bms_header {
	u16 inf;
	u16 age;
} __attribute__((packed));

#define BMS_ENTRY_COUNT	\
			((BML_SECT_SIZE - sizeof(struct bml_bms_header)) \
			/ sizeof(struct bml_bms_entry))

struct bml_bms_sector {
	struct bml_bms_header hdr;
	struct bml_bms_entry entries[BMS_ENTRY_COUNT];
} __attribute__((packed));

/*
 * Driver data
 */
struct bml_map_entry {
	size_t from;
	size_t to_offs;
};

struct bml_map_info {
	struct mtd_info		*mtd;
	u32			length;
	struct bml_map_entry	table[];
};

struct bml_dev {
	struct mtd_blktrans_dev mbd;
	int count;
	struct mutex cache_mutex;
	unsigned char *cache_data;
	unsigned long cache_offset;
	unsigned int cache_size;
	enum { STATE_EMPTY, STATE_CLEAN, STATE_DIRTY } cache_state;
	unsigned long *bad_bitmap;
	size_t offset;
};

static struct bml_map_info *bml_map_info;
static const struct bml_platform_data *bml_pdata;
static struct mutex bmls_lock;

/*
 * BML stuff...
 */

static inline bool bms_inf_valid(struct bml_bms_header *bms)
{
	switch (bms->inf) {
	case TYPE_INIT_CREATE:
	case TYPE_WRITE_ERR:
	case TYPE_ERASE_ERR:
	case TYPE_UPDATE_PIEXT:
		return true;
	default:
		return false;
	}
}

static ssize_t find_last_upcb(struct mtd_info *mtd)
{
	struct bml_pcb_header pch;
	size_t last_upcb;
	size_t retlen;
	size_t offset;
	size_t size;
	u16 max_age;
	int ret;

	retlen = 0;
	max_age = 0;
	for (offset = 0, size = mtd->size; size >= mtd->erasesize;
			size -= mtd->erasesize, offset += mtd->erasesize) {
		ret = mtd->block_isbad(mtd, offset);
		if (ret)
			continue;

		ret = mtd->read(mtd, offset, sizeof(pch),
						&retlen, (u_char *)&pch);
		if (ret || retlen != sizeof(pch))
			continue;

		if (strncmp(pch.sig, BML_UPCB_HDR, sizeof(pch.sig)))
			continue;

		if (pch.age <= max_age)
			continue;

		max_age = pch.age;
		last_upcb = offset;
	}

	if (max_age == 0)
		return -ENOENT;

	if (max_age == 0xffff)
		return -EINVAL;

	return last_upcb;
}

static int find_last_bms(struct mtd_info *mtd, size_t upcb_pos,
								size_t *bms_pos)
{
	struct bml_bms_header bms;
	size_t last_bms;
	size_t retlen;
	size_t offset;
	u16 max_age;
	u16 last_age;
	int ret;
	int count;

	count = 0;
	retlen = 0;
	max_age = 0;
	last_age = 0;
	for (offset = upcb_pos; offset < upcb_pos + mtd->erasesize;
					offset += SECT_PER_BMI*BML_SECT_SIZE) {
		ret = mtd->read(mtd, offset, sizeof(bms),
						&retlen, (u_char *)&bms);
		if (ret || retlen != sizeof(bms))
			continue;

		if (!bms_inf_valid(&bms))
			continue;

		ret = mtd->read(mtd, offset + SECT_PER_BMS*BML_SECT_SIZE,
				sizeof(bms), &retlen, (u_char *)&bms);
		if (ret || retlen != sizeof(bms))
			continue;

		if (!bms_inf_valid(&bms))
			continue;

		if (bms.age == last_age)
			++count;

		if (bms.age <= max_age)
			continue;

		max_age = bms.age;
		last_age = bms.age;
		last_bms = offset;
		count = 1;
	}

	if (max_age == 0)
		return -ENOENT;

	if (max_age == 0xffff)
		return -EINVAL;

	*bms_pos = last_bms;

	return count;
}

static int bml_map_cmp(const void *a, const void *b)
{
	const struct bml_map_entry *entry_a = a;
	const struct bml_map_entry *entry_b = b;

	return entry_a->from - entry_b->from;
}

static void bml_map_swap(void *a, void *b, int size)
{
	struct bml_map_entry *entry_a = a;
	struct bml_map_entry *entry_b = b;
	struct bml_map_entry tmp;

	tmp = *entry_a;
	*entry_a = *entry_b;
	*entry_b = tmp;
}

static struct bml_map_info *create_block_mapping(struct mtd_info *mtd,
						size_t bms_pos, int bms_count)
{
	struct bml_map_info *map;
	struct bml_bms_sector bms;
	struct bml_map_entry *entry;
	size_t retlen;
	size_t offset;
	u32 bmi_count;
	int count;
	int ret;
	int i;

	bmi_count = 0;
	count = bms_count;
	for (offset = bms_pos; count; --count,
					offset += SECT_PER_BMS*BML_SECT_SIZE) {
		ret = mtd->read(mtd, offset, sizeof(bms),
						&retlen, (u_char *)&bms);
		if (ret || retlen != sizeof(bms))
			return NULL;

		for (i = 0; i < ARRAY_SIZE(bms.entries); ++i) {
			if (bms.entries[i].rbn == 0xffff)
				goto scan_last;
			++bmi_count;
		}
	}

scan_last:
	map = vmalloc(sizeof(struct bml_map_info)
				+ bmi_count * sizeof(struct bml_map_entry));
	if (!map)
		return ERR_PTR(-ENOMEM);

	map->length = bmi_count;
	entry = map->table;
	count = bms_count;
	for (offset = bms_pos; count; --count,
					offset += SECT_PER_BMS*BML_SECT_SIZE) {
		ret = mtd->read(mtd, offset, sizeof(bms),
						&retlen, (u_char *)&bms);
		if (ret || retlen != sizeof(bms)) {
			vfree(map);
			return ERR_PTR(-EIO);
		}

		for (i = 0; i < ARRAY_SIZE(bms.entries); ++i) {
			if (bms.entries[i].rbn == 0xffff)
				goto build_last;
			if (mtd->erasesize_shift) {
				entry->from = bms.entries[i].sbn <<
							mtd->erasesize_shift;
				entry->to_offs = bms.entries[i].rbn <<
							mtd->erasesize_shift;
			} else {
				entry->from = bms.entries[i].sbn *
								mtd->erasesize;
				entry->to_offs = bms.entries[i].rbn *
								mtd->erasesize;
			}
			++entry;
		}
	}

build_last:
	sort(map->table, map->length, sizeof(struct bml_map_entry),
						bml_map_cmp, bml_map_swap);

	return map;
}

static int build_map_info(void)
{
	struct bml_map_entry *entry;
	struct mtd_info *mtd;
	ssize_t last_upcb;
	size_t last_bms;
	int bms_count;
	int ret;
	int i;

	mtd = get_mtd_device_nm("reservoir");
	if (IS_ERR(mtd)) {
		pr_err("mtd-bml: Could not find UPCA partition.\n");
		return PTR_ERR(mtd);
	}

	if (!mtd->lock || !mtd->unlock) {
		pr_err("mtd-bml: Block lock and unlock functionality is required.\n");
		put_mtd_device(mtd);
		return -EINVAL;
	}

	last_upcb = find_last_upcb(mtd);
	if (last_upcb < 0) {
		pr_err("mtd-bml: Could not find UPCB block.\n");
		ret = last_upcb;
		goto error;
	}

	bms_count = find_last_bms(mtd, last_upcb, &last_bms);
	if (bms_count <= 0) {
		pr_err("mtd-bml: Could not find BMS block.\n");
		ret = bms_count;
		goto error;
	}

	bml_map_info = create_block_mapping(mtd, last_bms, bms_count);
	if (IS_ERR(bml_map_info)) {
		pr_err("mtd-bml: Failed to create block mapping.\n");
		ret = PTR_ERR(bml_map_info);
		goto error;
	}

	entry = bml_map_info->table;
	for (i = 0; i < bml_map_info->length; ++i, ++entry) {
		u32 bad, res;
		if (mtd->erasesize_shift) {
			bad = entry->from >> mtd->erasesize_shift;
			res = entry->to_offs >> mtd->erasesize_shift;
		} else {
			bad = entry->from / mtd->erasesize_shift;
			res = entry->to_offs / mtd->erasesize_shift;
		}
		pr_info("mtd-bml: Found mapping of bad block %u to reserved block %u.\n",
								bad, res);
		mtd->unlock(mtd, entry->to_offs, mtd->erasesize);
	}

	bml_map_info->mtd = mtd;
	return 0;

error:
	put_mtd_device(mtd);
	return ret;
}

static ssize_t bml_lookup(size_t offset, size_t pos)
{
	struct bml_map_entry key = { .from = offset + pos };
	struct bml_map_entry *entry;

	entry = bsearch(&key, bml_map_info->table, bml_map_info->length,
				sizeof(struct bml_map_entry), bml_map_cmp);

	if (!entry)
		return -ENOENT;

	return entry->to_offs;
}

static int bml_read_block(struct bml_dev *bml, u32 block, size_t offset,
							size_t len, u_char *buf)
{
	struct mtd_info *mtd = bml->mbd.mtd;
	size_t retlen = 0;
	size_t pos;
	int ret;

	if (mtd->erasesize_shift)
		pos = block << mtd->erasesize_shift;
	else
		pos = block * mtd->erasesize;

	if (test_bit(block, bml->bad_bitmap)) {
		mtd = bml_map_info->mtd;
		pos = bml_lookup(bml->offset, pos);
		BUG_ON(pos < 0);
	}

	ret = mtd->read(mtd, pos + offset, len, &retlen, buf);
	if (ret)
		return ret;
	if (retlen != len)
		return -EIO;

	return 0;
}

static int bml_read(struct bml_dev *bml, size_t pos, size_t size, u_char *buf)
{
	struct mtd_info *mtd = bml->mbd.mtd;
	u32 block;
	size_t offset;
	ssize_t ssize = size;
	int ret;

	if (mtd->erasesize_shift)
		block = pos >> mtd->erasesize_shift;
	else
		block = pos / mtd->erasesize;

	if (mtd->erasesize_mask)
		offset = pos & mtd->erasesize_mask;
	else
		offset = pos % mtd->erasesize;

	if (offset) {
		size_t len = mtd->erasesize - offset;
		if (len > ssize)
			len = ssize;

		ret = bml_read_block(bml, block, offset, len, buf);
		if (ret)
			return ret;

		ssize -= len;
		pos += len;
		buf += len;
		++block;
	}

	while(ssize > 0) {
		size_t len = (ssize > mtd->erasesize) ? mtd->erasesize : ssize;

		ret = bml_read_block(bml, block, 0, len, buf);
		if (ret)
			return ret;

		ssize -= len;
		pos += len;
		buf += len;
		++block;
	}

	return 0;
}

static void erase_callback(struct erase_info *done)
{
	wait_queue_head_t *wait_q = (wait_queue_head_t *)done->priv;
	wake_up(wait_q);
}

static int bml_erase_write (struct bml_dev *bml, unsigned long pos,
			int len, const char *buf)
{
	struct mtd_info *mtd = bml->mbd.mtd;
	struct erase_info erase;
	DECLARE_WAITQUEUE(wait, current);
	wait_queue_head_t wait_q;
	size_t retlen;
	u32 block;
	int ret;

	if (mtd->erasesize_shift)
		block = pos >> mtd->erasesize_shift;
	else
		block = pos / mtd->erasesize;

	if (test_bit(block, bml->bad_bitmap)) {
		mtd = bml_map_info->mtd;
		pos = bml_lookup(bml->offset, pos);
		BUG_ON(pos < 0);
	}

	/*
	 * First, let's erase the flash block.
	 */

	init_waitqueue_head(&wait_q);
	erase.mtd = mtd;
	erase.callback = erase_callback;
	erase.addr = pos;
	erase.len = len;
	erase.priv = (u_long)&wait_q;

	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&wait_q, &wait);

	ret = mtd->erase(mtd, &erase);
	if (ret) {
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&wait_q, &wait);
		printk (KERN_WARNING "bml: erase of region [0x%lx, 0x%x] "
				     "on \"%s\" failed\n",
			pos, len, mtd->name);
		return ret;
	}

	schedule();  /* Wait for erase to finish. */
	remove_wait_queue(&wait_q, &wait);

	/*
	 * Next, write the data to flash.
	 */

	ret = mtd->write(mtd, pos, len, &retlen, buf);
	if (ret)
		return ret;
	if (retlen != len)
		return -EIO;
	return 0;
}

static int build_bad_bitmap(struct bml_dev *bml)
{
	struct mtd_info *mtd = bml->mbd.mtd;
	struct bml_map_entry *entry = bml_map_info->table;
	size_t offset;
	int block;
	int blocks;
	int i;

	if (mtd->erasesize_shift) {
		blocks = mtd->size >> mtd->erasesize_shift;
	} else {
		blocks = 0;
		for (offset = 0; offset < mtd->size; offset += mtd->erasesize)
			++blocks;
	}

	bml->bad_bitmap = vmalloc(BITS_TO_LONGS(blocks));
	if (!bml->bad_bitmap)
		return -ENOMEM;

	bitmap_zero(bml->bad_bitmap, blocks);

	for (i = 0; i < bml_map_info->length; ++i, ++entry) {
		if (entry->from < bml->offset)
			continue;

		if (entry->from >= bml->offset + mtd->size)
			break;

		offset = entry->from - bml->offset;

		if (mtd->erasesize_shift)
			block = offset >> mtd->erasesize_shift;
		else
			block = offset / mtd->erasesize;

		set_bit(block, bml->bad_bitmap);
	}

	return 0;
}

/*
 * Cache stuff...
 *
 * Since typical flash erasable sectors are much larger than what Linux's
 * buffer cache can handle, we must implement read-modify-write on flash
 * sectors for each block write requests.  To avoid over-erasing flash sectors
 * and to speed things up, we locally cache a whole flash sector while it is
 * being written to until a different sector is required.
 */

static int write_cached_data (struct bml_dev *bmldev)
{
	struct mtd_info *mtd = bmldev->mbd.mtd;
	int ret;

	if (bmldev->cache_state != STATE_DIRTY)
		return 0;

	DEBUG(MTD_DEBUG_LEVEL2, "bml: writing cached data for \"%s\" "
			"at 0x%lx, size 0x%x\n", mtd->name,
			bmldev->cache_offset, bmldev->cache_size);

	ret = bml_erase_write (bmldev, bmldev->cache_offset,
			   bmldev->cache_size, bmldev->cache_data);
	if (ret)
		return ret;

	/*
	 * Here we could arguably set the cache state to STATE_CLEAN.
	 * However this could lead to inconsistency since we will not
	 * be notified if this content is altered on the flash by other
	 * means.  Let's declare it empty and leave buffering tasks to
	 * the buffer cache instead.
	 */
	bmldev->cache_state = STATE_EMPTY;
	return 0;
}


static int do_cached_write (struct bml_dev *bmldev, unsigned long pos,
			    int len, const char *buf)
{
	struct mtd_info *mtd = bmldev->mbd.mtd;
	unsigned int sect_size = bmldev->cache_size;
	int ret;

	DEBUG(MTD_DEBUG_LEVEL2, "bml: write on \"%s\" at 0x%lx, size 0x%x\n",
		mtd->name, pos, len);

	while (len > 0) {
		unsigned long sect_start = (pos/sect_size)*sect_size;
		unsigned int offset = pos - sect_start;
		unsigned int size = sect_size - offset;
		if( size > len )
			size = len;

		if (size == sect_size) {
			/*
			 * We are covering a whole sector.  Thus there is no
			 * need to bother with the cache while it may still be
			 * useful for other partial writes.
			 */
			ret = bml_erase_write(bmldev, pos, size, buf);
			if (ret)
				return ret;
		} else {
			/* Partial sector: need to use the cache */

			if (bmldev->cache_state == STATE_DIRTY &&
			    bmldev->cache_offset != sect_start) {
				ret = write_cached_data(bmldev);
				if (ret)
					return ret;
			}

			if (bmldev->cache_state == STATE_EMPTY ||
			    bmldev->cache_offset != sect_start) {
				/* fill the cache with the current sector */
				bmldev->cache_state = STATE_EMPTY;
				ret = bml_read(bmldev, sect_start, sect_size,
							bmldev->cache_data);
				if (ret)
					return ret;

				bmldev->cache_offset = sect_start;
				bmldev->cache_size = sect_size;
				bmldev->cache_state = STATE_CLEAN;
			}

			/* write data to our local cache */
			memcpy (bmldev->cache_data + offset, buf, size);
			bmldev->cache_state = STATE_DIRTY;
		}

		buf += size;
		pos += size;
		len -= size;
	}

	return 0;
}


static int do_cached_read (struct bml_dev *bmldev, unsigned long pos,
			   int len, char *buf)
{
	struct mtd_info *mtd = bmldev->mbd.mtd;
	unsigned int sect_size = bmldev->cache_size;
	int ret;

	DEBUG(MTD_DEBUG_LEVEL2, "bml: read on \"%s\" at 0x%lx, size 0x%x\n",
			mtd->name, pos, len);

	while (len > 0) {
		unsigned long sect_start = (pos/sect_size)*sect_size;
		unsigned int offset = pos - sect_start;
		unsigned int size = sect_size - offset;
		if (size > len)
			size = len;

		/*
		 * Check if the requested data is already cached
		 * Read the requested amount of data from our internal cache if it
		 * contains what we want, otherwise we read the data directly
		 * from flash.
		 */
		if (bmldev->cache_state != STATE_EMPTY &&
		    bmldev->cache_offset == sect_start) {
			memcpy (buf, bmldev->cache_data + offset, size);
		} else {
			ret = bml_read(bmldev, pos, size, buf);
			if (ret)
				return ret;
		}

		buf += size;
		pos += size;
		len -= size;
	}

	return 0;
}

static int bml_readsect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct bml_dev *bmldev = container_of(dev, struct bml_dev, mbd);
	return do_cached_read(bmldev, block<<9, 512, buf);
}

static int bml_writesect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct bml_dev *bmldev = container_of(dev, struct bml_dev, mbd);
	if (unlikely(!bmldev->cache_data)) {
		bmldev->cache_data = vmalloc(bmldev->mbd.mtd->erasesize);
		if (!bmldev->cache_data)
			return -EINTR;
		/* -EINTR is not really correct, but it is the best match
		 * documented in man 2 write for all cases.  We could also
		 * return -EAGAIN sometimes, but why bother?
		 */
	}
	return do_cached_write(bmldev, block<<9, 512, buf);
}

static int bml_open(struct mtd_blktrans_dev *mbd)
{
	struct bml_dev *bmldev = container_of(mbd, struct bml_dev, mbd);
	int ret = 0;

	DEBUG(MTD_DEBUG_LEVEL1,"bml_open\n");

	mutex_lock(&bmls_lock);
	if (bmldev->count) {
		bmldev->count++;
		mutex_unlock(&bmls_lock);
		return 0;
	}

	/* OK, it's not open. Create cache info for it */
	bmldev->count = 1;
	mutex_init(&bmldev->cache_mutex);
	bmldev->cache_state = STATE_EMPTY;
	bmldev->cache_size = mbd->mtd->erasesize;
	bmldev->cache_data = NULL;
	if (unlikely(!bml_map_info))
		if((ret = build_map_info()) != 0)
			goto error;
	if (unlikely(!bmldev->bad_bitmap))
		ret = build_bad_bitmap(bmldev);

error:
	mutex_unlock(&bmls_lock);

	DEBUG(MTD_DEBUG_LEVEL1, "ok\n");

	return ret;
}

static int bml_release(struct mtd_blktrans_dev *mbd)
{
	struct bml_dev *bmldev = container_of(mbd, struct bml_dev, mbd);

	DEBUG(MTD_DEBUG_LEVEL1, "bml_release\n");

	mutex_lock(&bmls_lock);

	mutex_lock(&bmldev->cache_mutex);
	write_cached_data(bmldev);
	mutex_unlock(&bmldev->cache_mutex);

	if (!--bmldev->count) {
		/* It was the last usage. Free the cache */
		if (mbd->mtd->sync)
			mbd->mtd->sync(mbd->mtd);
		vfree(bmldev->cache_data);
	}

	mutex_unlock(&bmls_lock);

	DEBUG(MTD_DEBUG_LEVEL1, "ok\n");

	return 0;
}

static int bml_flush(struct mtd_blktrans_dev *dev)
{
	struct bml_dev *bmldev = container_of(dev, struct bml_dev, mbd);

	mutex_lock(&bmldev->cache_mutex);
	write_cached_data(bmldev);
	mutex_unlock(&bmldev->cache_mutex);

	if (dev->mtd->sync)
		dev->mtd->sync(dev->mtd);
	return 0;
}

static void bml_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct bml_dev *dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	int i;

	if (!dev)
		return;

	for (i = 0; i < bml_pdata->nr_parts; ++i)
		if (!strcmp(mtd->name, bml_pdata->parts[i].name))
			break;

	if (i == bml_pdata->nr_parts)
		return;

	if (!mtd->erasesize) {
		pr_err("mtd-bml: Unspecified eraseblock size for '%s'.\n",
								mtd->name);
		return;
	}

	if (mtd->flags & MTD_NO_ERASE) {
		pr_err("mtd-bml: Unsupported flag MTD_NO_ERASE for '%s'.\n",
								mtd->name);
		return;
	}

	dev->offset = bml_pdata->parts[i].offset;

	dev->mbd.mtd = mtd;
	dev->mbd.devnum = mtd->index;

	dev->mbd.size = mtd->size >> 9;
	dev->mbd.tr = tr;

	if (!(mtd->flags & MTD_WRITEABLE))
		dev->mbd.readonly = 1;

	if (add_mtd_blktrans_dev(&dev->mbd))
		kfree(dev);
}

static void bml_remove_dev(struct mtd_blktrans_dev *dev)
{
	struct bml_dev *bmldev = container_of(dev, struct bml_dev, mbd);

	del_mtd_blktrans_dev(dev);

	vfree(bmldev->bad_bitmap);
	kfree(bmldev);
}

static struct mtd_blktrans_ops bml_tr = {
	.name		= "bml",
	.major		= 137,
	.part_bits	= 0,
	.blksize 	= 512,
	.open		= bml_open,
	.flush		= bml_flush,
	.release	= bml_release,
	.readsect	= bml_readsect,
	.writesect	= bml_writesect,
	.add_mtd	= bml_add_mtd,
	.remove_dev	= bml_remove_dev,
	.owner		= THIS_MODULE,
};

static int bml_pm_notification(struct notifier_block *nb,
					unsigned long mode, void *_unused)
{
	struct bml_map_entry *entry;
	struct mtd_info *mtd;
	int i;

	switch (mode) {
	case PM_POST_SUSPEND:
		if (!bml_map_info) {
			mtd = get_mtd_device_nm("reservoir");
			if (!mtd)
				return 0;
			mtd->lock(mtd, 0, mtd->size);
			put_mtd_device(mtd);
			return 0;
		}

		mtd = bml_map_info->mtd;
		entry = bml_map_info->table;
		for (i = 0; i < bml_map_info->length; ++i, ++entry)
			mtd->unlock(mtd, entry->to_offs, mtd->erasesize);
		break;
	}

	return 0;
}

static struct notifier_block bml_pm_notifier = {
	.notifier_call	= bml_pm_notification,
};

static int __devinit bml_probe(struct platform_device *pdev)
{
	int ret;

	if (pdev->id != -1) {
		dev_err(&pdev->dev, "Only a single instance is allowed.\n");
		return -ENODEV;
	}

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "No platform data provided.\n");
		return -EINVAL;
	}

	bml_pdata = pdev->dev.platform_data;
	mutex_init(&bmls_lock);

	register_pm_notifier(&bml_pm_notifier);

	if ((ret = register_mtd_blktrans(&bml_tr)) != 0) {
		dev_err(&pdev->dev, "Failed to register mtd blktrans.\n");
		return ret;
	}

	return 0;
}

static int __devexit bml_remove(struct platform_device *pdev)
{
	deregister_mtd_blktrans(&bml_tr);

	unregister_pm_notifier(&bml_pm_notifier);

	if (bml_map_info) {
		put_mtd_device(bml_map_info->mtd);
		vfree(bml_map_info);
		bml_map_info = 0;
	}

	return 0;
}

static struct platform_driver bml_driver = {
	.probe	= bml_probe,
	.remove	= __devexit_p(bml_remove),
	.driver	= {
		.name = "mtd-bml",
	},
};

static int __init init_bml(void)
{
	return platform_driver_register(&bml_driver);
}

static void __exit cleanup_bml(void)
{
	platform_driver_unregister(&bml_driver);
}

module_init(init_bml);
module_exit(cleanup_bml);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("BML over MTD compatibility layer.");
