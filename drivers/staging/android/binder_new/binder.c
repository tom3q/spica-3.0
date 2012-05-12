/*
 * Android Binder IPC driver
 * Copyright (c) 2012 Rong Shen <rong1129@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/compiler.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/rbtree.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>

#include <asm/atomic.h>

#include "msg_queue.h"
#include "fast_slob.h"
#include "binder.h"


#define OBJ_HASH_BUCKET_SIZE			128
#define MAX_TRACE_DEPTH				4
#define MSG_BUF_ALIGN(n)			(((n) & (sizeof(void *) - 1)) ? ALIGN((n), sizeof(void *)) : (n))
#define OBJ_IS_BINDER(o)			((o)->owner_queue)
#define OBJ_IS_HANDLE(o)			(!OBJ_IS_BINDER(o))
#define DUMP_MSG(pid, tid, wrt, msg)		//_dump_msg(pid, tid, wrt, msg)


enum {	// compat: review looper idea
	BINDER_LOOPER_STATE_INVALID     = 0x00,
	BINDER_LOOPER_STATE_REGISTERED  = 0x01,
	BINDER_LOOPER_STATE_ENTERED     = 0x02,
	BINDER_LOOPER_STATE_READY       = 0x03		// compat
};

typedef enum {
	BINDER_EVT_OBJ_DEAD = 1,
} binder_event_t;


struct binder_proc {
	spinlock_t lock;
	struct rb_root thread_tree;

	spinlock_t obj_lock;
	struct rb_root obj_tree;
	struct hlist_head obj_hash[OBJ_HASH_BUCKET_SIZE];
	unsigned long obj_seq;

	struct msg_queue *queue;

	struct fast_slob *slob;
	int slob_uses;
	unsigned long ustart;

	pid_t pid;

	atomic_t busy_threads, proc_loopers, requested_loopers, registered_loopers;
	int max_threads;

	spinlock_t reclaim_lock;
	struct list_head reclaim_list;	// deferred release list for objects

	struct dentry *proc_dir, *thread_dir, *obj_dir;
};

struct binder_thread {
	pid_t pid;

	struct rb_node rb_node;
	struct msg_queue *queue;

	int state;
	int non_block;

	int pending_replies;
	struct list_head incoming_transactions;

	struct binder_proc *proc;
	struct dentry *info_node;
};

struct binder_notifier {
	struct list_head list;
	int event;
	void *cookie;
	msg_queue_id to_notify;
};

struct binder_obj {
	msg_queue_id owner;
	struct msg_queue *owner_queue;

	void *binder;
	void *cookie;

	struct rb_node rb_node;

	unsigned long ref;
	struct hlist_node hash_node;

	atomic_t refs;

	spinlock_t lock;		// used for notifiers only
	struct list_head notifiers;

	struct dentry *info_node;
};

#define bcmd_transaction_data	binder_transaction_data

struct bcmd_notifier_data {
	long handle;
	void *cookie;
};

struct bcmd_ref_return {
	uint32_t cmd;
	void *binder;
	void *cookie;
};

struct bcmd_msg_buf {
	uint8_t *data;
	uint8_t *offsets;

	size_t data_size;
	size_t offsets_size;
	size_t buf_size;

	msg_queue_id owners[0];		// owners of objects in this buffer
};

struct bcmd_msg_trace {
	msg_queue_id caller_proc;
	msg_queue_id caller_thread;
};

struct bcmd_msg {
	struct list_head list;

	void *binder;
	void *cookie;

	unsigned int type;		// compat: review all data types in/out of the ioctl
	unsigned int code;
	unsigned int flags;

	struct bcmd_msg_buf *buf;

	pid_t sender_pid;
	uid_t sender_euid;

	msg_queue_id reply_to;

	int trace_depth;
	struct bcmd_msg_trace traces[MAX_TRACE_DEPTH];
};

struct slob_buf {
	unsigned long uaddr_data;
	unsigned long uaddr_offsets;
	size_t data_size;
	size_t offsets_size;
	char data[0];
};


static struct binder_obj *context_mgr_obj;	// compat: is context mgr necessary?
static uid_t context_mgr_uid = -1;

static struct dentry *debugfs_root;


static int debugfs_new_proc(struct binder_proc *proc);
static int debugfs_new_thread(struct binder_proc *proc, struct binder_thread *thread);
static int debugfs_new_obj(struct binder_proc *proc, struct binder_obj *obj);


// TODO: to be deleted
void _hexdump(const void *buf, unsigned long size)
{
	int col = 0, off = 0, n = 0;
	unsigned char *p = (unsigned char *)buf;
	char cbuf[64];

	if (size > 256)
		size = 256;
	while (size--) {
		if (!col)
			printk("\t%08x:", off);

		printk(" %02x", *p);
		cbuf[n++] = (*p >= 0x20 && *p < 0x7f) ? (char)*p : ' ';
		cbuf[n++] = ' ';

		p++;
		off++;
		col++;

		if (!(col % 16)) {
			cbuf[n] = '\0';
			printk("    %s\n", cbuf);
			n = 0;
			col = 0;
		} else if (!(col % 4))
			printk("  ");
	}

	if (col % 16) {
		while (col++ < 16)
			printk("    ");
		cbuf[n] = '\0';
		printk("  %s\n", cbuf);
	} else
		printk("\n");
}

void _dump_msg(int pid, int tid, int write, struct bcmd_msg *msg)
{
	printk("proc %d (tid %d) %s %s message:\n", pid, tid, write ? "write" : "read", (msg->type == BC_REPLY) ? "reply" : "transaction");
	printk("\tbinder %p, cookie %p, type %u, code %u, flags %u, uid %d, pid %d, queue %ld\n",
		msg->binder, msg->cookie, msg->type, msg->code, msg->flags, msg->sender_pid, msg->sender_euid, msg->reply_to);

	if (msg->buf) {
		struct bcmd_msg_buf *mbuf = msg->buf;

		if (mbuf->data_size > 0) {
			printk("\t data size %u\n", mbuf->data_size);
			_hexdump(mbuf->data, mbuf->data_size);

			if (mbuf->offsets_size > 0) {
				printk("\t offsets size %u\n", mbuf->offsets_size);
				_hexdump(mbuf->offsets, mbuf->offsets_size);
			}
		}
	}
}

static inline int binder_cmp(msg_queue_id owner0, void *binder0, msg_queue_id owner1, void *binder1)
{
	ssize_t sign;

	if ((sign = owner0 - owner1))
		return (sign > 0) ? 1 : -1;

	if ((sign = binder0 - binder1))
		return (sign > 0) ? 1 : -1;

	return 0;
}

static struct binder_obj *binder_find_obj(struct binder_proc *proc, msg_queue_id owner, void *binder)
{
	struct rb_node **p = &proc->obj_tree.rb_node;
	struct rb_node *parent = NULL;
	struct binder_obj *obj;
	int r;

	spin_lock(&proc->obj_lock);
	while (*p) {
		parent = *p;
		obj = rb_entry(parent, struct binder_obj, rb_node);

		r = binder_cmp(owner, binder, obj->owner, obj->binder);
		if (r < 0)
			p = &(*p)->rb_left;
		else if (r > 0)
			p = &(*p)->rb_right;
		else {
			spin_unlock(&proc->obj_lock);
			return obj;
		}
	}
	spin_unlock(&proc->obj_lock);

	return NULL;
}

static inline struct binder_obj *binder_find_my_obj(struct binder_proc *proc, void *binder)
{
	return binder_find_obj(proc, msg_queue_id(proc->queue), binder);
}

static struct binder_obj *binder_find_obj_by_ref(struct binder_proc *proc, unsigned long ref)
{
	struct binder_obj *obj;
	struct hlist_head *head;
	struct hlist_node *node;

	spin_lock(&proc->obj_lock);

	head = &proc->obj_hash[ref % OBJ_HASH_BUCKET_SIZE];
	hlist_for_each_entry(obj, node, head, hash_node) {
		if (obj->ref == ref) {
			spin_unlock(&proc->obj_lock);
			return obj;
		}
	}

	spin_unlock(&proc->obj_lock);
	return NULL;
}

static struct binder_obj *_binder_new_obj(struct binder_proc *proc, msg_queue_id owner, struct msg_queue *owner_queue, void *binder, void *cookie)
{
	struct rb_node **p = &proc->obj_tree.rb_node;
	struct rb_node *parent = NULL;
	struct binder_obj *obj, *new_obj;
	int r;

	new_obj = kmalloc(sizeof(*obj), GFP_KERNEL);
	if (!new_obj)
		return NULL;

	new_obj->owner = owner;
	new_obj->owner_queue = owner_queue;
	new_obj->binder = binder;
	new_obj->cookie = cookie;
	spin_lock_init(&new_obj->lock);
	INIT_LIST_HEAD(&new_obj->notifiers);

	atomic_set(&new_obj->refs, 0);

	spin_lock(&proc->obj_lock);
	while (*p) {
		parent = *p;
		obj = rb_entry(parent, struct binder_obj, rb_node);

		r = binder_cmp(owner, binder, obj->owner, obj->binder);
		if (r < 0)
			p = &(*p)->rb_left;
		else if (r > 0)
			p = &(*p)->rb_right;
		else {	// other thread has created an object before we do
			spin_unlock(&proc->obj_lock);
			kfree(new_obj);
			return obj;
		}
	}

	rb_link_node(&new_obj->rb_node, parent, p);
	rb_insert_color(&new_obj->rb_node, &proc->obj_tree);

	new_obj->ref = proc->obj_seq++;
	hlist_add_head(&new_obj->hash_node, &proc->obj_hash[new_obj->ref % OBJ_HASH_BUCKET_SIZE]);
	spin_unlock(&proc->obj_lock);

	debugfs_new_obj(proc, new_obj);
	return new_obj;
}

static inline struct binder_obj *binder_new_obj(struct binder_proc *proc, void *binder, void *cookie)
{
	return _binder_new_obj(proc, msg_queue_id(proc->queue), proc->queue, binder, cookie);
}

// used by the queue owner
inline int _bcmd_read_msg(struct msg_queue *q, struct bcmd_msg **pmsg)
{
	struct list_head *list = &(*pmsg)->list;
	int r;

	r = read_msg_queue(q, &list);
	if (!r)
		*pmsg = container_of(list, struct bcmd_msg, list);
	return r;
}

// used by any process
inline int bcmd_read_msg(msg_queue_id id, struct bcmd_msg **pmsg)
{
	struct msg_queue *q;
	int r;

	if (!(q = get_msg_queue(id)))
		return -ENODEV;

	r = _bcmd_read_msg(q, pmsg);

	put_msg_queue(q);
	return r;
}

// used by the queue owner
inline int _bcmd_write_msg(struct msg_queue *q, struct bcmd_msg *msg)
{
	return write_msg_queue(q, &msg->list);
}

// used by any process
inline int bcmd_write_msg(msg_queue_id id, struct bcmd_msg *msg)
{
	struct msg_queue *q;
	int r;

	if (!(q = get_msg_queue(id)))
		return -ENODEV;

	r = _bcmd_write_msg(q, msg);

	put_msg_queue(q);
	return r;
}

// used by the queue owner
inline int _bcmd_write_msg_head(struct msg_queue *q, struct bcmd_msg *msg)
{
	return write_msg_queue_head(q, &msg->list);
}

// used by any process
inline int bcmd_write_msg_head(msg_queue_id id, struct bcmd_msg *msg)
{
	struct msg_queue *q;
	int r;

	if (!(q = get_msg_queue(id)))
		return -ENODEV;

	r = _bcmd_write_msg_head(q, msg);

	put_msg_queue(q);
	return r;
}

static struct bcmd_msg *binder_alloc_msg(size_t data_size, size_t offsets_size)
{
	size_t num_objs, msg_size, msg_buf_size, buf_size;
	struct bcmd_msg *msg;
	struct bcmd_msg_buf *mbuf;

	num_objs = offsets_size / sizeof(size_t);
	msg_buf_size = sizeof(*mbuf) + num_objs * sizeof(msg_queue_id);
	msg_size = sizeof(*msg) + msg_buf_size;
	buf_size = msg_size + MSG_BUF_ALIGN(data_size) + MSG_BUF_ALIGN(offsets_size);

	msg = kmalloc(buf_size, GFP_KERNEL);
	if (!msg)
		return NULL;

	mbuf = (struct bcmd_msg_buf *)((char *)msg + sizeof(*msg));
	mbuf->data = (char *)msg + msg_size;
	mbuf->offsets = (char *)mbuf->data + MSG_BUF_ALIGN(data_size);

	mbuf->data_size = data_size;
	mbuf->offsets_size = offsets_size;
	mbuf->buf_size = buf_size;

	msg->buf = mbuf;
	msg->trace_depth = 0;
	return msg;
}

static struct bcmd_msg *binder_realloc_msg(struct bcmd_msg *msg, size_t data_size, size_t offsets_size)
{
	size_t num_objs, msg_size, msg_buf_size, buf_size;
	struct bcmd_msg_buf *mbuf = msg->buf;

	num_objs = offsets_size / sizeof(size_t);
	msg_buf_size = sizeof(*mbuf) + num_objs * sizeof(msg_queue_id);
	msg_size = sizeof(*msg) + msg_buf_size;
	buf_size = msg_size + MSG_BUF_ALIGN(data_size) + MSG_BUF_ALIGN(offsets_size);

	if (mbuf->buf_size >= buf_size) {
		mbuf->data = (char *)msg + msg_size;
		mbuf->offsets = (char *)mbuf->data + MSG_BUF_ALIGN(data_size);

		mbuf->data_size = data_size;
		mbuf->offsets_size = offsets_size;

		msg->trace_depth = 0;
		return msg;
	}

	kfree(msg);
	return binder_alloc_msg(data_size, offsets_size);
}

// used by the queue owner
static inline int _binder_write_cmd(struct msg_queue *q, void *binder, void *cookie, unsigned int cmd)
{
	struct bcmd_msg *msg;
	int r;

	msg = binder_alloc_msg(0, 0);
	if (!msg)
		return -ENOMEM;

	msg->type = cmd;
	msg->binder = binder;
	msg->cookie = cookie;

	r = _bcmd_write_msg(q, msg);
	if (r < 0) {
		kfree(msg);
		return r;
	}

	return 0;
}

// used by any process
static inline int binder_write_cmd(msg_queue_id q, void *binder, void *cookie, unsigned int cmd)
{
	struct bcmd_msg *msg;
	int r;

	msg = binder_alloc_msg(0, 0);
	if (!msg)
		return -ENOMEM;

	msg->type = cmd;
	msg->binder = binder;
	msg->cookie = cookie;

	r = bcmd_write_msg(q, msg);
	if (r < 0) {
		kfree(msg);
		return r;
	}

	return 0;
}

static inline int binder_inform_owner(struct binder_obj *obj, unsigned int cmd)
{
	return binder_write_cmd(obj->owner, obj->binder, obj->cookie, cmd);
}

static inline void binder_reclaim_obj(struct binder_proc *proc, struct binder_obj *obj)
{
	if (atomic_read(&proc->busy_threads) <= 1)
		kfree(obj);
	else {
		spin_lock(&proc->reclaim_lock);
		list_add(&obj->notifiers, &proc->reclaim_list);	// reuse notifiers entry
		spin_unlock(&proc->reclaim_lock);
	}
}

static inline void binder_reclaim_objs(struct binder_proc *proc)
{
	struct binder_obj *obj, *next;

	spin_lock(&proc->reclaim_lock);
	list_for_each_entry_safe(obj, next, &proc->reclaim_list, notifiers) {
		list_del(&obj->notifiers);
		kfree(obj);
	}
	spin_unlock(&proc->reclaim_lock);
}

static int _binder_free_obj(struct binder_proc *proc, struct binder_obj *obj)
{
	int r = 0;

	if (obj->info_node)
		debugfs_remove(obj->info_node);

	if (OBJ_IS_BINDER(obj)) {
		struct binder_notifier *notifier, *next;
		struct bcmd_msg *msg = NULL;

		list_for_each_entry_safe(notifier, next, &obj->notifiers, list) {
			list_del(&notifier->list);

			if (!msg) {
				msg = binder_alloc_msg(0, 0);
				if (!msg) {
					r = -ENOMEM;
					kfree(notifier);
					continue;
				}
			}

			msg->type = BR_DEAD_BINDER;
			msg->binder = obj->binder;
			msg->cookie = notifier->cookie;
			msg->reply_to = obj->owner;	// identify the owner
			if (!bcmd_write_msg(notifier->to_notify, msg))
				msg = NULL;

			kfree(notifier);
		}
		if (msg)
			kfree(msg);
	} else {
		// reference - tell the owner we are no longer referencing the object
		if (atomic_read(&obj->refs) > 0)
			binder_inform_owner(obj, BC_RELEASE);
	}

	binder_reclaim_obj(proc, obj);
	return r;
}

static int binder_free_obj(struct binder_proc *proc, struct binder_obj *obj, int force)
{
	spin_lock(&proc->obj_lock);
	rb_erase(&obj->rb_node, &proc->obj_tree);
	hlist_del(&obj->hash_node);
	spin_unlock(&proc->obj_lock);

	if (!force)
		return _binder_free_obj(proc, obj);

	if (obj->info_node)
		debugfs_remove(obj->info_node);
	binder_reclaim_obj(proc, obj);
	return 0;
}

static int clear_msg_buf(struct binder_proc *proc, struct bcmd_msg *msg)
{
	struct bcmd_msg_buf *mbuf = msg->buf;

	if (mbuf->offsets_size > 0) {
		struct flat_binder_object *bp;
		size_t *p, *ep, off;
		int n;

		p = (size_t *)mbuf->offsets;
		ep = (size_t *)((char *)mbuf->offsets + mbuf->offsets_size);
		n = 0;

		while (p < ep) {
			off = *p++;
			if (off + sizeof(*bp) > mbuf->data_size)
				return -EINVAL;

			bp = (struct flat_binder_object *)(mbuf->data + off);
			switch (bp->type) {
				case BINDER_TYPE_BINDER:
				case BINDER_TYPE_HANDLE:
					if (mbuf->owners[n] != msg_queue_id(proc->queue))
						binder_write_cmd(mbuf->owners[n], bp->binder, bp->cookie, BC_RELEASE);
					break;

				case BINDER_TYPE_FD:
					if (bp->binder)
						fput(bp->binder);
					break;

				default:
					break;
			}

			n++;
		}
	}

	return 0;
}

static void clear_msg_queue(struct binder_proc *proc, struct msg_queue *q)
{
	struct list_head *entry;
	struct bcmd_msg *msg;

	while ((entry = msg_queue_pop(q))) {
		msg = container_of(entry, struct bcmd_msg, list);

		if (msg->type == BC_TRANSACTION) {
			clear_msg_buf(proc, msg);

			if (!(msg->flags & TF_ONE_WAY)) {
				msg->type = BR_DEAD_REPLY;
				if (!bcmd_write_msg(msg->reply_to, msg))
					continue;
			}
		} else if (msg->type == BC_REPLY) {
			clear_msg_buf(proc, msg);
		} else if (msg->type == BC_CLEAR_DEATH_NOTIFICATION) {
			struct binder_obj *obj;

			obj = binder_find_my_obj(proc, msg->binder);
			if (obj) {
				struct binder_notifier *notifier, *next;

				spin_lock(&obj->lock);
				list_for_each_entry_safe(notifier, next, &obj->notifiers, list) {
					if (notifier->event == BINDER_EVT_OBJ_DEAD &&
					    notifier->cookie == msg->cookie) {
						list_del(&notifier->list);
						kfree(notifier);
						break;
					}
				}
				spin_unlock(&obj->lock);
			}
		}

		kfree(msg);
	}
}

static void thread_queue_release(struct msg_queue *q, void *data)
{
	struct binder_thread *thread = data;
	struct binder_proc *proc = thread->proc;
	struct bcmd_msg *msg, *next;

	if (thread->info_node)
		debugfs_remove(thread->info_node);

	clear_msg_queue(proc, q);

	list_for_each_entry_safe(msg, next, &thread->incoming_transactions, list) {
		list_del(&msg->list);

		msg->type = BR_DEAD_REPLY;
		if (bcmd_write_msg(msg->reply_to, msg) < 0)
			kfree(msg);
	}

	spin_lock(&proc->lock);
	rb_erase(&thread->rb_node, &proc->thread_tree);
	spin_unlock(&proc->lock);

	kfree(thread);
	put_msg_queue(proc->queue);
}

static void proc_queue_release(struct msg_queue *q, void *data)
{
	struct binder_proc *proc = data;
	struct rb_node *n;
	struct binder_obj *obj;

	clear_msg_queue(proc, q);

	// safe to free objs and send BR_DEAD_BINDER
	while ((n = rb_first(&proc->obj_tree))) {
		obj = rb_entry(n, struct binder_obj, rb_node);

		rb_erase(n, &proc->obj_tree);
		hlist_del(&obj->hash_node);

		_binder_free_obj(proc, obj);
	}

	if (proc->proc_dir)
		debugfs_remove_recursive(proc->proc_dir);

	if (proc->slob)
		fast_slob_destroy(proc->slob);

	kfree(proc);
}

static struct binder_proc *binder_new_proc(struct file *filp)
{
	struct binder_proc *proc;
	int i;

	proc = kmalloc(sizeof(*proc), GFP_KERNEL);
	if (!proc)
		return NULL;

	proc->queue = create_msg_queue(0, proc_queue_release, proc);
	if (!proc->queue) {
		kfree(proc);
		return NULL;
	}

	proc->slob = NULL;
	proc->slob_uses = 0;
	proc->ustart = 0;
	proc->pid = task_tgid_vnr(current);
	proc->max_threads = 0;

	atomic_set(&proc->busy_threads, 0);
	atomic_set(&proc->proc_loopers, 0);
	atomic_set(&proc->requested_loopers, 0);
	atomic_set(&proc->registered_loopers, 0);

	spin_lock_init(&proc->lock);
	proc->thread_tree.rb_node = NULL;

	for (i = 0; i < sizeof(proc->obj_hash) / sizeof(proc->obj_hash[0]); i++)
		INIT_HLIST_HEAD(&proc->obj_hash[i]);
	proc->obj_seq = 1;

	spin_lock_init(&proc->obj_lock);
	proc->obj_tree.rb_node = NULL;

	spin_lock_init(&proc->reclaim_lock);
	INIT_LIST_HEAD(&proc->reclaim_list);

	debugfs_new_proc(proc);
	return proc;
}

static struct binder_thread *binder_new_thread(struct binder_proc *proc, struct file *filp, pid_t pid)
{
	struct binder_thread *new_thread, *thread;
	struct rb_node **p = &proc->thread_tree.rb_node;
	struct rb_node *parent = NULL;

	new_thread = kmalloc(sizeof(*new_thread), GFP_KERNEL);
	if (!new_thread)
		return NULL;

	new_thread->queue = create_msg_queue(0, thread_queue_release, new_thread);
	if (!new_thread->queue || !get_msg_queue(msg_queue_id(proc->queue))) {
		kfree(new_thread);
		return NULL;
	}

	new_thread->pid = pid;
	new_thread->state = 0;
	new_thread->non_block = (filp->f_flags & O_NONBLOCK) ? 1 : 0;	// compat
	new_thread->pending_replies = 0;
	INIT_LIST_HEAD(&new_thread->incoming_transactions);
	new_thread->proc = proc;

	spin_lock(&proc->lock);
	while (*p) {
		parent = *p;
		thread = rb_entry(parent, struct binder_thread, rb_node);

		if (pid < thread->pid)
			p = &(*p)->rb_left;
		else if (pid > thread->pid)
			p = &(*p)->rb_right;
		else {
			spin_unlock(&proc->lock);
			BUG();
			free_msg_queue(new_thread->queue);
			kfree(new_thread);
			return thread;
		}
	}

	rb_link_node(&new_thread->rb_node, parent, p);
	rb_insert_color(&new_thread->rb_node, &proc->thread_tree);
	spin_unlock(&proc->lock);

	debugfs_new_thread(proc, new_thread);
	return new_thread;
}

static struct binder_thread *binder_get_thread(struct binder_proc *proc, struct file *filp)
{
	struct rb_node **p = &proc->thread_tree.rb_node;
	struct rb_node *parent = NULL;
	struct binder_thread *thread;
	pid_t pid = task_pid_vnr(current);

	spin_lock(&proc->lock);
	while (*p) {
		parent = *p;
		thread = rb_entry(parent, struct binder_thread, rb_node);

		if (pid < thread->pid)
			p = &(*p)->rb_left;
		else if (pid > thread->pid)
			p = &(*p)->rb_right;
		else {
			spin_unlock(&proc->lock);
			return thread;
		}
	}
	spin_unlock(&proc->lock);

	return binder_new_thread(proc, filp, pid);
}

static int binder_free_thread(struct binder_proc *proc, struct binder_thread *thread)
{
	free_msg_queue(thread->queue);
	return 0;
}

static int binder_free_proc(struct binder_proc *proc)
{
	struct rb_node *n;
	struct binder_thread *thread;

	disable_msg_queue(proc->queue);

	while ((n = rb_first(&proc->thread_tree))) {
		thread = rb_entry(n, struct binder_thread, rb_node);
		binder_free_thread(proc, thread);
	}

	free_msg_queue(proc->queue);
	return 0;
}

static inline int binder_acquire_obj(struct binder_proc *proc, struct binder_thread *thread, struct binder_obj *obj)
{
	int refs = atomic_inc_return(&obj->refs);

	if (refs == 1) {
		struct bcmd_msg *msg;
		int r;

		msg = binder_alloc_msg(0, 0);
		if (!msg)
			return -ENOMEM;

		msg->binder = obj->binder;
		msg->cookie = obj->cookie;

		if (OBJ_IS_BINDER(obj)) {
			msg->type = BR_ACQUIRE;
			r = _bcmd_write_msg(thread->queue, msg); // owner thread should receive BR_ACQUIRE
		} else {
			msg->type = BC_ACQUIRE;
			r = bcmd_write_msg(obj->owner, msg); // tell the owner we are referencing it
		}

		if (r < 0) {
			kfree(msg);
			return r;
		}
	}

	return refs;
}

static inline int binder_release_obj(struct binder_proc *proc, struct binder_thread *thread, struct binder_obj *obj)
{
	int refs = atomic_dec_return(&obj->refs);

	if (refs == 0) {
		struct bcmd_msg *msg;
		int r;

		msg = binder_alloc_msg(0, 0);
		if (!msg)
			return -ENOMEM;

		msg->binder = obj->binder;
		msg->cookie = obj->cookie;

		if (OBJ_IS_BINDER(obj)) {
			msg->type = BR_RELEASE;
			r = _bcmd_write_msg(thread->queue, msg);
		} else {
			msg->type = BC_RELEASE;
			r = bcmd_write_msg(obj->owner, msg); // tell the owner we are no longer referencing it
		}

		if (r < 0) {
			kfree(msg);
			return r;
		}

		// regardless it's an object or a reference, it should cease to exist now
		r = binder_free_obj(proc, obj, 0);
		if (r < 0)
			return r;
	}

	return refs;
}

static int bcmd_write_flat_obj(struct binder_proc *proc, struct binder_thread *thread, struct flat_binder_object *bp, msg_queue_id *owner)
{
	unsigned long type = bp->type;
	struct binder_obj *obj;
	struct file *file;
	int r;

	switch (type) {
		case BINDER_TYPE_BINDER:
		case BINDER_TYPE_WEAK_BINDER:
			obj = binder_find_my_obj(proc, bp->binder);
			if (!obj) {
				obj = binder_new_obj(proc, bp->binder, bp->cookie);
				if (!obj)
					return -ENOMEM;
			} else if (bp->cookie != obj->cookie)
				return -EINVAL;

			*owner = obj->owner;

			if (type == BINDER_TYPE_BINDER) {
				r = binder_acquire_obj(proc, thread, obj);
				if (r < 0)
					return r;
			}
			break;

		case BINDER_TYPE_HANDLE:
		case BINDER_TYPE_WEAK_HANDLE:
			obj = binder_find_obj_by_ref(proc, bp->handle);
			if (!obj)
				return -EINVAL;

			bp->binder = obj->binder;
			bp->cookie = obj->cookie;
			*owner = obj->owner;

			if (type == BINDER_TYPE_HANDLE) {
				r = binder_inform_owner(obj, BC_ACQUIRE);
				if (r < 0)
					return r;
			}
			break;

		case BINDER_TYPE_FD:
			file = fget(bp->handle);
			if (!file)
				return -EINVAL;

			bp->binder = file;
			*owner = 0;		// unused
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int bcmd_read_flat_obj(struct binder_proc *proc, struct binder_thread *thread, struct flat_binder_object *bp, msg_queue_id owner)
{
	struct binder_obj *obj;
	struct file *file;
	unsigned long type = bp->type;
	int fd, r;

	switch (type) {
		case BINDER_TYPE_BINDER:
		case BINDER_TYPE_HANDLE:
		case BINDER_TYPE_WEAK_BINDER:
		case BINDER_TYPE_WEAK_HANDLE:
			obj = binder_find_obj(proc, owner, bp->binder);
			if (!obj) {
				obj = _binder_new_obj(proc, owner, NULL, bp->binder, bp->cookie);
				if (!obj)
					return -ENOMEM;
			}

			if (OBJ_IS_BINDER(obj)) {
				if (type == BINDER_TYPE_HANDLE)
					bp->type = BINDER_TYPE_BINDER;
				else if (type == BINDER_TYPE_WEAK_HANDLE)
					bp->type = BINDER_TYPE_WEAK_BINDER;

				bp->cookie = obj->cookie;
			} else {
				if (type == BINDER_TYPE_BINDER)
					bp->type = BINDER_TYPE_HANDLE;
				else if (type == BINDER_TYPE_WEAK_BINDER)
					bp->type = BINDER_TYPE_WEAK_HANDLE;

				if (bp->type == BINDER_TYPE_HANDLE) {
					/* Reference has already been increased by the writer for us,
					   so we just need to increase our local counter */
					if (atomic_inc_return(&obj->refs) > 1) {
						/* We aleady have a reference to the object, so tell
						   the owner to decrease one reference */
						r = binder_inform_owner(obj, BC_RELEASE);
						if (r < 0)
							return r;
					}
				}
			}
			bp->handle = (long)obj->ref;
			break;

		case BINDER_TYPE_FD:
			file = (struct file *)bp->binder;
			fd = get_unused_fd();	// compat/TODO: O_CLOEXEC
			if (fd < 0) {
				fput(file);
				return -ENOMEM;
			}

			fd_install(fd, file);
			bp->handle = fd;
			break;

		default:
			return -EFAULT;
	}

	return 0;
}

static int bcmd_write_msg_buf(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_msg_buf *mbuf, struct bcmd_transaction_data *tdata)
{
	size_t *p, *ep, off;
	struct flat_binder_object *bp;
	int n, r;

	if (copy_from_user(mbuf->data, tdata->data.ptr.buffer, mbuf->data_size))
		return -EFAULT;

	if (!mbuf->offsets_size)
		return 0;
	if (copy_from_user(mbuf->offsets, tdata->data.ptr.offsets, mbuf->offsets_size))
		return -EFAULT;

	n = 0;
	p = (size_t *)mbuf->offsets;
	ep = (size_t *)((char *)mbuf->offsets + mbuf->offsets_size);
	while (p < ep) {
		off = *p++;
		if (off + sizeof(*bp) > mbuf->data_size)
			return -EINVAL;

		bp = (struct flat_binder_object *)(mbuf->data + off);

		r = bcmd_write_flat_obj(proc, thread, bp, mbuf->owners + n++);
		if (r < 0)
			return r;
	}

	return 0;
}

/* The call backtrace machenism implemented in the following two functions is to find the destination thread
   (to deliver the transaction to, instead of the normal path - process queue) in the case of recurisive calls.

   It addresses scenarios like,
	1. A -> B -> A. In this case, B's transaction is delivered back to thread A, instead of A's process queue.
	2. A -> B -> C -> A. In this case, B's transaction is delivered to C's process queue, but C's transaction
           is delivered to thread A.

   It doesn't impose above thread preference if a caller thread makes consecutive calls to more than one destination
   before getting the reply back. For example, like in case 2, if A -> B -> C, and before getting reply back from B,
   A makes another call to C (A -> C), in which case, the second message is not guranteed to be delivered to thread
   C who received B's transaction (triggered by A's transaction).
*/
static inline int bcmd_fill_traces(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_msg *msg)
{
	struct bcmd_msg_trace *traces = msg->traces;
	struct bcmd_msg *trace_msg;
	int n, i;

	traces[0].caller_proc = msg_queue_id(proc->queue);
	traces[0].caller_thread = msg_queue_id(thread->queue);
	n = 1;

	list_for_each_entry(trace_msg, &thread->incoming_transactions, list) {
		for (i = 0; i < trace_msg->trace_depth; i++) {
			traces[n++] = trace_msg->traces[i];
			if (n >= MAX_TRACE_DEPTH) {
				printk("binder: pid %d (tid %d) transaction reached maximum call traces (%d), possible dead loop?\n",
					proc->pid, thread->pid, MAX_TRACE_DEPTH);
				msg->trace_depth = n;
				return -1;
			}
		}
	}

	msg->trace_depth = n;
	return 0;
}

static inline int bcmd_lookup_caller(struct binder_proc *proc, struct binder_thread *thread, msg_queue_id *dest)
{
	struct bcmd_msg *trace_msg;
	msg_queue_id caller_proc = *dest;
	int i;

	list_for_each_entry(trace_msg, &thread->incoming_transactions, list) {
		for (i = 0; i < trace_msg->trace_depth; i++) {
			if (trace_msg->traces[i].caller_proc == caller_proc) {
				*dest = trace_msg->traces[i].caller_thread;
				return 1;
			}
		}
	}

	return 0;
}

static int bcmd_write_transaction(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_transaction_data *tdata, uint32_t bcmd)
{
	struct bcmd_msg *msg;
	msg_queue_id to_id;
	void *binder, *cookie;

	if (bcmd == BC_TRANSACTION) {
		struct binder_obj *obj;

		if (unlikely(!tdata->target.handle))
			obj = context_mgr_obj;
		else
			obj = binder_find_obj_by_ref(proc, tdata->target.handle);

		if (!obj)
			goto failed_reply;

		msg = binder_alloc_msg(tdata->data_size, tdata->offsets_size);
		if (!msg)
			goto failed_reply;

		to_id = obj->owner;
		if (!(tdata->flags & TF_ONE_WAY)) {
			if (bcmd_fill_traces(proc, thread, msg) < 0)
				goto failed_msg;
			bcmd_lookup_caller(proc, thread, &to_id);
		}

		binder = obj->binder;
		cookie = obj->cookie;
	} else {
		/* compat: pop out the top transaction without checking. The big issue
		   here is the reply message doesn't carry enough information we could use
		   to check its validity, in particular if there are more than one pending
		   incoming transactions on the stack waiting to be replied. See comments
		   in bcmd_read_transaction() */
		if (list_empty(&thread->incoming_transactions))
			goto failed_reply;

		msg = list_first_entry(&thread->incoming_transactions, struct bcmd_msg, list);
		list_del(&msg->list);

		to_id = msg->reply_to;
		binder = cookie = NULL;		// compat

		msg = binder_realloc_msg(msg, tdata->data_size, tdata->offsets_size);
		if (!msg)
			goto failed_reply;
	}

	msg->type = bcmd;
	msg->binder = binder;
	msg->cookie = cookie;
	msg->code = tdata->code;
	msg->flags = tdata->flags;
	msg->sender_pid = proc->pid;
	msg->sender_euid = current->cred->euid;
	msg->reply_to = msg_queue_id(thread->queue);	// reply queue & indicating source

	if (tdata->data_size > 0) {
		if (bcmd_write_msg_buf(proc, thread, msg->buf, tdata) < 0)
			goto failed_msg;
	}
	DUMP_MSG(proc->pid, thread->pid, 1, msg);

	/* compat: send BR_TRANSACTION_COMPLETE to the calling thread. It has to be written to the
	   thread queue after the message ('msg') has been assembled, so that the referencing commands
	   (BR_*, done in bcmd_write_msg_buf()) are ahead of COMPLETE, but before 'msg' is written to
	   the destination queue. If COMPLETE is written after that, there's a chance that the reply
	   message (of 'msg') arrives earlier before COMPLETE is enqueued, in which case the framework
	   will simply treat the reply message as the response and return immediately before draining
	   COMPLETE. The COMPLETE message left behind will then mess up the next transaction. */
	if (_binder_write_cmd(thread->queue, binder, cookie, BR_TRANSACTION_COMPLETE) < 0)
		goto failed_write;

	if (bcmd_write_msg(to_id, msg) < 0)
		goto failed_write;

	if (bcmd == BC_TRANSACTION && !(tdata->flags & TF_ONE_WAY))
		thread->pending_replies++;
	return 0;

failed_write:
	clear_msg_buf(proc, msg);
failed_msg:
	kfree(msg);
failed_reply:
	return _binder_write_cmd(thread->queue, NULL, NULL, BR_FAILED_REPLY);
}

static int bcmd_write_free_buffer(struct binder_proc *proc, struct binder_thread *thread, void *uaddr)
{
	size_t off;
	struct slob_buf *sbuf;
	int bucket;

	if (!proc->slob || !proc->ustart || (unsigned long)uaddr < proc->ustart) {
		printk("binder: pid %d (tid %d) trying to free an invalid buffer %p, slob %p, ustart %lx\n",
			proc->pid, thread->pid, uaddr, proc->slob, proc->ustart);
		return -EINVAL;
	}

	off = (unsigned long)uaddr - proc->ustart - (unsigned long)(((struct slob_buf *)0)->data);
	sbuf = (struct slob_buf *)(proc->slob->start + off);

	bucket = fast_slob_bucket(proc->slob, sbuf);
	if (bucket < 0 || (sbuf->uaddr_data != (unsigned long)uaddr)) {
		printk("binder: pid %d (tid %d) trying to free an invalid buffer %p, bucket %d, sbuf %p\n",
			proc->pid, thread->pid, uaddr, bucket, sbuf);
		return -EINVAL;
	}

	if (sbuf->offsets_size > 0) {
		struct flat_binder_object *bp;
		struct binder_obj *obj;
		size_t *p, *ep;

		off = sbuf->uaddr_offsets - sbuf->uaddr_data;
		p = (size_t *)(sbuf->data + off);
		ep = (size_t *)((char *)p + sbuf->offsets_size);
		while (p < ep) {
			bp = (struct flat_binder_object *)(sbuf->data + *p++);

			if (bp->type == BINDER_TYPE_BINDER)
				obj = binder_find_my_obj(proc, bp->binder);
			else if (bp->type == BINDER_TYPE_HANDLE)
				obj = binder_find_obj_by_ref(proc, bp->handle);
			else
				continue;

			if (obj)
				binder_release_obj(proc, thread, obj);
		}
	}

	_fast_slob_free(proc->slob, bucket, sbuf);
	return 0;
}

static int bcmd_write_acquire(struct binder_proc *proc, struct binder_thread *thread, unsigned long ref, uint32_t bcmd)
{
	struct binder_obj *obj;
	int r;

	if (bcmd != BC_ACQUIRE && bcmd != BC_RELEASE)
		return 0;

	if (unlikely(!ref))
		obj = context_mgr_obj;
	else
		obj = binder_find_obj_by_ref(proc, ref);

	if (!obj)
		return -EINVAL;

	if (bcmd == BC_ACQUIRE)
		r = binder_acquire_obj(proc, thread, obj);
	else
		r = binder_release_obj(proc, thread, obj);

	return (r < 0) ? r : 0;
}

static int bcmd_write_notifier(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_notifier_data *notifier, uint32_t bcmd)
{
	struct binder_obj *obj;
	struct bcmd_msg *msg;
	int r;

	obj = binder_find_obj_by_ref(proc, notifier->handle);
	if (!obj)
		return -EINVAL;

	msg = binder_alloc_msg(0, 0);
	if (!msg)
		return -ENOMEM;

	msg->type = bcmd;
	msg->binder = obj->binder;
	msg->cookie = notifier->cookie;

	// dead_binder sent to the process queue, while clear_notifcation_done sent to the request thread
	msg->reply_to = (bcmd == BC_REQUEST_DEATH_NOTIFICATION) ? msg_queue_id(proc->queue) : msg_queue_id(thread->queue);

	if ((r = bcmd_write_msg(obj->owner, msg)) < 0) {
		kfree(msg);
		return r;
	}

	return 0;
}

static int bcmd_write_looper(struct binder_proc *proc, struct binder_thread *thread, uint32_t bcmd)
{
	switch (bcmd) {
		case BC_ENTER_LOOPER:
			if (thread->state & BINDER_LOOPER_STATE_READY)
				return -EINVAL;
			else
				thread->state |= BINDER_LOOPER_STATE_ENTERED;
			break;

		case BC_EXIT_LOOPER:
			if (thread->state & BINDER_LOOPER_STATE_ENTERED)
				thread->state &= ~BINDER_LOOPER_STATE_READY;
			else
				return -EINVAL;
			break;

		case BC_REGISTER_LOOPER:
			if (thread->state & BINDER_LOOPER_STATE_READY)
				return -EINVAL;
			else {
				atomic_inc(&proc->registered_loopers);
				atomic_dec(&proc->requested_loopers);
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static long binder_thread_write(struct binder_proc *proc, struct binder_thread *thread, char __user *buf, char __user *end)
{
	char __user *p = buf;
	uint32_t bcmd;
	int r;

	while ((p + sizeof(bcmd)) <= end) {
		if (get_user(bcmd, (uint32_t *)p))
			return -EFAULT;
		p += sizeof(bcmd);

		switch (bcmd) {
			case BC_TRANSACTION:
			case BC_REPLY:  {
				struct bcmd_transaction_data tdata;

				if ((p + sizeof(tdata)) > end || copy_from_user(&tdata, p, sizeof(tdata)))
					return -EFAULT;
				p += sizeof(tdata);

				if (tdata.data_size > 0) {
					size_t objs_size = tdata.offsets_size / sizeof(size_t) * sizeof(struct flat_binder_object);

					if (objs_size > tdata.data_size)
						return -EINVAL;
				}

				r = bcmd_write_transaction(proc, thread, &tdata, bcmd);
				if (r < 0) {
					printk("binder: pid %d (tid %d) wrote transaction/reply failed: %d\n",
						proc->pid, thread->pid, r);
					return r;
				}
				break;
			}

			case BC_FREE_BUFFER: {
				void *buffer;

				if ((p + sizeof(void *)) > end || get_user(buffer, (void __user **)p))
					return -EFAULT;
				p += sizeof(void *);

				/* compat: there're transactions containing no data, e.g. PING_TRANSACTION, but the
				   framework still sends us FREE_BUFFER command for them (with a NULL buffer). */
				if (buffer) {
					r = bcmd_write_free_buffer(proc, thread, buffer);
					if (r < 0) {
						printk("binder: pid %d (tid %d) wrote free_buffer failed: %d\n",
							proc->pid, thread->pid, r);
						return r;
					}
				}
				break;
			}

			case BC_ACQUIRE:
			case BC_RELEASE:
			case BC_INCREFS:
			case BC_DECREFS: {
				void *handle;

				if ((p + sizeof(void *)) > end || get_user(handle, (void __user **)p))
					return -EFAULT;
				p += sizeof(void *);

				r = bcmd_write_acquire(proc, thread, (unsigned long)handle, bcmd);
				if (r < 0) {
					printk("binder: pid %d (tid %d) wrote acquire/release failed: %d\n",
						proc->pid, thread->pid, r);
					//return r;	// compat: don't return error if target no longer exists
				}
				break;
			}

			case BC_REQUEST_DEATH_NOTIFICATION:
			case BC_CLEAR_DEATH_NOTIFICATION: {
				struct bcmd_notifier_data notifier;

				if ((p + sizeof(notifier)) > end || copy_from_user(&notifier, p, sizeof(notifier)))
					return -EFAULT;
				p += sizeof(notifier);

				r = bcmd_write_notifier(proc, thread, &notifier, bcmd);
				if (r < 0) {
					printk("binder: pid %d (tid %d) wrote notifier failed: %d\n",
						proc->pid, thread->pid, r);
					//return r;	// compat: don't return error if target no longer exists
				}
				break;
			}

			case BC_ENTER_LOOPER:
			case BC_EXIT_LOOPER:
			case BC_REGISTER_LOOPER:
				r = bcmd_write_looper(proc, thread, bcmd);
				if (r < 0) {
					printk("binder: pid %d (tid %d) wrote looper failed: %d\n",
						proc->pid, thread->pid, r);
					return r;
				}
				break;

			case BC_DEAD_BINDER_DONE:
				if ((p + sizeof(void *)) > end)
					return -EFAULT;

				// compat: not used
				p += sizeof(void *);
				break;

			case BC_INCREFS_DONE:
			case BC_ACQUIRE_DONE:
				if ((p + 2 * sizeof(void *)) > end)
					return -EFAULT;

				// compat: not used
				p += 2 * sizeof(void *);
				break;

			default:
				printk("binder: pid %d (tid %d) wrote unknown binder command %x\n",
					proc->pid, thread->pid, bcmd);
				return -EINVAL;
		}
	}

	return p - buf;
}

static long bcmd_read_transaction(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_msg **pmsg, void __user *buf, unsigned long size)
{
	struct bcmd_transaction_data tdata;
	struct bcmd_msg *msg = *pmsg;
	struct bcmd_msg_buf *mbuf = msg->buf;
	uint32_t cmd = (msg->type == BC_TRANSACTION) ? BR_TRANSACTION : BR_REPLY;
	size_t data_size;
	int r;

	if (sizeof(cmd) + sizeof(tdata) > size)
		return -ENOSPC;

	tdata.target.ptr = msg->binder;
	tdata.code = msg->code;
	tdata.cookie = msg->cookie;
	tdata.flags = msg->flags;
	tdata.sender_pid = msg->sender_pid;
	tdata.sender_euid = msg->sender_euid;

	tdata.data_size = mbuf->data_size;
	tdata.offsets_size = mbuf->offsets_size;

	data_size = MSG_BUF_ALIGN(mbuf->data_size) + MSG_BUF_ALIGN(mbuf->offsets_size);
	if (data_size > 0) {
		struct slob_buf *sbuf;

		if (proc->slob && proc->ustart) {
			sbuf = fast_slob_alloc(proc->slob, sizeof(*sbuf) + data_size);
			if (!sbuf) {
				printk("binder: pid %d (tid %d) failed to allocate transaction data (%u)\n",
					proc->pid, thread->pid, data_size);
				return -ENOMEM;
			}
		} else
			return -ENOMEM;

		sbuf->data_size = mbuf->data_size;
		sbuf->offsets_size = mbuf->offsets_size;

		sbuf->uaddr_data = proc->ustart + (sbuf->data - proc->slob->start);
		tdata.data.ptr.buffer = (void *)sbuf->uaddr_data;

		if (mbuf->offsets_size > 0) {
			size_t *p, *ep;
			struct flat_binder_object *bp;
			int n;

			n = 0;
			p = (size_t *)mbuf->offsets;
			ep = (size_t *)(mbuf->offsets + mbuf->offsets_size);
			while (p < ep) {
				bp = (struct flat_binder_object *)(mbuf->data + *p++);

				r = bcmd_read_flat_obj(proc, thread, bp, mbuf->owners[n++]);
				if (r < 0)
					return r;
			}

			sbuf->uaddr_offsets = sbuf->uaddr_data + (mbuf->offsets - mbuf->data);
		} else
			sbuf->uaddr_offsets = 0;
		tdata.data.ptr.offsets = (void *)sbuf->uaddr_offsets;

		memcpy(sbuf->data, mbuf->data, data_size);
	} else
		tdata.data.ptr.buffer = tdata.data.ptr.offsets = NULL;

	if (put_user(cmd, (uint32_t *)buf) ||
	    copy_to_user(buf + sizeof(cmd), &tdata, sizeof(tdata)))
		return -EFAULT;
	DUMP_MSG(proc->pid, thread->pid, 0, msg);

	if (msg->type == BC_TRANSACTION) {
		if (!(msg->flags & TF_ONE_WAY)) {
			/* This is where things get nasty. When launching an app, a call scenario can be
			   1. Launcher calls ActivityManager
			   2. ActivityManager calls SurfaceComposer
			   3. SurfaceComposer calls ActivityManager (for permission of frame_buffer)
			   which causes ActivityManager to have two incoming transactions on the stack.
			   It appears that it has to follow a strict FILO order, and requires the application
			   to follow the same order. Because there's no strict sequencing or alike to enforce
			   the order, things can easily go wrong. */
			list_add(&msg->list, &thread->incoming_transactions);
			msg = NULL;
		}
	} else {
		if (thread->pending_replies > 0)
			thread->pending_replies--;
	}

	if (msg)
		kfree(msg);
	*pmsg = NULL;

	return (sizeof(cmd) + sizeof(tdata));
}

static long bcmd_read_notifier(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_msg **pmsg, void __user *buf, unsigned long size)
{
	struct bcmd_msg *msg = *pmsg;
	struct binder_notifier *notifier;
	struct binder_obj *obj;

	if (msg->type == BR_CLEAR_DEATH_NOTIFICATION_DONE) {
		if (size < sizeof(uint32_t) * 2)
			return -ENOSPC;

		if (put_user(BR_CLEAR_DEATH_NOTIFICATION_DONE, (uint32_t *)buf) ||
		    put_user((uint32_t)msg->cookie, (uint32_t *)((char *)buf + sizeof(uint32_t))))
			return -EFAULT;

		kfree(msg);
		*pmsg = NULL;
		return sizeof(uint32_t) * 2;
	}

	obj = binder_find_my_obj(proc, msg->binder);
	if (!obj)
		return -EFAULT;

	if (msg->type == BC_REQUEST_DEATH_NOTIFICATION) {
		// TODO: check duplication?
		notifier = kmalloc(sizeof(*notifier), GFP_KERNEL);
		if (!notifier)
			return -ENOMEM;

		notifier->event = BINDER_EVT_OBJ_DEAD;
		notifier->cookie = msg->cookie;
		notifier->to_notify = msg->reply_to;

		spin_lock(&obj->lock);
		list_add_tail(&notifier->list, &obj->notifiers);
		spin_unlock(&obj->lock);

		kfree(msg);
	} else {
		struct binder_notifier *next;
		int found = 0;

		spin_lock(&obj->lock);
		list_for_each_entry_safe(notifier, next, &obj->notifiers, list) {
			if (notifier->event == BINDER_EVT_OBJ_DEAD &&
			    notifier->cookie == msg->cookie) {
				found = 1;
				list_del(&notifier->list);
				break;
			}
		}
		spin_unlock(&obj->lock);

		if (found) {
			msg->type = BR_CLEAR_DEATH_NOTIFICATION_DONE;
			if (bcmd_write_msg(msg->reply_to, msg) < 0)
				kfree(msg);
			kfree(notifier);
		} else
			kfree(msg);
	}

	*pmsg = NULL;
	return 0;
}

static long bcmd_read_transaction_complete(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_msg **pmsg, void __user *buf, unsigned long size)
{
	uint32_t cmd = (*pmsg)->type;

	if (size < sizeof(cmd))
		return -ENOSPC;

	if (put_user(cmd, (uint32_t *)buf))
		return -EFAULT;

	kfree(*pmsg);
	*pmsg = NULL;
	return sizeof(cmd);
}

static long bcmd_read_dead_binder(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_msg **pmsg, void __user *buf, unsigned long size)
{
	struct bcmd_msg *msg = *pmsg;
	uint32_t cmd = msg->type, cookie = (uint32_t)msg->cookie;
	struct binder_obj *obj;

	if (size < sizeof(cmd) * 2)
		return -ENOSPC;

	obj = binder_find_obj(proc, msg->reply_to, msg->binder);
	if (obj) {
		binder_free_obj(proc, obj, 1);

		if (put_user(cmd, (uint32_t *)buf) ||
		    put_user(cookie, (uint32_t *)((char *)buf + sizeof(cmd))))
			return -EFAULT;
	}

	kfree(msg);
	*pmsg = NULL;
	return sizeof(cmd) * 2;
}

static long bcmd_read_dead_reply(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_msg **pmsg, void __user *buf, unsigned long size)
{
	uint32_t cmd = (*pmsg)->type;

	if (size < sizeof(cmd))
		return -ENOSPC;

	if (cmd == BR_DEAD_REPLY && thread->pending_replies > 0)
		thread->pending_replies--;

	if (put_user(cmd, (uint32_t *)buf))
		return -EFAULT;

	kfree(*pmsg);
	*pmsg = NULL;
	return sizeof(cmd);
}

static long bcmd_read_acquire(struct binder_proc *proc, struct binder_thread *thread, struct bcmd_msg **pmsg, void __user *buf, unsigned long size)
{
	struct bcmd_msg *msg = *pmsg;
	struct bcmd_ref_return ref_cmd;
	unsigned int cmd = 0;
	int r;

	if (size < sizeof(ref_cmd))
		return -ENOSPC;

	if (msg->type == BR_ACQUIRE || msg->type == BR_RELEASE)
		cmd = msg->type;
	else {
		struct binder_obj *obj;

		obj = binder_find_my_obj(proc, msg->binder);
		if (!obj) {
			printk("binder: pid %d (tid %d) got ref command (%x) after the object (%p) is removed\n",
				proc->pid, thread->pid, msg->type, msg->binder);
			r = 0;
			goto obj_removed;
		}

		if (msg->type == BC_ACQUIRE) {
			if (atomic_inc_return(&obj->refs) == 1)
				cmd = BR_ACQUIRE;
		} else {
			if (atomic_dec_return(&obj->refs) == 0) {
				cmd = BR_RELEASE;
				binder_free_obj(proc, obj, 0);
			}
		}
	}

	if (cmd) {
		ref_cmd.cmd = cmd;
		ref_cmd.binder = msg->binder;
		ref_cmd.cookie = msg->cookie;

		if (copy_to_user(buf, &ref_cmd, sizeof(ref_cmd)))
			return -EFAULT;
		r = sizeof(ref_cmd);
	} else
		r = 0;

obj_removed:
	kfree(*pmsg);
	*pmsg = NULL;
	return r;
}

static int bcmd_spawn_on_busy(struct binder_proc *proc, struct binder_thread *thread, void __user *buf, unsigned long size)
{
	uint32_t cmd = BR_SPAWN_LOOPER;
	int n;

	if (size < sizeof(cmd))
		return 0;

	n = msg_queue_size(proc->queue);

	if ((atomic_read(&proc->proc_loopers) < n) &&
	    (atomic_read(&proc->registered_loopers) < proc->max_threads) &&
	    !atomic_read(&proc->requested_loopers)) {
		if (put_user(cmd, (uint32_t *)buf))
			return -EFAULT;

		atomic_inc(&proc->requested_loopers);
		return sizeof(cmd);
	}

	return 0;
}

static long binder_thread_read(struct binder_proc *proc, struct binder_thread *thread, char __user *buf, char __user *end)
{
	struct msg_queue *q;
	struct bcmd_msg *msg = NULL;
	char __user *p = buf;
	ssize_t size = end - buf;
	int proc_looper = 0, force_return = 0;
	long n;

	if (thread->state & BINDER_LOOPER_STATE_READY) {	// compat: only ready threads can request spawn
		n = bcmd_spawn_on_busy(proc, thread, p, size);
		if (n)	// spawn or error returned immediately
			return n;
	}

	while (size >= sizeof(uint32_t) && !force_return) {
		if (thread->pending_replies > 0 || !msg_queue_empty(thread->queue))
			q = thread->queue;
		else {
			q = proc->queue;

			proc_looper = 1;
			atomic_inc(&proc->proc_loopers);
		}

		if (msg_queue_empty(q) && thread->non_block)
			break;

		n = _bcmd_read_msg(q, &msg);
		if (n < 0)
			goto clean_up;

		if (proc_looper) {
			atomic_dec(&proc->proc_loopers);
			proc_looper = 0;
		}

		switch (msg->type) {
			case BC_TRANSACTION:
			case BC_REPLY:
				n = bcmd_read_transaction(proc, thread, &msg, p, size);
				force_return = 1;
				break;

			case BR_TRANSACTION_COMPLETE:
				n = bcmd_read_transaction_complete(proc, thread, &msg, p, size);
				force_return = 1;
				break;

			case BC_ACQUIRE:
			case BC_RELEASE:
			case BR_ACQUIRE:
			case BR_RELEASE:
				n = bcmd_read_acquire(proc, thread, &msg, p, size);
				if (n > 0)
					force_return = 1;
				break;

			case BC_REQUEST_DEATH_NOTIFICATION:
			case BC_CLEAR_DEATH_NOTIFICATION:
			case BR_CLEAR_DEATH_NOTIFICATION_DONE:
				n = bcmd_read_notifier(proc, thread, &msg, p, size);
				if (n > 0)
					force_return = 1;
				break;

			case BR_DEAD_BINDER:
				n = bcmd_read_dead_binder(proc, thread, &msg, p, size);
				force_return = 1;
				break;

			case BR_DEAD_REPLY:
			case BR_FAILED_REPLY:
				n = bcmd_read_dead_reply(proc, thread, &msg, p, size);
				force_return = 1;
				break;

			default:
				kfree(msg);
				n = -EFAULT;
				goto clean_up;
		}

		if (msg && (n != -ENOSPC))
			kfree(msg);

		if (n > 0) {
			p += n;
			size -= n;
		} else if (n < 0) {
			if (n == -ENOSPC) {
				if (msg) {	// put msg back to the queue
					printk("binder: proc %d (tid %d): not enough read space, put message back\n",
						proc->pid, thread->pid);
					n = _bcmd_write_msg_head(q, msg);
					if (n < 0) {
						kfree(msg);
						goto clean_up;
					}
				}
				n = 0;		// TODO: review no-space handling
			}
			break;
		}
	}

clean_up:
	if (proc_looper)
		atomic_dec(&proc->proc_loopers);

	if (n < 0)
		return n;
	else
		return (p - buf);
}

static inline int cmd_write_read(struct binder_proc *proc, struct binder_thread *thread, struct binder_write_read *bwr)
{
	int r;

	if (bwr->write_size > 0 && bwr->write_consumed < bwr->write_size) {
		r = binder_thread_write(proc, thread,
					(char __user *)bwr->write_buffer + bwr->write_consumed,
					(char __user *)bwr->write_buffer + bwr->write_size);
		if (r < 0)
			return r;
		bwr->write_consumed += r;
	}

	if (bwr->read_size > 0 && bwr->read_consumed < bwr->read_size) {
		r = binder_thread_read(proc, thread,
					(char __user *)bwr->read_buffer + bwr->read_consumed,
					(char __user *)bwr->read_buffer + bwr->read_size);
		if (r < 0)
			return r;
		bwr->read_consumed += r;
	}

	return 0;
}

static inline int cmd_thread_exit(struct binder_proc *proc, struct binder_thread *thread)
{
	return binder_free_thread(proc, thread);
}

static inline int cmd_set_max_threads(struct binder_proc *proc, int max_threads)
{
	spin_lock(&proc->lock);
	proc->max_threads = max_threads;
	spin_unlock(&proc->lock);
	return 0;
}

static inline int cmd_set_context_mgr(struct binder_proc *proc)
{
	struct binder_obj *obj;

	if (context_mgr_obj)
		return -EBUSY;

	if (context_mgr_uid == -1)
		context_mgr_uid = current->cred->euid;
	else if (context_mgr_uid != current->cred->euid)
		return -EPERM;

	obj = binder_new_obj(proc, NULL, NULL);
	if (!obj)
		return -ENOMEM;

	// increase reference count, so it never gets deleted
	atomic_set(&obj->refs, 100000);
	smp_mb();

	context_mgr_obj = obj;
	return 0;
}

static int binder_open(struct inode *nodp, struct file *filp)
{
	struct binder_proc *proc;

	proc = binder_new_proc(filp);
	if (!proc)
		return -ENOMEM;

	filp->private_data = proc;
	return 0;
}

static int binder_release(struct inode *nodp, struct file *filp)
{
	struct binder_proc *proc = filp->private_data;

	if (context_mgr_obj && (context_mgr_obj->owner_queue == proc->queue))
		context_mgr_obj = NULL;

	// TODO: make sure existing referencing context_mgr_obj is safe

	binder_free_proc(proc);
	return 0;
}

static long binder_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct binder_proc *proc = filp->private_data;
	struct binder_thread *thread;
	unsigned int size = _IOC_SIZE(cmd);
	void __user *ubuf = (void __user *)arg;
	int r;

	thread = binder_get_thread(proc, filp);
	if (!thread)
		return -ENOMEM;

	switch (cmd) {
		case BINDER_WRITE_READ: {
			struct binder_write_read bwr;

			if (size != sizeof(bwr))
				return -EINVAL;
			if (copy_from_user(&bwr, ubuf, sizeof(bwr)))
				return -EFAULT;

			atomic_inc(&proc->busy_threads);

			r = cmd_write_read(proc, thread, &bwr);

			/* no one is referencing any objects, so it's safe to do reclaiming now */
			if (!atomic_dec_return(&proc->busy_threads) && !list_empty(&proc->reclaim_list))
				binder_reclaim_objs(proc);

			/* copy bwr back regardlessly in case we've done write but got interrupted
			   in read */
			if (copy_to_user(ubuf, &bwr, sizeof(bwr)))
				return -EFAULT;

			return (r < 0) ? r : 0;
		}

		case BINDER_THREAD_EXIT:
			return cmd_thread_exit(proc, thread);

		case BINDER_SET_MAX_THREADS: {
			int max_threads;

			if (size != sizeof(int))
				return -EINVAL;
			if (get_user(max_threads, (int *)ubuf))
				return -EFAULT;

			return cmd_set_max_threads(proc, max_threads);
		}

		case BINDER_VERSION:
			if (size != sizeof(struct binder_version))
				return -EINVAL;
			if (put_user(BINDER_CURRENT_PROTOCOL_VERSION, &((struct binder_version *)ubuf)->protocol_version))
				return -EFAULT;
			return 0;

		case BINDER_SET_CONTEXT_MGR:
			return cmd_set_context_mgr(proc);

		default:
			printk("binder: pid %d (tid %d) unknown ioctl command %x\n",
				proc->pid, thread->pid, cmd);
			return -EINVAL;
	}
}

static unsigned int binder_poll(struct file *filp, poll_table *p)
{
	struct binder_proc *proc = filp->private_data;
	struct binder_thread *thread;

	thread = binder_get_thread(proc, filp);
	if (!thread)
		return -ENOMEM;

	msg_queue_poll_wait_read(proc->queue, filp, p);
	msg_queue_poll_wait_read(thread->queue, filp, p);

	if (!msg_queue_empty(thread->queue) || msg_queue_size(proc->queue) > 0)
		return POLLIN | POLLRDNORM;

	// TODO: consider POLLOUT case as write can block too (not compat)
	return 0;
}

static int binder_flush(struct file *filp, fl_owner_t id)
{
	return 0;	// compat
}

static void binder_vm_open(struct vm_area_struct *vma)
{
	struct binder_proc *proc = vma->vm_private_data;

	proc->slob_uses++;
}

static void binder_vm_close(struct vm_area_struct *vma)
{
	struct binder_proc *proc = vma->vm_private_data;

	if (--proc->slob_uses <= 0)
		proc->ustart = 0;
}

static struct vm_operations_struct binder_vm_ops = {
	.open = binder_vm_open,
	.close = binder_vm_close,
};

static int binder_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct binder_proc *proc = filp->private_data;
	size_t size = vma->vm_end - vma->vm_start;
	int r;

	if (size > 4096 * 1024)		// compat
		size = 4096 * 1024;

	if (vma->vm_flags & VM_WRITE)
		return -EPERM;

	if (proc->ustart || proc->slob)	// TODO: free existing slob?
		return -EBUSY;

	/* compat: sericemanager has a map size of 128K and the rest uses (1024-8)k */
	if (size < 512 * 1024)
		proc->slob = fast_slob_create(size, 16 * 1024, 4, 2);
	else
		proc->slob = fast_slob_create(size, 128 * 1024, 3, 4);
	if (!proc->slob)
		return -ENOMEM;

	r = remap_vmalloc_range(vma, proc->slob->start, 0);
	if (r < 0) {
		fast_slob_destroy(proc->slob);
		proc->slob = NULL;
		return r;
	}
	vma->vm_flags = vma->vm_flags | VM_DONTCOPY | VM_DONTEXPAND;
	vma->vm_ops = &binder_vm_ops;
	vma->vm_private_data = proc;

	proc->ustart = vma->vm_start;
	proc->slob_uses = 1;
	return 0;
}

static int debugfs_proc_info(struct seq_file *seq, void *start)
{
	struct binder_proc *proc = seq->private;

	seq_printf(seq, "pid: %d\n", proc->pid);
	seq_printf(seq, "queue: %p\n", proc->queue);
	seq_printf(seq, "obj_seq: %lu\n", proc->obj_seq);
	seq_printf(seq, "max_threads: %d\n", proc->max_threads);
	seq_printf(seq, "registered_loopers: %d\n", atomic_read(&proc->registered_loopers));
	seq_printf(seq, "proc_loopers: %d\n", atomic_read(&proc->proc_loopers));
	seq_printf(seq, "requested_loopers: %d\n", atomic_read(&proc->requested_loopers));

	return 0;
}

static int debugfs_thread_info(struct seq_file *seq, void *start)
{
	struct binder_thread *thread = seq->private;

	seq_printf(seq, "pid: %d\n", thread->pid);
	seq_printf(seq, "queue: %p\n", thread->queue);
	seq_printf(seq, "state: %d\n", thread->state);
	seq_printf(seq, "non_block: %d\n", thread->non_block);
	seq_printf(seq, "pending_replies: %d\n", thread->pending_replies);

	return 0;
}

static int debugfs_obj_info(struct seq_file *seq, void *start)
{
	struct binder_obj *obj = seq->private;

	seq_printf(seq, "ref: %lu\n", obj->ref);
	seq_printf(seq, "type: %s\n", OBJ_IS_BINDER(obj) ? "binder" : "handle");
	seq_printf(seq, "owner: %ld\n", obj->owner);
	seq_printf(seq, "binder: %p\n", obj->binder);
	seq_printf(seq, "cookie: %p\n", obj->cookie);
	seq_printf(seq, "refs: %d\n", atomic_read(&obj->refs));

	return 0;
}

static int debugfs_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_proc_info, inode->i_private);
}

static int debugfs_thread_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_thread_info, inode->i_private);
}

static int debugfs_obj_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_obj_info, inode->i_private);
}

static const struct file_operations debugfs_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static const struct file_operations debugfs_thread_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_thread_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static const struct file_operations debugfs_obj_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_obj_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static inline int debugfs_new_proc(struct binder_proc *proc)
{
	struct dentry *d;
	char str[32];

	sprintf(str, "%d", proc->pid);
	proc->proc_dir = debugfs_create_file(str, S_IFDIR | S_IRWXU | S_IRUGO | S_IXUGO,
				debugfs_root, proc, NULL);
	if (!proc->proc_dir)
		goto no_proc;

	proc->thread_dir = debugfs_create_file("threads", S_IFDIR | S_IRWXU | S_IRUGO | S_IXUGO,
				proc->proc_dir, proc, NULL);
	if (!proc->thread_dir)
		goto no_threads;

	proc->obj_dir = debugfs_create_file("objs", S_IFDIR | S_IRWXU | S_IRUGO | S_IXUGO,
				proc->proc_dir, proc, NULL);
	if (!proc->obj_dir)
		goto no_objs;

	d = debugfs_create_file("info", S_IRUGO, proc->proc_dir, proc, &debugfs_proc_fops);
	if (!d)
		goto no_info;
	return 0;

no_info:
	debugfs_remove(proc->obj_dir);
no_objs:
	debugfs_remove(proc->thread_dir);
no_threads:
	debugfs_remove(proc->proc_dir);
no_proc:
	proc->proc_dir = proc->thread_dir = proc->obj_dir = NULL;
	return -ENOMEM;
}

static inline int debugfs_new_thread(struct binder_proc *proc, struct binder_thread *thread)
{
	if (proc->thread_dir) {
		char str[32];

		sprintf(str, "%d", thread->pid);
		thread->info_node = debugfs_create_file(str, S_IRUGO, proc->thread_dir, thread, &debugfs_thread_fops);

		return thread->info_node ? 0 : -ENOMEM;
	} else
		return -ENOMEM;
}

static inline int debugfs_new_obj(struct binder_proc *proc, struct binder_obj *obj)
{
	if (proc->obj_dir) {
		char str[32];

		sprintf(str, "%lu", obj->ref);
		obj->info_node = debugfs_create_file(str, S_IRUGO, proc->obj_dir, obj, &debugfs_obj_fops);

		return obj->info_node ? 0 : -ENOMEM;
	} else
		return -ENOMEM;
}

static int __init binder_debugfs_init(void)
{
	debugfs_root = debugfs_create_dir("binder", NULL);

	if (!debugfs_root)
		return -ENODEV;
	return 0;
}

static const struct file_operations binder_fops = {
	.owner = THIS_MODULE,
	.open = binder_open,
	.release = binder_release,
	.unlocked_ioctl = binder_ioctl,
	.poll = binder_poll,
	.mmap = binder_mmap,
	.flush = binder_flush
};

static struct miscdevice binder_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "binder",
	.fops = &binder_fops
};

static int __init binder_init(void)
{
	int r;

	r = misc_register(&binder_miscdev);
	if (r < 0)
		return r;

	r = binder_debugfs_init();
	if (r < 0)
		return r;

	return 0;
}

static void __exit binder_exit(void)
{
	misc_deregister(&binder_miscdev);

	debugfs_remove(debugfs_root);
}

module_init(binder_init);
module_exit(binder_exit);
MODULE_LICENSE("GPL v2");
