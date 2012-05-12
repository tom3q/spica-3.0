/*
 * msg_queue.h: a generic process messaging queue implementation
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
#ifndef _MSG_QUEUE_H
#define _MSG_QUEUE_H

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/rbtree.h>
#include <linux/wait.h>
#include <linux/poll.h>


#define DEFAULT_MAX_QUEUE_LENGTH		100


struct msg_queue;

typedef unsigned long msg_queue_id;
typedef void (*queue_release_handler)(struct msg_queue *q, void *);


struct msg_queue {
	msg_queue_id id;

	spinlock_t lock;
	int active;

	size_t num_msgs, max_msgs;
	struct list_head msgs;

	wait_queue_head_t rd_wait;
	wait_queue_head_t wr_wait;

	struct rb_node rb_node;
	int usage;

	queue_release_handler release;
	void *private;
};


extern struct msg_queue *create_msg_queue(size_t max_msgs, queue_release_handler handler, void *data);
extern int free_msg_queue(struct msg_queue *q);

extern struct msg_queue *get_msg_queue(msg_queue_id id);
extern int put_msg_queue(struct msg_queue *q);

extern int write_msg_queue(struct msg_queue *q, struct list_head *msg);
extern int write_msg_queue_head(struct msg_queue *q, struct list_head *msg);

extern int read_msg_queue(struct msg_queue *q, struct list_head **pmsg);
extern int read_msg_queue_tail(struct msg_queue *q, struct list_head **pmsg);


#define msg_queue_id(q)		(q)->id

/* Following inline functions should be called either by the queue owner or
 * with a reference held via a call to get_msg_queue() previously
 */
static inline void enable_msg_queue(struct msg_queue *q)
{
	q->active = 1;
}

static inline void disable_msg_queue(struct msg_queue *q)
{
	q->active = 0;
}

static inline int msg_queue_active(struct msg_queue *q)
{
	return (q->active > 0);
}

static inline int msg_queue_empty(struct msg_queue *q)
{
	return (q->num_msgs < 1);
}

static inline int msg_queue_full(struct msg_queue *q)
{
	return (q->num_msgs >= q->max_msgs);
}

static inline size_t msg_queue_size(struct msg_queue *q)
{
	return q->num_msgs;
}

// Unsafe! Only use it when no one else is accessing the queue
static inline struct list_head *msg_queue_pop(struct msg_queue *q)
{
	struct list_head *next = q->msgs.next;

	if (next != &q->msgs) {
		list_del(next);
		return next;
	} else
		return NULL;
}

static inline void msg_queue_poll_wait(struct msg_queue *q, struct file *filp, poll_table *p)
{
	poll_wait(filp, &q->rd_wait, p);
	poll_wait(filp, &q->wr_wait, p);
}

static inline void msg_queue_poll_wait_read(struct msg_queue *q, struct file *filp, poll_table *p)
{
	poll_wait(filp, &q->rd_wait, p);
}

static inline void msg_queue_poll_wait_write(struct msg_queue *q, struct file *filp, poll_table *p)
{
	poll_wait(filp, &q->wr_wait, p);
}
#endif /* _MSG_QUEUE_H */
