#ifndef _INST_H
#define _INST_H

#undef KERNEL_INSTRUMENTING

#ifdef KERNEL_INSTRUMENTING
#include <linux/time.h>

typedef union {
	struct timeval tv;
	char label[8];
} inst_entry_t;

typedef struct {
	uint32_t magic;
	uint32_t seq;
	uint32_t max_entries;
	uint32_t next_entry;
	inst_entry_t entries[0];
} inst_buf_t;


static inline void __inst_entry(void *ptr, char *label, struct timeval *copy)
{
	inst_buf_t *inst = (inst_buf_t *)ptr;

	if (inst->magic != 0x696e7374)
		return;

	if (inst->next_entry < inst->max_entries) {
		inst_entry_t *entry = inst->entries + inst->next_entry++;

		if (inst->seq) {
			if (!copy)
				do_gettimeofday(&entry->tv);
			else
				entry->tv = *copy;
		} else {
			strncpy(entry->label, label, 8);
			entry->label[7] = '\0';
		}
	}
}
#define INST_ENTRY(p, label)			__inst_entry((p), (label), NULL)
#define INST_ENTRY_COPY(t, p, label, n)		__inst_entry((p), (label), (t)->__inst_copies + (n))
#define INST_RECORD(t, n)			do_gettimeofday((t)->__inst_copies + (n));
#else
#define INST_ENTRY(b, label)
#define INST_ENTRY_COPY(t, p, label, n)
#define INST_RECORD(t, n)
#endif /* KERNEL_INSTRUMENTING */

#endif /* _INST_H */
