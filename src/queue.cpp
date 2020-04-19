#include <stdlib.h>
#include <stdio.h>

#include <bootloader.h>
#include "queue.h"

/* x should never be 0. 0 is reserved for idle to indicate available */
static unsigned int test_and_set(unsigned int *p, unsigned int x) {
    volatile unsigned int *q = p;
    /* TODO replace by atomic assembly instruction! */
    if (*q == 0) {
        *q = x;
    }
    return *q;
}

/* returns length of available buffer. length = 0 is an error condition */
int get_xmit_buffer(Pool *pool, unsigned int id, uint8_t **buff, unsigned int **length) {

    for (int i = 0; i < pool->pool_size; ++i) {
        if (pool->owner[pool->position] == 0) {
            unsigned int *request = &pool->owner[pool->position];
            uint16_t owner = test_and_set(request, id);
            if (owner == id) { /* buffer granted */
                *buff = pool->buff[pool->position];
                *length = &pool->length[pool->position];
                pool->length[pool->position] = 0; /* caller fills in when ready to transmit */
                return pool->buff_size; /* advise caller of maximum length */
            }
        }
        pool->position = (pool->position + 1) & pool->mask;
    }
    return 0; /* no buffer was available. this should be flagged as an error condition */
}

int flush_queue(Queue *q) {
    /* TODO next line is for debugging. not part of formal port */
    if (q->tail != q->head) {
        int n = q->head - q->tail;
        printf("flush_queue() flushed %d bytes = %d - %d : ", n, q->head, q->tail);
        for (int i = 0; i < n; ++i) {
            printf("0x%2.2x ", q->buff[(q->tail + i) & q->mask]);
        }
        printf("\n");
    }
    q->tail = q->head;
    return EXIT_SUCCESS;
}