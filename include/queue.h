#ifndef QUEUE_H
#define QUEUE_H

#include <inttypes.h>

#include "bootloader.h"

#define MaxRxSize (512)
#define MaxTxSize (512)
#define TxPoolSize 16

typedef struct Queue {
    uint8_t buff[MaxRxSize];
    unsigned int head, tail, mask;
    unsigned long int cntr;
} Queue;

typedef struct Pool {
    uint8_t buff[TxPoolSize][MaxTxSize];
    unsigned int owner[TxPoolSize];
    unsigned int length[TxPoolSize];
    unsigned int pool_size;
    unsigned int buff_size;
    unsigned int position; /* to keep circular rotation */
    unsigned int mask;
} Pool;

void initialize_pool(Pool *pool);
void initialize_queue(Queue *queue);
//int queue_recv(Bootloader *bootloader, uint8_t *buff, int n, int timeout_ms);
int get_xmit_buffer(Pool *pool, unsigned int id, uint8_t **buff, unsigned int **length);
int flush_queue(Queue *q);

#endif

