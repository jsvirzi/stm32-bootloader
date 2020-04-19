#ifndef HEXREADER_H
#define HEXREADER_H

#include <stdio.h>
#include <stdint.h>

#pragma pack(1)
typedef struct BootloaderRecord {
    uint32_t segment_address;
    uint32_t base_address;
    uint16_t size;
    uint16_t max_size;
    uint8_t type, checksum;
    uint8_t payload[256];
} BootloaderRecord;

FILE *hexreader_init(const char *ifile);
int hexreader_next(FILE *fp, BootloaderRecord *work_record, BootloaderRecord *record);

#endif
