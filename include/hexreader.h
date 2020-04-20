#ifndef HEXREADER_H
#define HEXREADER_H

#include <stdio.h>
#include <stdint.h>

#define MaxRecordSize (256)

enum {
    BootloaderRecordUsageWork = 0,
    BootloaderRecordUsageFinal,
    BootloaderRecordUsages
};

#pragma pack(1)
typedef struct BootloaderRecord {
    uint32_t segment_address;
    uint32_t base_address;
    uint32_t address; /* final address */
    uint16_t size;
    uint16_t max_size;
    uint8_t type, checksum, dirty, usage;
    uint8_t payload[MaxRecordSize];
} BootloaderRecord;

FILE *hexreader_init(const char *ifile);
int hexreader_next(FILE *fp, BootloaderRecord *work_record, BootloaderRecord *record);

#endif
