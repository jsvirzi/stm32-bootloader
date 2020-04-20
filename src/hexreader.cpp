//
// Created by jsvirzi on 6/18/19.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>
#include <fcntl.h>
#include <sys/stat.h>

#include "hexreader.h"

uint8_t ascii_to_hex[128] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

uint32_t atoh(const char *s) {
    int idx = 0;
    uint32_t x = 0;
    while (s[idx]) {
        x = (x << 4) | ascii_to_hex[s[idx++] & 0x7f];
    }
    return x;
}

#define MaxRecordSize (256)
#define SyncByte (0xaa)
#define DataRecord (0x00)
#define ExtendedLinearAddressRecord (0x04)

/* final records use the following fields: address, size, payload, checksum */
void make_final_record(BootloaderRecord *dst, BootloaderRecord *src) {
    bzero(dst, sizeof(BootloaderRecord));
    dst->usage = BootloaderRecordUsageFinal;
    dst->address = src->base_address;
    dst->size = src->size;
    /* N corresponding to N+1 bytes of payload */
    dst->checksum = src->size - 1;
    for (int i = 0; i < src->size; ++i) { dst->checksum = dst->checksum ^ src->payload[i]; }
    memcpy(dst->payload, src->payload, src->size);
}

FILE *hexreader_init(const char *ifile)  {
    FILE *fp = fopen(ifile, "r");
    if (fp == NULL) {
        printf("unable to open input file [%s]\n", ifile);
        return NULL;
    }
    return fp;
}

int hexreader_next(FILE *fp, BootloaderRecord *work_record, BootloaderRecord *record) {
    if (fp == 0) { return 0; }
    char gp_string[1024];
    char *line = NULL;
    size_t len = 0;
    ssize_t nread;
    int errors = 0;
    record->usage = BootloaderRecordUsageWork;
    while ((record->usage != BootloaderRecordUsageFinal) && (nread = getline(&line, &len, fp)) != -1) {

        int idx = 0;

        if (line[idx++] != ':') { continue; } /* not a proper line */

        uint8_t calc_checksum = 0;
        gp_string[0] = line[idx++];
        gp_string[1] = line[idx++];
        gp_string[2] = 0;
        unsigned int record_size = atoh(gp_string);

        calc_checksum += (record_size & 0xff);

        gp_string[0] = line[idx++];
        gp_string[1] = line[idx++];
        gp_string[2] = line[idx++];
        gp_string[3] = line[idx++];
        gp_string[4] = 0;
        uint32_t address = atoh(gp_string);

        calc_checksum += (address >> 0) & 0xff;
        calc_checksum += (address >> 8) & 0xff;

        gp_string[0] = line[idx++];
        gp_string[1] = line[idx++];
        gp_string[2] = 0;
        uint8_t type = atoh(gp_string);

        calc_checksum += (type & 0xff);

        if (type == DataRecord) {
            uint32_t effective_address = work_record->segment_address + address;
            if (work_record->size == 0) { /* just starting out with new extended linear address */
                work_record->base_address = effective_address;
                work_record->address = work_record->base_address;
            }

            uint32_t expected_address = effective_address;
            if (work_record->address != expected_address) {
            /*** we are no longer contiguous, write record and start new one ***/
                make_final_record(record, work_record);
                work_record->base_address = effective_address;
                work_record->address = effective_address;
                work_record->size = 0;
            }

            for (int i = 0; i < record_size; ++i) {
                gp_string[0] = line[idx++];
                gp_string[1] = line[idx++];
                gp_string[2] = 0;
                uint8_t byte = atoh(gp_string) & 0xff;

                calc_checksum += (byte & 0xff);

                work_record->payload[work_record->size] = byte;
                ++work_record->size;
                ++work_record->address;
                if (work_record->size == work_record->max_size) {
                    make_final_record(record, work_record); /* use record to program new sector of flash */
                    work_record->base_address += work_record->size;
                    work_record->size = 0;
                }
            }

            if (record_size != 0) { work_record->dirty = 1; }

            /* checksum */
            gp_string[0] = line[idx++];
            gp_string[1] = line[idx++];
            gp_string[2] = 0;
            uint8_t read_checksum = atoh(gp_string);
            uint8_t checksum = (calc_checksum + read_checksum) & 0xff;
            if (checksum != 0) {
                ++errors;
                printf("ERROR: index = %d. length = %zu. checksums = %2.2x / %2.2x %2.2x\n", idx, nread,
                       calc_checksum, read_checksum, checksum);
            }

        } else if (type == ExtendedLinearAddressRecord) {

            if (work_record->dirty && (work_record->size != 0)) {
                make_final_record(record, work_record);
            }

            work_record->dirty = 0; /* start record in virgin state */
            work_record->size = 0;

            gp_string[0] = line[idx++];
            gp_string[1] = line[idx++];
            gp_string[2] = line[idx++];
            gp_string[3] = line[idx++];
            gp_string[4] = 0;
            address = atoh(gp_string);

            calc_checksum += ((address >> 8) & 0xff);
            calc_checksum += ((address >> 0) & 0xff);

            work_record->segment_address = (address << 16);
            printf("extended linear address = 0x%8.8x\n", work_record->segment_address);

            gp_string[0] = line[idx++];
            gp_string[1] = line[idx++];
            gp_string[2] = 0;
            uint8_t read_checksum = atoh(gp_string);

            uint8_t checksum = (calc_checksum + read_checksum) & 0xff;

            if (checksum != 0) {
                ++errors;
                printf("ERROR: index = %d. length = %zu. checksums = %2.2x / %2.2x %2.2x\n", idx, nread,
                       calc_checksum, read_checksum, checksum);
            }

        } else if (type == 5) {
            printf("type = 5. line = [%s]\n", line);
        } else {
            printf("record type = %d found\n", type);
        }
    }

    if (record->usage == BootloaderRecordUsageFinal) {
        return 1;
    }

    if (work_record->size > 0) {
        make_final_record(record, work_record);
        work_record->size = 0;
        return 1;
    }

    free(line);
    fclose(fp);
    return 0;
}
