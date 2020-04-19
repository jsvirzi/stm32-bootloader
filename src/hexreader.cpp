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

void copy_record(BootloaderRecord *dst, BootloaderRecord *src) {
    dst->base_address = src->base_address;

    if (dst->base_address == 0x00008d00) {
        int x = 5;
    }

    dst->segment_address = src->segment_address;
    dst->size = src->size;
    dst->type = src->type;
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
    memset(record, 0, sizeof(BootloaderRecord));
    int errors = 0, new_record = 0;
    while ((new_record == 0) && (nread = getline(&line, &len, fp)) != -1) {

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
        uint32_t base_address = atoh(gp_string);

        calc_checksum += (base_address >> 0) & 0xff;
        calc_checksum += (base_address >> 8) & 0xff;

        gp_string[0] = line[idx++];
        gp_string[1] = line[idx++];
        gp_string[2] = 0;
        uint8_t type = atoh(gp_string);

        calc_checksum += (type & 0xff);

        if (type == DataRecord) {
            if ((work_record->base_address == 0) && (work_record->size == 0)) {
                work_record->base_address = base_address; /* just starting off. this will happen only once at beginning */
            } else if ((base_address != (work_record->base_address + work_record->size)) && (new_record == 0)) {
            /*** we are no longer contiguous, write record and start new one ***/
                copy_record(record, work_record);
                work_record->base_address = base_address;
                work_record->size = 0;
                new_record = 1;
            }

            for (int i = 0; i < record_size; ++i) {
                gp_string[0] = line[idx++];
                gp_string[1] = line[idx++];
                gp_string[2] = 0;
                uint8_t byte = atoh(gp_string) & 0xff;

                calc_checksum += (byte & 0xff);

                work_record->payload[work_record->size++] = byte;
                if ((work_record->size == work_record->max_size) && (new_record == 0)) {
                    copy_record(record, work_record); /* use record to program new sector of flash */
                    work_record->base_address += work_record->size; /* update for next iteration */
                    work_record->size = 0;
                    new_record = 1;
                }
            }

            /* checksum */
            gp_string[0] = line[idx++];
            gp_string[1] = line[idx++];
            gp_string[2] = 0;
            uint8_t read_checksum = atoh(gp_string);
            uint8_t checksum = (calc_checksum + read_checksum) & 0xff;
            if (checksum != 0) {
                ++errors;
                printf("index = %d. length = %zu. checksums = %2.2x / %2.2x %2.2x\n", idx, nread,
                       calc_checksum, read_checksum, checksum);
            }

        } else if (type == ExtendedLinearAddressRecord) {

            gp_string[0] = line[idx++];
            gp_string[1] = line[idx++];
            gp_string[2] = line[idx++];
            gp_string[3] = line[idx++];
            gp_string[4] = 0;
            uint32_t base_address = atoh(gp_string);

            calc_checksum += ((base_address >> 0) & 0xff);
            calc_checksum += ((base_address >> 8) & 0xff);

            work_record->segment_address = atoh(gp_string);
            work_record->segment_address = (work_record->segment_address << 16);
            printf("extended linear address = 0x%8.8x\n", work_record->segment_address);

            gp_string[0] = line[idx++];
            gp_string[1] = line[idx++];
            gp_string[2] = 0;
            uint8_t read_checksum = atoh(gp_string);

            uint8_t checksum = (calc_checksum + read_checksum) & 0xff;

            if (checksum != 0) {
                ++errors;
                printf("index = %d. length = %zu. checksums = %2.2x / %2.2x %2.2x\n", idx, nread,
                       calc_checksum, read_checksum, checksum);
            }

        } else if (type == 5) {
            printf("type = 5. line = [%s]\n", line);
        } else {
            printf("record type = %d found\n", type);
        }
    }

    if (new_record) {
        return new_record;
    }

    if (work_record->size > 0) {
        copy_record(record, work_record);
        work_record->base_address += work_record->size;
        work_record->size = 0;
        return 1;
    }

    free(line);
    fclose(fp);
    return 0;
}
