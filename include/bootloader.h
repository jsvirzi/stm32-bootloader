#ifndef BOOTLOAD_H
#define BOOTLOAD_H

#include "queue.h"

struct Bootloader;
struct BootloaderRecord;

#define MaxPageSize (256)

typedef struct Bootloader {
    int fd;
    Queue *rx_queue;
    // Pool *tx_pool;
    // unsigned int id;

    unsigned int serial_timeout_ms;

    /* system and time functions that need to be provided */
    int (*print)(const char *msg, unsigned int length);
    void (*delay)(unsigned int delay_ms);
    uint32_t (*time)(void);
    void (*usleep)(unsigned int period_us); /* short sleep */

    uint8_t caps[16];
    uint8_t caps_size;
    uint32_t base_address;
    unsigned int erase_page_size;
    unsigned int write_page_size;
    unsigned int read_page_size;
    unsigned int verbose;
    uint8_t *data_buffer;
    unsigned int data_buffer_size;
    uint8_t *verify_buffer;
    unsigned int verify_buffer_size;

    /* api */
    int (*connect)(struct Bootloader *bootloader);
    int (*get)(struct Bootloader *bootloader);
    int (*go)(struct Bootloader *bootloader, uint32_t address);
    int (*chip_id)(struct Bootloader *bootloader);
    int (*open_records)(struct Bootloader *bootloader, const char *filename);
    int (*program_records)(struct Bootloader *bootloader, int *fd);
    int (*erase)(struct Bootloader *bootloader, unsigned int start_page, unsigned int pages);
    int (*mass_erase)(struct Bootloader *bootloader);
    int (*extended_erase)(struct Bootloader *bootloader, unsigned int start_page, unsigned int pages);
    int (*extended_mass_erase)(struct Bootloader *bootloader, int bank);
    int (*read_memory)(Bootloader *bootloader, uint32_t address, uint8_t *buff, unsigned int n_bytes);
    int (*write_memory)(Bootloader *bootloader, uint32_t address, uint8_t *buff, unsigned int n_bytes);
} Bootloader;

#define BootloaderInit (0x7f)
#define BootloaderAckByte (0x79)
#define BootloaderNackByte (0x1f)
#define BootloaderGetCommand (0x00)
#define BootloaderReadProtection (0x01)
#define BootloaderGetId (0x02)
#define BootloaderReadMemory (0x11)
#define BootloaderGo (0x21)
#define BootloaderWriteMemory (0x31)
#define BootloaderErase (0x43)
#define BootloaderExtendedErase (0x44)
#define BootloaderWriteProtect (0x63)
#define BootloaderWriteUnprotect (0x73)

#define BootloaderRecordSyncByte (0xaa)
#define BootloaderRecordDataPacket (0x00)

enum {
    BootloaderSuccess = 0,
    BootloaderErrorNack,
    BootloaderTimeout,
    BootloaderInvalidResponse,
    BootloaderErrorChecksum,
    BootloaderErrorFormat,
    BootloaderError, /* generic error */
    BootloaderErrors
};

int bootload(Bootloader *bootloader);
int program_record(Bootloader *bootloader, BootloaderRecord *record);

enum {
    ModuleIdIdle = 0,
    ModuleIdSystem, /* print(), etc */
    ModuleIds
} ModuleId;

enum {
    EraseOptionGlobal = 0,
    EraseOptionBank1,
    EraseOptionBank2,
    EraseOptions,
    EraseOptionInvalid = EraseOptions
};

#endif
