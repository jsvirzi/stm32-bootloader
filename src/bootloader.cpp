#include <inttypes.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "hexreader.h"
#include "bootloader.h"

unsigned int debug = 0;

uint32_t InvalidAddress = 0xffffffff;
static int transmit_byte(Bootloader *bootloader, uint8_t byte);
static int transmit_buff(Bootloader *bootloader, uint8_t* buff, unsigned int n);
static int recv_find(Bootloader *bootloader, uint8_t byte);
static int send_command(Bootloader *bootloader, uint8_t command);

/* threads */
#include <sched.h>
#include <pthread.h>

/* time */
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>

/* serial communications */
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>

/* TODO where to get these buffers */
static uint8_t erase_pages_buff[512 * 2 + 2 + 1];
static uint8_t bl_data_buff[512 * 2 + 2 + 1];

int queue_recv(Bootloader *bootloader, uint8_t *buff, int n) {
    Queue *q = bootloader->rx_queue;
    uint64_t timeout = bootloader->time() + bootloader->serial_timeout_ms;
    int k = 0;
    do {
        while (q->tail != q->head) {
            buff[k++] = q->buff[q->tail];
            q->tail = (q->tail + 1) & q->mask;
            if (k >= n) { return EXIT_SUCCESS; } /* success */
        }
    } while (bootloader->time() < timeout);
    return EXIT_FAILURE; /* timeout */
}

int flush_queue(Bootloader *bootloader) {
    Queue *q = bootloader->rx_queue;
    if (q->tail != q->head) {
        int n = (q->head - q->tail) & q->mask;
        printf("flush_queue() flushed %d bytes = %d - %d : ", n, q->head, q->tail);
        for (int i = 0; i < n; ++i) { printf("0x%2.2x ", q->buff[(q->tail + i) & q->mask]); }
        printf("\n");
    }
    q->tail = q->head;
    return EXIT_SUCCESS;
}

int print(const char *msg, unsigned int length) {
    printf("%s", msg);
    return EXIT_SUCCESS;
}

/* parity = 0 (no parity), = 1 odd parity, = 2 even parity */
int initialize_serial_port(const char *dev, bool canonical, int parity, int min_chars) {
    // int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if(fd < 0) { return fd; }
    fcntl(fd, F_SETFL, 0);
    struct termios *settings, current_settings;

    memset(&current_settings, 0, sizeof(current_settings));
    tcgetattr(fd, &current_settings);

    /* effect new settings */
    settings = &current_settings;
    cfmakeraw(settings);
    if (parity == 0) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB | PARENB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= CS8; /* eight bits */
    } else if (parity == 1) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= (CS8 | PARENB | PARODD); /* eight bits, odd parity */
    } else if (parity == 2) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= (CS8 | PARENB); /* eight bits, odd parity is clear for even parity */
    }
    settings->c_cflag |= (CLOCAL | CREAD); /* ignore carrier detect. enable receiver */
    settings->c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    settings->c_iflag |= ( IGNPAR | IGNBRK);
    settings->c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE);
    if (canonical) { settings->c_lflag |= ICANON; } /* set canonical */
    else { settings->c_lflag &= ~ICANON; } /* or clear it */
    settings->c_oflag &= ~(OPOST | ONLCR);
    settings->c_cc[VMIN] = min_chars;
    settings->c_cc[VTIME] = 1; /* 200ms timeout */

    cfsetispeed(settings, B115200);
    cfsetospeed(settings, B115200);

    tcsetattr(fd, TCSANOW, settings); /* apply settings */
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void initialize_queue(Queue *queue) {
    memset(queue, 0, sizeof(Queue));
    queue->mask = sizeof(queue->buff) - 1;
}

typedef struct RxLooperArgs {
    Queue *queue;
    unsigned int *run;
    int fd;
    unsigned int verbose, debug;
} RxLooperArgs;

void *rx_looper(void *ext) {
    RxLooperArgs *args = (RxLooperArgs *) ext;
    Queue *q = args->queue;
    uint8_t buffer[512];
    while (*args->run) {
        /*
         * todo figure out queue math
         * m = 7 h = 0 t = 0 r = 7 f = 7
         * m = 7 h = 7 t = 4 r = 4 f = 4
         * m = 7 h = 4 t = 6 r = 2
         */
        unsigned int fit = (q->mask + q->tail - q->head) & q->mask;
        if (q->head >= q->tail) { fit = 1 + q->mask - q->head; }
        else { fit = q->tail - q->head - 1; }
        if (fit) {
            int n_read = read(args->fd, &q->buff[q->head], fit);
            if (args->verbose) {
                for (int i = 0; i < n_read; ++i) {
                    uint8_t byte = q->buff[(q->head + i) & q->mask];
                    printf("%c(%2.2x) ", byte, byte);
                }
                if (n_read > 0) { printf("\n"); }
            }
            q->head = (q->head + n_read) & q->mask;
        }
        usleep(1000);
    }
    return NULL;
}

uint64_t utime() { /* microseconds */
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    uint64_t t = now.tv_sec * 1000000L;
    t = t + (now.tv_nsec / 1000);
    return t;
}

uint32_t get_time() { /* milliseconds */
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    uint64_t t = now.tv_sec * 1000L;
    t = t + (now.tv_nsec / 1000000L);
    return (uint32_t) t;
}

static int connect(Bootloader *bootloader);
static int get(Bootloader *bootloader);
static int chip_id(Bootloader *bootloader);
static int open_records(Bootloader *bootloader, const char *filename);
static int go(Bootloader *bootloader, uint32_t address);
static int erase(Bootloader *bootloader, unsigned int start_page, unsigned pages);
static int extended_erase(Bootloader *bootloader, unsigned int start_page, unsigned pages);
static int mass_erase(Bootloader *bootloader);
static int extended_mass_erase(Bootloader *bootloader, int bank);
static int read_memory(Bootloader *bootloader, uint32_t address, uint8_t *buff, unsigned int n_bytes);
static int write_memory(Bootloader *bootloader, uint32_t address, uint8_t *buff, unsigned int n_bytes);

int initialize_bootloader(Bootloader *bootloader) {
/* this is where there is a platform-dependent port */
    bootloader->time = get_time;
    bootloader->print = print;
    bootloader->usleep = (void (*)(unsigned int)) usleep;
    bootloader->delay = (void (*)(unsigned int)) sleep;
    bootloader->connect = connect;
    bootloader->get = get;
    bootloader->go = go;
    bootloader->chip_id = chip_id;
    bootloader->open_records = open_records;
    bootloader->mass_erase = mass_erase;
    bootloader->extended_mass_erase = extended_mass_erase;
    bootloader->erase = erase;
    bootloader->extended_erase = extended_erase;
    bootloader->read_memory = read_memory;
    bootloader->write_memory = write_memory;
    return EXIT_SUCCESS;
}

/*
 * -d /dev/ttyUSB5 -i /home/jsvirzi/projects/nauto-prototype/n3-stm/firmware/soc/Debug_CM4/soc_CM4.hex -cpu stm32h745-m4 -erase 0
 * -d /dev/ttyUSB5 -i /home/jsvirzi/projects/nauto-prototype/n3-stm/firmware/soc/Debug_CM7/soc_CM7.hex -cpu stm32h745-m7
 */
int main(int argc, char **argv) {

    char device_name[128], hex_filename[256];
    uint32_t go_address = InvalidAddress;
    uint32_t base_address = InvalidAddress;
    uint32_t read_address = InvalidAddress;
    uint32_t flash_size = 0x10000;
    uint16_t page_size = 128;
    int mass_erase = EraseOptionInvalid;
    uint8_t monitor = 0;

    Bootloader bootloader;
    memset(&bootloader, 0, sizeof(Bootloader));
    initialize_bootloader(&bootloader);
    bootloader.data_buffer = bl_data_buff;
    bootloader.data_buffer_size = sizeof(bl_data_buff);
    bootloader.serial_timeout_ms = 10000; /* 10 seconds = long time */

    device_name[0] = 0;
    hex_filename[0] = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
        } else if (strcmp(argv[i], "-d") == 0) {
            snprintf(device_name, sizeof(device_name), "%s", argv[++i]);
        } else if (strcmp(argv[i], "-i") == 0) {
            snprintf(hex_filename, sizeof(hex_filename), "%s", argv[++i]);
        } else if (strcmp(argv[i], "-page") == 0) {
            page_size = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-flash") == 0) {
            sscanf(argv[++i], "%" PRIu32 "", &flash_size);
        } else if (strcmp(argv[i], "-go") == 0) {
            sscanf(argv[++i], "%x", &go_address);
        } else if (strcmp(argv[i], "-base") == 0) {
            sscanf(argv[++i], "%x", &base_address);
        } else if (strcmp(argv[i], "-read") == 0) {
            sscanf(argv[++i], "%x", &read_address);
        } else if (strcmp(argv[i], "-erase") == 0) {
            mass_erase = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-cpu") == 0) {
            const char *cpu = argv[++i];
            if (strcmp(cpu, "stm32l051") == 0) {
                bootloader.base_address = 0x08000000;
                bootloader.erase_page_size = 128;
                bootloader.write_page_size = 256;
                bootloader.read_page_size = 256;
            } else if (strcmp(cpu, "stm32h745-m4") == 0) {
                bootloader.base_address = 0x08100000;
                bootloader.write_page_size = 256;
                bootloader.read_page_size = 256;
            } else if (strcmp(cpu, "stm32h745-m7") == 0) {
                bootloader.base_address = 0x08000000;
                bootloader.write_page_size = 256;
                bootloader.read_page_size = 256;
            }
        } else if (strcmp(argv[i], "-monitor") == 0) {
            monitor = 1;
        }
    }

    /* TODO necessary to provide base address? included in hex file? */

    printf("programming %" PRIu32 " bytes with page size = %d. go address = %" PRIu32 "\n",
        flash_size, page_size, go_address);

    RxLooperArgs rx_looper_args;
    memset(&rx_looper_args, 0, sizeof(RxLooperArgs));
    Queue rx_queue;
    initialize_queue(&rx_queue);

    unsigned int run = 1;
    rx_looper_args.run = &run;
    pthread_t rx_thread;

    unsigned int parity = monitor ? 0 : 2; /* even parity for bootloader, no parity otherwise */
    bool canonical_mode = false;
    unsigned int min_chars = 0;
    bootloader.fd = initialize_serial_port(device_name, canonical_mode, parity, min_chars);

    bootloader.rx_queue = &rx_queue;
    // bootloader.id = ModuleIdSystem; /* anything, just avoid 0 */
    bootloader.serial_timeout_ms = 1000; /* 100ms */
    // bootloader.page_size = 128;
    // bootloader.base_address = base_address;

    rx_looper_args.run = &run;
    rx_looper_args.fd = bootloader.fd;
    rx_looper_args.queue = &rx_queue;

    pthread_create(&rx_thread, NULL, rx_looper, (void *) &rx_looper_args); /* create thread */

    if (monitor) {
        while (1) {
            while (rx_queue.head != rx_queue.tail) {
                printf("%c", rx_queue.buff[rx_queue.tail]);
                rx_queue.tail = (rx_queue.tail + 1) & rx_queue.mask;
            }
        }
    }

    printf("open serial port. fd = %d\n", bootloader.fd);

    printf("set up BOOTSEL = 1 and toggle RESET...\n");
    getchar();
    printf("start bootload\n");

    bootloader.serial_timeout_ms = 10000; /* long time TODO */

    bootloader.print("enter connect()\n", 0);
    int status = bootloader.connect(&bootloader);
    if (status == EXIT_SUCCESS) { bootloader.print("connect() success\n", 0); }
    else { bootloader.print("failed connect()\n", 0); }

    bootloader.print("enter get()\n", 0);
    status = bootloader.get(&bootloader);
    if (status == EXIT_SUCCESS) { bootloader.print("get() success\n", 0); }
    else { bootloader.print("failed get()\n", 0); }

    bootloader.print("enter chip_id()\n", 0);
    status = bootloader.chip_id(&bootloader);
    if (status == EXIT_SUCCESS) { bootloader.print("chip_id() success\n", 0); }
    else { bootloader.print("failed chip_id()\n", 0); }

    if (read_address != InvalidAddress) {
        uint8_t data[256];
        uint32_t address = read_address;
        status = bootloader.read_memory(&bootloader, read_address, data, sizeof(data));
        if (status != EXIT_SUCCESS) {
            return EXIT_FAILURE;
        }
        printf("data readout: ");
        for (int i = 0; i < sizeof(data); ++i, ++address) {
            if ((i % 32) == 0) {
                printf("\n0x%8.8x: ", address);
            }
            printf("%2.2x ", data[i]);
        }
        printf("\n");
        return EXIT_SUCCESS;
    }

    if (mass_erase != EraseOptionInvalid) {
        bootloader.print("enter (extended) erase()\n", 0);
        status = bootloader.extended_mass_erase(&bootloader, mass_erase);
        // status = bootloader.mass_erase(&bootloader);
        if (status == EXIT_SUCCESS) { bootloader.print("(extended) erase() success\n", 0); }
        else { bootloader.print("failed (extended) erase()\n", 0); }
    }

    bootloader.print("enter memory_flash()\n", 0);

    BootloaderRecord record, work_record;
    memset(&work_record, 0, sizeof(BootloaderRecord));
    memset(&record, 0, sizeof(BootloaderRecord));

    work_record.max_size = bootloader.write_page_size;

    FILE *fp = hexreader_init(hex_filename);
    while (hexreader_next(fp, &work_record, &record)) {

        int status;

        static uint8_t TODO_debug[128];
        status = bootloader.read_memory(&bootloader, record.address, TODO_debug, sizeof(TODO_debug));
        for (int i = 0; i < sizeof(TODO_debug); ++i) {
            if (TODO_debug[i] != 0xff) {
                printf("erasing failed at address = 0x%x\n", record.address);
            }
        }
        status = bootloader.write_memory(&bootloader, record.address, record.payload, record.size);
    }

    if (go_address != InvalidAddress) {
        printf("go(0x%8.8x)\n", go_address);
        status = bootloader.go(&bootloader, go_address);
        if (status == EXIT_SUCCESS) { bootloader.print("go() success\n", 0); }
    }
    run = 0;
    sleep(2);

    pthread_join(rx_thread, NULL);

    return EXIT_SUCCESS;
}

/* returns 0 = success, 1 = fail */

static int transmit_byte(Bootloader *bootloader, uint8_t byte) {
    int n_bytes = write(bootloader->fd, &byte, 1);
    return (n_bytes == 1) ? EXIT_SUCCESS : EXIT_FAILURE;
}

static int transmit_buff(Bootloader *bootloader, uint8_t* buff, unsigned int n) {
    int n_bytes = write(bootloader->fd, buff, n);
    return (n_bytes == n) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/* queue_recv? */
static int recv_buffer(Bootloader *bootloader, uint8_t *bytes, unsigned int n, unsigned int timeout_ms) {
    uint32_t now = bootloader->time();
    uint32_t timeout = now + bootloader->serial_timeout_ms;
    Queue *q = bootloader->rx_queue;
    unsigned int k = 0;
    int status = EXIT_SUCCESS; /* default = success */
    while (k < n) {
        if (q->head != q->tail) {
            bytes[k] = q->buff[q->tail];
            ++k;
            q->tail = (q->tail + 1) & q->mask;
        }
        now = bootloader->time();
        if (now > timeout) {
            if (bootloader->print) { bootloader->print("recv_buffer() timeout\n", 0); }
            status = EXIT_FAILURE;
            break;
        }
    }
    return status;
}

/* find byte in incoming datastream, discarding any other data until byte arrives */
static int recv_find(Bootloader *bootloader, uint8_t byte) {
    uint32_t now = bootloader->time();
    uint32_t timeout = now + bootloader->serial_timeout_ms;;

    Queue *q = bootloader->rx_queue;
    uint8_t rx_byte;
    while (now <= timeout) {
        if (q->head != q->tail) {
            rx_byte = q->buff[q->tail];
            q->tail = (q->tail + 1) & q->mask;
            if (rx_byte == byte) {
                // if (bootloader->print) { bootloader->print("find byte succeeded\n", 0); }
                return EXIT_SUCCESS;
            } else {
                printf("discarding byte 0x%2.2x\n", rx_byte);
            }
        }
        now = bootloader->time();
    }
    if (bootloader->print) { bootloader->print("byte not found\n", 0); }
    flush_queue(bootloader->rx_queue);
    return EXIT_FAILURE; /* fail */
}

static int connect(Bootloader *bootloader) {
    /* connect to chip after bootsel and reset have driven to respective states */
    uint8_t byte = BootloaderInit;
    int status;
    uint8_t *b;
    unsigned int *length;

    status = transmit_byte(bootloader, BootloaderInit);
    if (status != EXIT_SUCCESS) { return status; }

    status = recv_find(bootloader, BootloaderAckByte);
    if (status != EXIT_SUCCESS) { return status; }

    return EXIT_SUCCESS;
}

static int send_command(Bootloader *bootloader, uint8_t command) {
    flush_queue(bootloader->rx_queue);
    uint8_t tx_buff[2];
    tx_buff[0] = command;
    tx_buff[1] = ~command;
    int status = transmit_buff(bootloader, tx_buff, 2);
    if (status != EXIT_SUCCESS) { return EXIT_FAILURE; }
    status = recv_find(bootloader, BootloaderAckByte);
    return status;
};

static int get(Bootloader *bootloader) {
    int status = send_command(bootloader, BootloaderGetCommand);
    if (status != EXIT_SUCCESS) { return status; }

    /* first get the count */
    uint8_t count = 0, version;
    status = queue_recv(bootloader, &count, 1);
    if (status != EXIT_SUCCESS) { return status; }

    if ((count != 0) && (count < sizeof(bootloader->caps))) {
        status = queue_recv(bootloader, &version, 1);
        if (status != EXIT_SUCCESS) { return status; }
    } else {
        printf("get() command. bad count value = %d\n", count);
        return EXIT_FAILURE;
    }

    uint8_t major = (version >> 4) & 0xf;
    uint8_t minor = (version >> 0) & 0xf;

    bootloader->caps_size = count;
    status = queue_recv(bootloader, bootloader->caps, bootloader->caps_size); /* already picked up version, which is part of count */
    if (status != EXIT_SUCCESS) {
        bootloader->print("unable to read GET command response\n", 0);
        return status;
    }

    status = recv_find(bootloader, BootloaderAckByte);
    if (status != EXIT_SUCCESS) { return status; }
    return status;
}

static int chip_id(Bootloader *bootloader) {
    int status = send_command(bootloader, BootloaderGetId);
    if (status != EXIT_SUCCESS) {
        bootloader->print("failed to receive ack for chip id command response\n", 0);
        return status;
    }

    uint8_t count;
    status = queue_recv(bootloader, &count, 1);
    if (status != EXIT_SUCCESS) { return status; }

    char str[64];
    int n_bytes;
    if ((count + 1) > bootloader->data_buffer_size) {
        n_bytes = snprintf(str, sizeof(str), "unexpected count = %d for chip id\n", count);
        bootloader->print(str, n_bytes);
        return EXIT_FAILURE;
    }

    uint8_t *chip_id_str = bootloader->data_buffer;
    status = queue_recv(bootloader, chip_id_str, count + 1);
    if (status != EXIT_SUCCESS) { return status; }

    n_bytes = snprintf(str, sizeof(str), "chip id = %2.2x:%2.2x\n", chip_id_str[0], chip_id_str[1]);
    bootloader->print(str, n_bytes);

    return EXIT_SUCCESS;
}

static int open_records(Bootloader *bootloader, const char *filename) {
    int fd = open(filename, O_RDONLY, S_IREAD);
    return fd;
}

static int go(Bootloader *bootloader, uint32_t address) {
    int status = send_command(bootloader, BootloaderGo);
    if (status != EXIT_SUCCESS) { return status; }
    uint8_t record_address[5]; /* 4 bytes + checksum */
    record_address[0] = (address >> 0x18) & 0xff;
    record_address[1] = (address >> 0x10) & 0xff;
    record_address[2] = (address >> 0x08) & 0xff;
    record_address[3] = (address >> 0x00) & 0xff;
    uint8_t checksum = 0;
    checksum = checksum ^ record_address[0];
    checksum = checksum ^ record_address[1];
    checksum = checksum ^ record_address[2];
    checksum = checksum ^ record_address[3];
    record_address[4] = checksum;
    status = transmit_buff(bootloader, record_address, 5);
    if (status != EXIT_SUCCESS) { return status; }
    status = recv_find(bootloader, BootloaderAckByte);
    if (status != EXIT_SUCCESS) { return status; }
    return status;
}

static int mass_erase(Bootloader *bootloader) {
    int status = send_command(bootloader, BootloaderErase);
    if (status != EXIT_SUCCESS) {
        bootloader->print("failed to send erase command\n", 0);
        return status;
    }
    /* not exactly a command, but pattern for global erase coincides with sending a command */
    uint8_t buff[2];
    buff[0] = 0xff;
    buff[1] = 0x00;
    transmit_buff(bootloader, buff, 2);
    printf("TODO erase\n");
    status = recv_find(bootloader, BootloaderAckByte);
    return status;
}

static int extended_mass_erase(Bootloader *bootloader, int bank) {
    int status = send_command(bootloader, BootloaderExtendedErase);
    if (status != EXIT_SUCCESS) {
        bootloader->print("failed to send extended erase command\n", 0);
        return status;
    }
    flush_queue(bootloader);

    uint8_t buff[3];
    switch (bank) {
    case EraseOptionGlobal:
        buff[0] = 0xff;
        buff[1] = 0xff;
        buff[2] = 0x00;
        break;
    case EraseOptionBank1:
        buff[0] = 0xff;
        buff[1] = 0xfe;
        buff[2] = 0x01;
        break;
    case EraseOptionBank2:
        buff[0] = 0xff;
        buff[1] = 0xfd;
        buff[2] = 0x02;
        break;
    }
    status = transmit_buff(bootloader, buff, 3);
    if (status != EXIT_SUCCESS) {
        printf("extended_mass_erase(%d). error sending options\n", bank);
        return EXIT_FAILURE;
    }

    status = recv_find(bootloader, BootloaderAckByte);
    return status;
}

static int erase(Bootloader *bootloader, unsigned int start_page, unsigned int pages) {
    int status = send_command(bootloader, BootloaderErase);
    if (status != EXIT_SUCCESS) {
        bootloader->print("failed to send erase command\n", 0);
        return status;
    }
    /* not exactly a command, but pattern for global erase coincides with sending a command */
    uint8_t buff[2];
    buff[0] = 0xff;
    buff[1] = 0x00;
    transmit_buff(bootloader, buff, 2);
    status = recv_find(bootloader, BootloaderAckByte);
    return status;
}

static int extended_erase(Bootloader *bootloader, unsigned int start_page, unsigned int pages) {
    int status = send_command(bootloader, BootloaderExtendedErase);
    if (status != EXIT_SUCCESS) {
        bootloader->print("failed to send extended erase command\n", 0);
        return status;
    }

    unsigned int idx = 0;
    uint8_t checksum = 0;
    uint8_t *buff = erase_pages_buff;
    buff[idx] = ((pages-1) >> 8) & 0xff;
    checksum = checksum ^ buff[idx++];
    buff[idx] = ((pages-1) >> 0) & 0xff;
    checksum = checksum ^ buff[idx++];
    unsigned int page = start_page;
    for (int i = 0; i < pages; ++i, ++page) {
        buff[idx] = (page >> 8) & 0xff;
        checksum = checksum ^ buff[idx++];
        buff[idx] = (page >> 0) & 0xff;
        checksum = checksum ^ buff[idx++];
    }
    buff[idx++] = checksum;
    transmit_buff(bootloader, buff, idx);

    status = recv_find(bootloader, BootloaderAckByte);
    return status;
}

static int read_memory(Bootloader *bootloader, uint32_t address, uint8_t *buff, unsigned int n_bytes) {
    int status = send_command(bootloader, BootloaderReadMemory);
    if (status != EXIT_SUCCESS) {
        bootloader->print("failed to send extended erase command\n", 0);
        return EXIT_FAILURE;
    }

    uint8_t record_address[5]; /* 4 bytes + checksum */
    record_address[0] = (address >> 0x18) & 0xff;
    record_address[1] = (address >> 0x10) & 0xff;
    record_address[2] = (address >> 0x08) & 0xff;
    record_address[3] = (address >> 0x00) & 0xff;
    uint8_t checksum = 0;
    checksum = checksum ^ record_address[0];
    checksum = checksum ^ record_address[1];
    checksum = checksum ^ record_address[2];
    checksum = checksum ^ record_address[3];
    record_address[4] = checksum;
    status = transmit_buff(bootloader, record_address, 5);
    if (status != EXIT_SUCCESS) { return status; }
    status = recv_find(bootloader, BootloaderAckByte);
    if (status != EXIT_SUCCESS) { return status; }

    /* hijack record_address for another purpose TODO */
    record_address[0] = n_bytes - 1;
    record_address[1] = ~record_address[0];
    status = transmit_buff(bootloader, record_address, 2);
    if (status != EXIT_SUCCESS) { return status; }
    status = recv_find(bootloader, BootloaderAckByte);
    if (status != EXIT_SUCCESS) { return status; }

    status = queue_recv(bootloader, buff, n_bytes);
    if (status != EXIT_SUCCESS) {
        printf("error reading address 0x%8.8x\n", address);
        return status;
    }

    return EXIT_SUCCESS;
}

// int write_memory(Bootloader *bootloader, BootloaderRecord *record) {
static int write_memory(Bootloader *bootloader, uint32_t address, uint8_t *data, unsigned int n_bytes) {
    int status;

    uint8_t record_address[5]; /* 4 bytes + checksum */
    // uint32_t address = record->segment_address + record->base_address;
    record_address[0] = (address >> 0x18) & 0xff;
    record_address[1] = (address >> 0x10) & 0xff;
    record_address[2] = (address >> 0x08) & 0xff;
    record_address[3] = (address >> 0x00) & 0xff;
    uint8_t checksum = 0;
    checksum = checksum ^ record_address[0];
    checksum = checksum ^ record_address[1];
    checksum = checksum ^ record_address[2];
    checksum = checksum ^ record_address[3];
    record_address[4] = checksum;

//    if (mass_erase == EraseOptionInvalid) { /* we did not perform a mass erase, so erase individually each "sector" */
//        unsigned int start_page = (address - bootloader.base_address) / bootloader.erase_page_size;
//        unsigned int pages = 1 + (record.size - 1) / bootloader.erase_page_size;
//        int status = bootloader.extended_erase(&bootloader, start_page, pages);
//        if (status != EXIT_SUCCESS) {
//            printf("failed erase\n");
//            return status;
//        }
//    }

    /* at this point all systems are a go */
    printf("loading memory at %8.8x size = %d\n", address, n_bytes);
    status = send_command(bootloader, BootloaderWriteMemory);
    if (status != EXIT_SUCCESS) {
        printf("failed issue command\n");
        return status;
    }

    uint8_t *src = data;
    uint8_t *dst = bootloader->data_buffer;
    // memcpy(&p[1], data, n_bytes);
    // data[n_bytes + 1] = record->checksum;
    uint8_t byte = n_bytes - 1;
    *dst++ = byte; /* first byte used to indicate number of bytes */
    checksum = byte; /* checksum is taken over entire payload, including count */
    for (unsigned int i = 0; i < n_bytes; ++i) {
        byte = data[i];
        checksum = checksum ^ byte;
        *dst++ = byte;
    }
    *dst = checksum;

    flush_queue(bootloader); /* should not be stragglers, but just in case */
    status = transmit_buff(bootloader, record_address, 5);
    if (status != EXIT_SUCCESS) {
        printf("failed issue address\n");
        return EXIT_FAILURE;
    }
    status = recv_find(bootloader, BootloaderAckByte);
    if (status != EXIT_SUCCESS) {
        printf("failed issue address ack\n");
        return EXIT_FAILURE;
    }

    flush_queue(bootloader->rx_queue); /* should not be stragglers, but you can't tell these days */
    status = transmit_buff(bootloader, bootloader->data_buffer, n_bytes + 2);
    if (status != EXIT_SUCCESS) { return status; }
    status = recv_find(bootloader, BootloaderAckByte);
    if (status != EXIT_SUCCESS) { return status; }

#if 1
    memset(erase_pages_buff, 0, n_bytes);
    status = bootloader->read_memory(bootloader, address, erase_pages_buff, n_bytes);
    if (status != EXIT_SUCCESS) {
        printf("failed to read memory at address = 0x%8.8x\n", address);
        return status;
    }

    checksum = 0; /* hijack for verifying data */
    for (int i = 0; i < n_bytes; ++i) {
        uint8_t byte = data[i] ^ erase_pages_buff[i];
        if (byte) { printf("poor you %d. %x vs %x\n", i, data[i], erase_pages_buff[i]); }
        checksum = checksum | byte;
    }

    if (checksum == 0) {
        printf("wrote and verified memory at address = 0x%8.8x\n", address);
    }
#endif

    if (checksum) { return EXIT_FAILURE; }

}