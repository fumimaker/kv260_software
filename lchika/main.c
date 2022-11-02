#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define GPIO_NAME axigpio-uio

static inline void regs_write32(void *addr, uint32_t data) {
    volatile uint32_t *regs_addr = (uint32_t *)(addr);
    *regs_addr = data;
}

static inline uint32_t regs_read32(void *addr) {
    volatile uint32_t *regs_addr = (uint32_t *)(addr);
    return *regs_addr;
}

int main(int argc, char **argv)
{
    printf("Hello World!\n");

    int *address;    /* GPIOレジスタへの仮想アドレス(ユーザ空間) */
    int fd;

    /* メモリアクセス用のデバイスファイルを開く */
    if ((fd = open("/dev/uio4", O_RDWR | O_SYNC)) < 0) {
        perror("open");
        return -1;
    }

    /* ARM(CPU)から見た物理アドレス → 仮想アドレスへのマッピング */
    address = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    // address[0] : DATA
    // address[1] : TRI-STATE(OUTPUT)
    if (address == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return -1;
    }
    int counter = 0;
    // address[0] = 0;
    // usleep(1*1000*1000);
    // address[0] = 0x0F;
    // usleep(1 * 1000 * 1000);

    while(1) {
        if(counter==8) {
            counter = 0;
        }
        address[0] = counter;
        printf("%d\n", address[0]);
        usleep(1*1000*500); //500ms
        counter++;
        // address[0] = 0x00;
        // usleep(1*1000*1000);
    }

    /* 使い終わったリソースを解放する */
    munmap((void*)address, 0x1000);
    close(fd);

    return 0;
}