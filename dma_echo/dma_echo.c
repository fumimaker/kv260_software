// refered: https://www.fugafuga.org/fpga/fpga_soc_linux_xilinx_dma.html
// refered:
// https://github.com/ikwzm/ZYBO_UIO_IRQ_SAMPLE/blob/master/c-sample/sample1.c
#include <stdint.h>
#include <stdio.h>
// #include <sysexits.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>

#include "xcounteraxistream.h"
#include "xv_tpg.h"

#define TPG_NAME "tpg-uio"
#define OUTPUTIF_NAME "outputif-uio"
#define UDMABUF_NAME "udmabuf1"
#define DMA_UIO "/dev/uio4"
#define COUNTER_NAME "counter-uio"

#define S2MM_CONTROL_REGISTER 0x30
#define S2MM_STATUS_REGISTER 0x34
#define S2MM_DESTINATION_ADDRESS 0x48
#define S2MM_DESTINATION_ADDRESS_MSB 0x4C
#define S2MM_LENGTH 0x58

#define PUMP_CR_RS (1 << 0)
#define PUMP_CR_RESET (1 << 2)
#define PUMP_CR_IOC_IrqEn (1 << 12)
#define PUMP_CR_ERR_IrqEn (1 << 14)

#define PUMP_SR_HALTED (1 << 0)
#define PUMP_SR_IDLE (1 << 1)
#define PUMP_SR_DMA_ERR_Irq (1 << 4)
#define PUMP_SR_IOC_Irq (1 << 12)
#define PUMP_SR_ERR_Irq (1 << 14)

#define WIDTH 1280
#define HEIGHT 720

#define check_size (500*1000)

unsigned int reg_set(unsigned int* dma_virtual_address, int offset,
                     unsigned int value) {
    dma_virtual_address[offset >> 2] = value;
}

unsigned int reg_get(unsigned int* dma_virtual_address,
                                   int offset) {
    return dma_virtual_address[offset >> 2];
}

void dma_s2mm_status(unsigned int* dma_virtual_address) {
    unsigned int status = reg_get(dma_virtual_address, S2MM_STATUS_REGISTER);
    printf("Stream to memory-mapped status (0x%08x@0x%02x):", status,
           S2MM_STATUS_REGISTER);
    if (status & 0x00000001)
        printf(" halted");
    else
        printf(" running");
    if (status & 0x00000002) printf(" idle");
    if (status & 0x00000008) printf(" SGIncld");
    if (status & 0x00000010) printf(" DMAIntErr");
    if (status & 0x00000020) printf(" DMASlvErr");
    if (status & 0x00000040) printf(" DMADecErr");
    if (status & 0x00000100) printf(" SGIntErr");
    if (status & 0x00000200) printf(" SGSlvErr");
    if (status & 0x00000400) printf(" SGDecErr");
    if (status & 0x00001000) printf(" IOC_Irq");
    if (status & 0x00002000) printf(" Dly_Irq");
    if (status & 0x00004000) printf(" Err_Irq");
    printf("\n");
}

int dma_s2mm_sync(unsigned int* dma_virtual_address) {
    unsigned int s2mm_status =
        reg_get(dma_virtual_address, S2MM_STATUS_REGISTER);
    while (!(s2mm_status & 1 << 12) || !(s2mm_status & 1 << 1)) {
        dma_s2mm_status(dma_virtual_address);
        s2mm_status = reg_get(dma_virtual_address, S2MM_STATUS_REGISTER);
    }
}

struct udmabuf {
    char name[128];
    int file;
    unsigned char* buf;
    unsigned int buf_size;
    unsigned long phys_addr;
    unsigned long debug_vma;
    unsigned long sync_mode;
};

int udmabuf_open(struct udmabuf* udmabuf, const char* name) {
    char file_name[1024];
    int fd;
    unsigned char attr[1024];

    strcpy(udmabuf->name, name);
    udmabuf->file = -1;

    sprintf(file_name, "/sys/class/u-dma-buf/%s/phys_addr", name);
    if ((fd = open(file_name, O_RDONLY)) == -1) {
        printf("Can not open %s\n", file_name);
        return (-1);
    }
    read(fd, (void*)attr, 1024);
    sscanf(attr, "%x", &udmabuf->phys_addr);
    close(fd);

    sprintf(file_name, "/sys/class/u-dma-buf/%s/size", name);
    if ((fd = open(file_name, O_RDONLY)) == -1) {
        printf("Can not open %s\n", file_name);
        return (-1);
    }
    read(fd, (void*)attr, 1024);
    sscanf(attr, "%d", &udmabuf->buf_size);
    close(fd);

    sprintf(file_name, "/dev/%s", name);
    if ((udmabuf->file = open(file_name, O_RDWR | O_SYNC)) == -1) {
        printf("Can not open %s\n", file_name);
        return (-1);
    }

    udmabuf->buf = mmap(NULL, udmabuf->buf_size, PROT_READ | PROT_WRITE,
                       MAP_SHARED, udmabuf->file, 0);
    udmabuf->debug_vma = 0;
    udmabuf->sync_mode = 1;

    return 0;
}

int udmabuf_close(struct udmabuf* udmabuf) {
    if (udmabuf->file < 0) return -1;

    close(udmabuf->file);
    udmabuf->file = -1;
    return 0;
}
int uio_irq_on(int uio_fd) {
    unsigned int irq_on = 1;
    write(uio_fd, &irq_on, sizeof(irq_on));
}

int uio_wait_irq(int uio_fd) {
    unsigned int count = 0;
    return read(uio_fd, &count, sizeof(count));
}
int main(){
    struct udmabuf outbuf;
    XV_tpg tpgInst;
    XCounteraxistream counterInst;
    uint32_t *reg;
    int uio_fd, tpg_fd;
    int status;

    if ((uio_fd = open("/dev/uio7", O_RDWR|O_SYNC)) == -1) {
        printf("Can not open /dev/uio7\n");
        exit(1);
    }
    reg = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, uio_fd, 0);

    if (udmabuf_open(&outbuf, "udmabuf1") == -1) {
        printf("Can not open udmabuf1\n");
        exit(1);
    }
    for (int i = 0; i < check_size; i++) {
        outbuf.buf[i] = 0x00;
    }


    status = XV_tpg_Initialize(&tpgInst, TPG_NAME);
    if (status != XST_SUCCESS) {
        printf("TPG configuration failed\r\n");
        exit(1);
    }
    XV_tpg_Set_height(&tpgInst, HEIGHT);
    XV_tpg_Set_width(&tpgInst, WIDTH);
    if (XV_tpg_Get_height(&tpgInst) == HEIGHT &&
        XV_tpg_Get_width(&tpgInst) == WIDTH) {
        printf("[ OK ] ");
    } else {
        printf("[ NG ] ");
    }
    printf("TPG Height:%u, Width:%u\n", XV_tpg_Get_height(&tpgInst),
           XV_tpg_Get_width(&tpgInst));
    XV_tpg_Set_colorFormat(&tpgInst, 0x0);  // RGB
    XV_tpg_Set_bckgndId(&tpgInst, 0x11);
    XV_tpg_EnableAutoRestart(&tpgInst);




    status = XCounteraxistream_Initialize(&counterInst, COUNTER_NAME);
    if (status != XST_SUCCESS) {
        printf("counter inst failded\n");
        exit(1);
    } else {
        printf("counter inst done\n");
    }
    XCounteraxistream_Set_height_V(&counterInst, HEIGHT);
    XCounteraxistream_Set_width_V(&counterInst, WIDTH);
    if (XCounteraxistream_Get_height_V(&counterInst) == HEIGHT &&
        XCounteraxistream_Get_width_V(&counterInst) == WIDTH) {
        printf("[ OK ] ");
    } else {
        printf("[ NG ] ");
    }
    printf("CounterAXIStream Height:%u, Width:%u\n",
           XCounteraxistream_Get_height_V(&counterInst),
           XCounteraxistream_Get_width_V(&counterInst));

    XCounteraxistream_EnableAutoRestart(&counterInst);





    printf("Resetting DMA\n");
    reg_set(reg, S2MM_CONTROL_REGISTER, 4);//reset
    dma_s2mm_status(reg);
    printf("Halting DMA\n");
    reg_set(reg, S2MM_CONTROL_REGISTER, 0); //stop(halt)

    printf("Writing destination address\n");
    reg_set(reg, S2MM_DESTINATION_ADDRESS, (uint32_t)outbuf.phys_addr);//
    // Write destination address
    reg_set(reg, S2MM_DESTINATION_ADDRESS_MSB,(uint32_t)(outbuf.phys_addr >> 32));  // Write destination address
    dma_s2mm_status(reg);

    printf("Starting S2MM channel with all interrupts masked...\n");
    reg_set(reg, S2MM_CONTROL_REGISTER, 0xf001); //start & all IRQ Enable
    dma_s2mm_status(reg);

    printf("Writing S2MM transfer length...\n");
    reg_set(reg, S2MM_LENGTH, check_size);  // write length --> start
    dma_s2mm_status(reg);

    printf("Waiting for S2MM finish...\n");
    XCounteraxistream_Start(&counterInst);
    XV_tpg_Start(&tpgInst);

    uint32_t dma_addr = 0;
    #define cnt (90*1)
    unsigned int addrbuf[cnt];
    for(int i=0; i<cnt; i++){
        reg_set(reg, S2MM_DESTINATION_ADDRESS, (uint32_t)outbuf.phys_addr + dma_addr);
        reg_set(reg, S2MM_DESTINATION_ADDRESS_MSB,
                (uint32_t)(((outbuf.phys_addr + dma_addr) >> 32)));
        reg_set(reg, S2MM_CONTROL_REGISTER, 0x5001);
        reg_set(reg, S2MM_LENGTH, check_size);
        reg_set(reg, S2MM_STATUS_REGISTER, 0x5000);
        uio_irq_on(uio_fd);
        if (uio_wait_irq(uio_fd) == -1) {
            printf("uio_wait_irq error\n");
        }
        uint32_t addr = reg_get(reg, S2MM_LENGTH);
        dma_addr = addr + dma_addr;
        addrbuf[i] = addr;
    }

    for(int i=0; i<cnt; i++){

    }

    int address=0;
    for(int i=0; i<cnt; i++){
        char moji[32];
        sprintf(moji, "binout/%03d.bin", i);
        FILE* fp = fopen(moji, "wb");
        for (int j = 0; j < addrbuf[i]; j = j + 4) {
            unsigned char buffer[4];
            buffer[0] = outbuf.buf[3 + j + address];
            buffer[1] = outbuf.buf[2 + j + address];
            buffer[2] = outbuf.buf[1 + j + address];
            buffer[3] = outbuf.buf[0 + j + address];
            int n = fwrite(buffer, sizeof(unsigned char), 4, fp);
        }
        printf("write:%d addrbuf:%u\n", i, addrbuf[i]);
        address = address + addrbuf[i];
        fclose(fp);
    }

    // //dma_s2mm_sync(reg);
    // If this locks up make sure all memory ranges are
    // assigned under Address Editor!
    dma_s2mm_status(reg);

    unsigned int len = reg_get(reg, S2MM_LENGTH);

    printf("total filesize:%u\n", address);

    udmabuf_close(&outbuf);
    close(uio_fd);
    return 0;
}