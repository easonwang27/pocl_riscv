
#ifndef UART_H
#define UART_H

#include <assert.h>
#include <stdio.h>

#define DEV_NAME    "/dev/ttyUSB0"    ///< 串口设备
#define MAX_DLFW_PAGE_SIZE 1024
#define ARG_NUM 3

//add for montage
enum host_cmd{
    POCL_PC = 0xe0,
    POCL_ELF,
    POCL_ARG,
    POCL_NAME,
    POCL_MOLLOC,
};

typedef struct _pocl_uart_data {
  uint8_t  cmd;
  uint16_t len;
  const void*  uart_data;
} pocl_uart_data;


typedef struct uart_dev
{
     int fd;
    uint8_t kernel_arg[MAX_DLFW_PAGE_SIZE];
    uint8_t kernel_name[MAX_DLFW_PAGE_SIZE];
    uint8_t kernel_wg[MAX_DLFW_PAGE_SIZE];
    uint8_t *kernel_elf;
    
}POCL_DEVTYPE; 


int pocl_uart_config();
void set_speed(int, int);       
int  set_Parity(int,int,int,int);
void print_frame(const char *desc,uint8_t *buf,int size);
void getCompleteFrame(uint8_t *inBuf,int inCnt,uint8_t *outBuf,int destCnt,int *readStatus);
void *monitor_serial_readable(void *arg);
int pocl_serial_send(int fd, uint8_t *send_buf, int data_len);
void pocl_send_data(uint8_t *send_buf,int data_len);

#endif