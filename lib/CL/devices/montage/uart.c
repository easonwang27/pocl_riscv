#include <stdio.h>        //标准输入输出,如printf、scanf以及文件操作
#include <stdint.h>
#include <stdlib.h>        //标准库头文件，定义了五种类型、一些宏和通用工具函数
#include <unistd.h>        //定义 read write close lseek 等Unix标准函数
#include <sys/types.h>    //定义数据类型，如 ssiz e_t off_t 等
#include <sys/stat.h>    //文件状态
#include <fcntl.h>        //文件控制定义
#include <termios.h>    //终端I/O
#include <errno.h>        //与全局变量 errno 相关的定义
#include <getopt.h>        //处理命令行参数
#include <string.h>        //字符串操作
#include <time.h>        //时间
#include<pthread.h>
#include <sys/select.h>    //select函数

#include "uart.h"

#define FALSE -1
#define TRUE   0

#define RED    "\033[0;32;31m"
#define GRREEN "\033[0;32;32m"

#define BLUE   "\033[0;32;34m"
#define NONE   "\033[0m"


POCL_DEVTYPE pocl_dev;
/**
*@brief   设置串口数据位，停止位和效验位
*@param fd     类型 int 打开的串口文件句柄*
*@param databits 类型 int 数据位   取值 为 7 或者8*
*@param stopbits 类型 int 停止位   取值为 1 或者2*
*@param parity 类型 int 效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;
if ( tcgetattr( fd,&options) != 0)
{
    perror("SetupSerial 1");
    return(FALSE);
}
options.c_cflag &= ~CSIZE;
switch (databits) /*设置数据位数*/
{
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return (FALSE);
    }
switch (parity)
    {
    case 'n':
    case 'N':
//        options.c_cflag &= ~PARENB;   /* Clear parity enable */
//        options.c_iflag &= ~INPCK;     /* Enable parity checking */
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input*/
        options.c_oflag &= ~OPOST;   /*Output*/
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/ 
        options.c_iflag |= INPCK;             /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;     /* Enable parity */
        options.c_cflag &= ~PARODD;   /* 转换为偶效验*/ 
        options.c_iflag |= INPCK;       /* Disnable parity checking */
        break;
    case 'S':
    case 's': /*as no parity*/
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        return (FALSE);
        }
/* 设置停止位*/   
switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        return (FALSE);
    }
/* Set input parity option */
if ((parity != 'n')&&(parity != 'N'))
        options.c_iflag |= INPCK;

    options.c_cc[VTIME] = 5; // 0.5 seconds
    options.c_cc[VMIN] = 1;

    options.c_cflag &= ~HUPCL;
    options.c_iflag &= ~INPCK;
    options.c_iflag |= IGNBRK;
    options.c_iflag &= ~ICRNL;
    options.c_iflag &= ~IXON;
    options.c_lflag &= ~IEXTEN;
    options.c_lflag &= ~ECHOK;
    options.c_lflag &= ~ECHOCTL;
    options.c_lflag &= ~ECHOKE;
    options.c_oflag &= ~ONLCR;
    
    tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    
return (TRUE);
}


void set_speed(int fd, int speed)
{
    unsigned int   i; 
    int   status; 
    int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200, 300};
    struct termios   Opt;

    if(tcgetattr(fd,&Opt)!=0)  
    {  
        perror("SetupSerial 1");      
        return;  
    }  

    for ( i= 0; i < sizeof(speed_arr) / sizeof(int); i++) { 
    
    if (speed == name_arr[i]) {    
        tcflush(fd, TCIOFLUSH);     
        cfsetispeed(&Opt, speed_arr[i]); 
        cfsetospeed(&Opt, speed_arr[i]);   
        status = tcsetattr(fd, TCSANOW, &Opt); 
    if (status != 0) {    
            perror("tcsetattr fd1"); 
            return;     
        }    
        tcflush(fd,TCIOFLUSH);   
        } 
        }
    //激活配置 (将修改后的termios数据设置到串口中）  
    if (tcsetattr(fd,TCSANOW,&Opt) != 0)    
    {  
        perror("com set error!\n");    
        return ;
    }  
}



void *monitor_serial_readable(void *arg)
{

    int rd_num,i,nread=0;
    fd_set rset;
    struct timeval timeout;
    uint8_t buf[2048] = {0};
    int read_status = 0;
    int dest_cnt = 0;
    int transfer_started=0;
  
    while(1)
    {   
        rd_num=read(pocl_dev.fd,buf,sizeof(buf));
        timeout.tv_sec=1;
        timeout.tv_usec=0;
        if(rd_num>0)
        {
            printf("%d(interval%4.3fs)：we can read \n\"%s\" \n from the uart interface,total:%d characters.\n",++i,timeout.tv_sec+timeout.tv_usec*0.000001,buf,rd_num);
            transfer_started=1; 
        }
        else{
            printf("%d(interval%4.3fs)：read fail! rd_num=%d.This data transfer :%s\n",++i,timeout.tv_sec+timeout.tv_usec*0.000001,rd_num,transfer_started==1?"Is over":"Not yet started");
        
          }
        select(0,NULL,NULL,NULL,&timeout);/*精确定时*/

        if(sizeof(buf) == 2048)
              break;
      
    }
} 

void print_frame(const char *desc,uint8_t *buf,int size)
{
    int i;
    printf(RED"[%s] [LEN=%d]"NONE,desc,size);
    for(i=0; i<size; i++)
    {
        printf(BLUE"[%.2x]"NONE,buf[i]);
    }
    printf("\n");
}

//getCompleteFrame(buf,nread,dest,&dest_cnt,&read_status);
void getCompleteFrame(uint8_t *inBuf,int inCnt,uint8_t *outBuf,int destCnt,int *readStatus)
{
    int i;

    for(i = 0; i< inCnt; i++)
    {
        if(inBuf[i] == 0xe2)//header
        {
            outBuf[(destCnt)++] = inBuf[i];
            *readStatus = 1;
             //print_frame("header",outBuf,destCnt);
            // continue;
        }
        if(*readStatus == 1)//body
        {
            outBuf[(destCnt)++] = inBuf[i];    
            print_frame("body",outBuf,destCnt);
        }
        if(destCnt == outBuf[1])//tail
        {
            print_frame("tail",outBuf,destCnt);
            *readStatus = 2;
            destCnt = 0;
            memset(outBuf,-1,sizeof(outBuf));
            memset(inBuf,0,sizeof(inBuf));
            continue;
        }
    }

}

int pocl_serial_send(int fd, uint8_t *send_buf, int data_len)
{
    ssize_t ret = 0;

    int i = 0;
    int j = 0;
    sleep(1);
    printf("data_len = %d\n",data_len);

   for(i = 0; i <data_len;i++)
   {  j++;
      printf("%02x ",(uint8_t)send_buf[i]);
      if(j%16==0)
          printf("\n");
   }
   printf("\n");

    ret = write(fd, send_buf, data_len);
    if (ret == data_len)
    {
        
        return ret;
    }
    else
    {
        printf("write device error\n");
        tcflush(fd,TCOFLUSH);
        return -1;
    }
}


int pocl_uart_config()
{
    int flag,rd_num=0;
    pthread_t wtid;
    struct termios term;
    struct timeval timeout;
    speed_t baud_rate_i,baud_rate_o;

    uint8_t recv_buf[1024] = {0};
    uint8_t dest[1024]={0};

    int fd ;
    int dest_cnt = 0;

    fd=open(DEV_NAME,O_RDWR|O_NONBLOCK);
    if (-1 == fd)
    { /*设置数据位数*/
			perror("Can't Open Serial Port");
			return -1;
	}
    pocl_dev.fd = fd;
    printf("pocl_dev->fd = %d\n",pocl_dev.fd);
  //  pocl_uartdev
    flag=tcgetattr(fd,&term);/*取得终端设备fd的属性，存放在termios类型的结构体term中*/
    baud_rate_i=cfgetispeed(&term);/*从term中读取输入波特率*/
    baud_rate_o=cfgetospeed(&term);/*从term中读取输出波特率*/
    //printf("baudrate of in is:%d,baudrate of out is %d,fd=%d\n",baud_rate_i,baud_rate_o,fd);/*打印设置之前的波特率，目的是方便和设置之后的波特率对比*/

    set_speed(fd,38400);

    flag=tcgetattr(fd,&term);
    baud_rate_i=cfgetispeed(&term);
    baud_rate_o=cfgetospeed(&term);
    //printf("baudrate of out is:%d,baudrate of out is %d,fd=%d\n",baud_rate_i,baud_rate_o,fd);/*打印设置之前的波特率，目的是方便和设置之后的波特率对比*/

   if (set_Parity(fd,8,1,'N')== FALSE)
    {
        printf("Set Parity Error\n");
        exit(1);
    }
}

void pocl_send_data(uint8_t *send_buf,int data_len)
{
    pocl_serial_send(pocl_dev.fd, send_buf,data_len);
}


void pocl_uart_dev()
{
     close(pocl_dev.fd);
}