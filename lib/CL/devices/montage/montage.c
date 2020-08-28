/* montage.c - a minimalistic pocl device driver layer implementation

   Copyright (c) 2011-2013 Universidad Rey Juan Carlos and
                 2011-2019 Pekka Jääskeläinen

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#include "config.h"
#include "config2.h"
#include "montage.h"
#include "cpuinfo.h"
#include "topology/pocl_topology.h"
#include "common.h"
#include "utlist.h"
#include "devices.h"
#include "pocl_util.h"

#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <utlist.h>

#include "pocl_cache.h"
#include "pocl_timing.h"
#include "pocl_file_util.h"
#include "pocl_workgroup_func.h"

#ifdef OCS_AVAILABLE
#include "pocl_llvm.h"
#endif

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

#define FALSE -1
#define TRUE   0

#define RED    "\033[0;32;31m"
#define GRREEN "\033[0;32;32m"

#define BLUE   "\033[0;32;34m"
#define NONE   "\033[0m"


#define DEV_NAME    "/dev/ttyUSB0"    ///< 串口设备


#ifndef HAVE_LIBDL
#error montage driver requires DL library
#endif
int fd;
/* default WG size in each dimension & total WG size.
 * this should be reasonable for CPU */
#define DEFAULT_WG_SIZE 4096
#define MAX_DLFW_PAGE_SIZE 1024

void set_speed(int, int);       
int set_Parity(int,int,int,int);
void print_frame(const char *desc,uint8_t *buf,int size);
void getCompleteFrame(uint8_t *inBuf,int inCnt,uint8_t *outBuf,int destCnt,int *readStatus);
void *monitor_serial_readable(void *arg);
int pocl_serial_send(int fd, char *send_buf, int data_len);

struct data {
  /* Currently loaded kernel. */
  cl_kernel current_kernel;

  /* List of commands ready to be executed */
  _cl_command_node * volatile ready_list;
  /* List of commands not yet ready to be executed */
  _cl_command_node * volatile command_list;
  /* Lock for command list related operations */
  pocl_lock_t cq_lock;
  /* printf buffer */
  void *printf_buffer;
};
// add for pocl

static const cl_image_format supported_image_formats[] = {
    { CL_A, CL_SNORM_INT8 },
    { CL_A, CL_SNORM_INT16 },
    { CL_A, CL_UNORM_INT8 },
    { CL_A, CL_UNORM_INT16 },
    { CL_A, CL_SIGNED_INT8 },
    { CL_A, CL_SIGNED_INT16 },
    { CL_A, CL_SIGNED_INT32 },
    { CL_A, CL_UNSIGNED_INT8 },
    { CL_A, CL_UNSIGNED_INT16 },
    { CL_A, CL_UNSIGNED_INT32 },
    { CL_A, CL_FLOAT },
    { CL_RGBA, CL_SNORM_INT8 },
    { CL_RGBA, CL_SNORM_INT16 },
    { CL_RGBA, CL_UNORM_INT8 },
    { CL_RGBA, CL_UNORM_INT16 },
    { CL_RGBA, CL_SIGNED_INT8 },
    { CL_RGBA, CL_SIGNED_INT16 },
    { CL_RGBA, CL_SIGNED_INT32 },
    { CL_RGBA, CL_UNSIGNED_INT8 },
    { CL_RGBA, CL_UNSIGNED_INT16 },
    { CL_RGBA, CL_UNSIGNED_INT32 },
    { CL_RGBA, CL_HALF_FLOAT },
    { CL_RGBA, CL_FLOAT },
    { CL_ARGB, CL_SNORM_INT8 },
    { CL_ARGB, CL_UNORM_INT8 },
    { CL_ARGB, CL_SIGNED_INT8 },
    { CL_ARGB, CL_UNSIGNED_INT8 },
    { CL_BGRA, CL_SNORM_INT8 },
    { CL_BGRA, CL_UNORM_INT8 },
    { CL_BGRA, CL_SIGNED_INT8 },
    { CL_BGRA, CL_UNSIGNED_INT8 }
 };

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


void *monitor_serial_readable(void *arg)
{

    int rd_num,i,nread=0;
    fd_set rset;
    struct timeval timeout;
    uint8_t buf[1024] = {0};
    uint8_t dest[1024]={0};
    int read_status = 0;
    int dest_cnt = 0;
     int transfer_started=0;
    while(1)
    {   
        rd_num=read(fd,buf,sizeof(buf));
        timeout.tv_sec=0;
        timeout.tv_usec=200000;
        if(rd_num>0)
        {
            printf("%d(间隔%4.3fs)：we can read \"%s\" from the COM1,total:%d characters.\n",++i,timeout.tv_sec+timeout.tv_usec*0.000001,buf,rd_num);
            transfer_started=1; 
        }
        else{
            printf("%d(间隔%4.3fs)：read fail! rd_num=%d。本次数据传输%s\n",++i,timeout.tv_sec+timeout.tv_usec*0.000001,rd_num,transfer_started==1?"已经结束":"尚未开始");
        
          }
        select(0,NULL,NULL,NULL,&timeout);/*精确定时*/
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

    printf("inBuf[i] = 0x%x\n",inBuf[i]);

    for(i = 0; i< inCnt; i++)
    {
        if(inBuf[i] == 0xe2)//header
        {
            outBuf[(destCnt)++] = inBuf[i];
            *readStatus = 1;
             //print_frame("header",outBuf,destCnt);
            // continue;
        }
        /*
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
        */
    }

}

int pocl_serial_send(int fd, char *send_buf, int data_len)
{
    ssize_t ret = 0;

    ret = write(fd, send_buf, data_len);
    if (ret == data_len)
    {
        //printf("send data is %s\n", send_buf);
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
    
    int dest_cnt = 0;

    fd=open(DEV_NAME,O_RDWR|O_NONBLOCK);
    if(fd==-1)
        printf("can not open the COM1!\n");
    else
        printf("open COM1 ok!\n");

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

void
pocl_montage_init_device_ops(struct pocl_device_ops *ops)
{
  ops->device_name = "montage";
  ops->probe = pocl_montage_probe;
  ops->uninit = pocl_montage_uninit;
  ops->reinit = pocl_montage_reinit;
  ops->init = pocl_montage_init;
  ops->alloc_mem_obj = pocl_montage_alloc_mem_obj;
  ops->free = pocl_montage_free;
  ops->read = pocl_montage_read;
  ops->read_rect = pocl_montage_read_rect;
  ops->write = pocl_montage_write;
  ops->write_rect = pocl_montage_write_rect;
  ops->copy = pocl_montage_copy;
  ops->copy_rect = pocl_montage_copy_rect;
  ops->memfill = pocl_montage_memfill;
  ops->map_mem = pocl_montage_map_mem;
  ops->compile_kernel = pocl_montage_compile_kernel;
  ops->unmap_mem = pocl_montage_unmap_mem;
  ops->run = pocl_montage_launch;  //add lunch
  //ops->run = pocl_montage_run;
  ops->run_native = pocl_montage_run_native;
  ops->join = pocl_montage_join;
  ops->submit = pocl_montage_submit;
  ops->broadcast = pocl_broadcast;
  ops->notify = pocl_montage_notify;
  ops->flush = pocl_montage_flush;
  ops->build_hash = pocl_montage_build_hash;

  ops->svm_free = pocl_montage_svm_free;
  ops->svm_alloc = pocl_montage_svm_alloc;
  /* no need to implement these two as they're noop
   * and pocl_exec_command takes care of it */
  ops->svm_map = NULL;
  ops->svm_unmap = NULL;
  ops->svm_copy = NULL;
  ops->svm_fill = NULL;

  ops->create_image = NULL;
  ops->free_image = NULL;
  ops->create_sampler = NULL;
  ops->free_sampler = NULL;
  ops->copy_image_rect = NULL;
  ops->write_image_rect = NULL;
  ops->read_image_rect = NULL;
  ops->map_image = NULL;
  ops->unmap_image = NULL;
  ops->fill_image = NULL;
}

char *
pocl_montage_build_hash (cl_device_id device)
{
  char* res = calloc(1000, sizeof(char));
#ifdef KERNELLIB_HOST_DISTRO_VARIANTS
  char *name = get_llvm_cpu_name ();
  snprintf (res, 1000, "montage-%s-%s", HOST_DEVICE_BUILD_HASH, name);
  POCL_MEM_FREE (name);
#else
  snprintf (res, 1000, "montage-%s", HOST_DEVICE_BUILD_HASH);
#endif
  return res;
}

static cl_device_partition_property montage_partition_properties[1] = { 0 };


static const char *final_ld_flags[] =
  {"-lm", "-nostartfiles", HOST_LD_FLAGS_ARRAY, NULL};

void
pocl_init_riscv_device_infos (cl_device_id dev)
{
  size_t i;
  dev->type = CL_DEVICE_TYPE_ACCELERATOR; //fix,by eason
  dev->max_work_item_dimensions = 3;
  dev->final_linkage_flags = final_ld_flags;

  SETUP_DEVICE_CL_VERSION(HOST_DEVICE_CL_VERSION_MAJOR, HOST_DEVICE_CL_VERSION_MINOR)
  /*
    The hard restriction will be the context data which is
    stored in stack that can be as small as 8K in Linux.
    Thus, there should be enough work-items alive to fill up
    the SIMD lanes times the vector units, but not more than
    that to avoid stack overflow and cache trashing.
  */
  int max_wg
      = pocl_get_int_option ("POCL_MAX_WORK_GROUP_SIZE", DEFAULT_WG_SIZE);
  assert (max_wg > 0);
  max_wg = min (max_wg, DEFAULT_WG_SIZE);
  if (max_wg < 0)
    max_wg = DEFAULT_WG_SIZE;

  dev->max_work_item_sizes[0] = dev->max_work_item_sizes[1]
      = dev->max_work_item_sizes[2] = dev->max_work_group_size = max_wg;

  dev->preferred_wg_size_multiple = 8;
#ifdef OCS_AVAILABLE
  cpu_setup_vector_widths (dev);
#else
  dev->preferred_vector_width_char = POCL_DEVICES_PREFERRED_VECTOR_WIDTH_CHAR;
  dev->preferred_vector_width_short = POCL_DEVICES_PREFERRED_VECTOR_WIDTH_SHORT;
  dev->preferred_vector_width_int = POCL_DEVICES_PREFERRED_VECTOR_WIDTH_INT;
  dev->preferred_vector_width_long = POCL_DEVICES_PREFERRED_VECTOR_WIDTH_LONG;
  dev->preferred_vector_width_float = POCL_DEVICES_PREFERRED_VECTOR_WIDTH_FLOAT;
  /* TODO: figure out what the difference between preferred and native widths are */
  dev->native_vector_width_char = POCL_DEVICES_NATIVE_VECTOR_WIDTH_CHAR;
  dev->native_vector_width_short = POCL_DEVICES_NATIVE_VECTOR_WIDTH_SHORT;
  dev->native_vector_width_int = POCL_DEVICES_NATIVE_VECTOR_WIDTH_INT;
  dev->native_vector_width_long = POCL_DEVICES_NATIVE_VECTOR_WIDTH_LONG;
  dev->native_vector_width_float = POCL_DEVICES_NATIVE_VECTOR_WIDTH_FLOAT;

#ifdef _CL_DISABLE_DOUBLE
  dev->native_vector_width_double = 0;
  dev->preferred_vector_width_double = 0;
#else
  dev->native_vector_width_double = POCL_DEVICES_NATIVE_VECTOR_WIDTH_DOUBLE;
  dev->preferred_vector_width_double = POCL_DEVICES_PREFERRED_VECTOR_WIDTH_DOUBLE;
#endif
#ifdef _CL_DISABLE_HALF
  dev->preferred_vector_width_half = 0;
  dev->native_vector_width_half = 0;
#else
  dev->preferred_vector_width_half = POCL_DEVICES_PREFERRED_VECTOR_WIDTH_HALF;
  dev->native_vector_width_half = POCL_DEVICES_NATIVE_VECTOR_WIDTH_HALF;
#endif

#endif

  dev->grid_width_specialization_limit = USHRT_MAX;
  dev->address_bits = HOST_DEVICE_ADDRESS_BITS;
  dev->image_support = CL_TRUE;
  /* Use the minimum values until we get a more sensible upper limit from
     somewhere. */
  dev->max_read_image_args = dev->max_write_image_args
      = dev->max_read_write_image_args = 128;
  dev->image2d_max_width = dev->image2d_max_height = 8192;
  dev->image3d_max_width = dev->image3d_max_height = dev->image3d_max_depth = 2048;
  dev->max_samplers = 16;

  for (i = 0; i < NUM_OPENCL_IMAGE_TYPES; ++i)
    {
      dev->num_image_formats[i]
          = sizeof (supported_image_formats) / sizeof (cl_image_format);
      dev->image_formats[i] = supported_image_formats;
    }

  dev->image_max_buffer_size = 65536;
  dev->image_max_array_size = 2048;
  dev->max_constant_args = 8;
  dev->max_mem_alloc_size = 0;
  dev->max_parameter_size = 1024;
  dev->min_data_type_align_size = MAX_EXTENDED_ALIGNMENT;
  dev->mem_base_addr_align = MAX_EXTENDED_ALIGNMENT;
  dev->half_fp_config = 0;
  dev->single_fp_config = CL_FP_ROUND_TO_NEAREST | CL_FP_INF_NAN;
#ifdef __x86_64__
  dev->single_fp_config |= (CL_FP_DENORM | CL_FP_ROUND_TO_INF
                            | CL_FP_ROUND_TO_ZERO
                            | CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT);
#ifdef OCS_AVAILABLE
  if (cpu_has_fma())
    dev->single_fp_config |= CL_FP_FMA;
#endif
#endif

#ifdef _CL_DISABLE_DOUBLE
  dev->double_fp_config = 0;
#else
  /* TODO: all of these are the minimum mandated, but not all CPUs may actually
   * support all of them. */
  dev->double_fp_config = CL_FP_FMA | CL_FP_ROUND_TO_NEAREST
                          | CL_FP_ROUND_TO_ZERO | CL_FP_ROUND_TO_INF
                          | CL_FP_INF_NAN | CL_FP_DENORM;
  /* this is a workaround for issue 28 in https://github.com/Oblomov/clinfo
   * https://github.com/Oblomov/clinfo/issues/28 */
  dev->double_fp_config |= CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT;
#endif

  dev->global_mem_cache_type = CL_NONE;
  dev->global_mem_cacheline_size = 0;
  dev->global_mem_cache_size = 0;
  dev->global_mem_size = 0;
  dev->max_constant_buffer_size = 0;
  dev->max_constant_args = 8;
  dev->local_mem_type = CL_GLOBAL;
  dev->local_mem_size = 0;
  dev->error_correction_support = CL_FALSE;
  dev->host_unified_memory = CL_TRUE;

  dev->profiling_timer_resolution = pocl_timer_resolution;

  dev->endian_little = !(WORDS_BIGENDIAN);
  dev->available = CL_TRUE;
  dev->compiler_available = CL_TRUE;
  dev->spmd = CL_FALSE;
  dev->arg_buffer_launcher = CL_FALSE;
  dev->workgroup_pass = CL_TRUE;
  dev->execution_capabilities = CL_EXEC_KERNEL | CL_EXEC_NATIVE_KERNEL;
  dev->platform = 0;

  dev->parent_device = NULL;
  /* These two are only used for subdevices.
   * Each subdevice has these two setup when created.
   * The subdevice will then use these CUs:
   *  [start, start+1, ..., start+count-1]
   * this may not work with more complicated partitioning schemes,
   * but is good enough for now. */
  dev->core_start = 0;
  dev->core_count = 0;
  /* montage does not support partitioning */
  dev->max_sub_devices = 1;
  dev->num_partition_properties = 1;
  dev->partition_properties = montage_partition_properties;
  dev->num_partition_types = 0;
  dev->partition_type = NULL;

  dev->device_side_printf = 1;
  dev->printf_buffer_size = 16 * 1024 * 1024;

  dev->vendor = "pocl";
  dev->profile = "FULL_PROFILE";
  /* Note: The specification describes identifiers being delimited by
     only a single space character. Some programs that check the device's
     extension  string assume this rule. Future extension additions should
     ensure that there is no more than a single space between
     identifiers. */

  dev->global_as_id = dev->local_as_id = dev->constant_as_id = 0;

  dev->should_allocate_svm = 0;
  /* OpenCL 2.0 properties */
  dev->svm_caps = CL_DEVICE_SVM_COARSE_GRAIN_BUFFER
                  | CL_DEVICE_SVM_FINE_GRAIN_BUFFER
                  | CL_DEVICE_SVM_ATOMICS;
  /* TODO these are minimums, figure out whats a reasonable value */
  dev->max_events = 1024;
  dev->max_queues = 1;
  dev->max_pipe_args = 16;
  dev->max_pipe_active_res = 1;
  dev->max_pipe_packet_size = 1024;
  dev->dev_queue_pref_size = 16 * 1024;
  dev->dev_queue_max_size = 256 * 1024;
  dev->on_dev_queue_props = CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE
                               | CL_QUEUE_PROFILING_ENABLE;
  dev->on_host_queue_props = CL_QUEUE_PROFILING_ENABLE;
  dev->extensions = HOST_DEVICE_EXTENSIONS;
  dev->has_64bit_long = 1;
  dev->autolocals_to_args = 1;
  dev->device_alloca_locals = 0;

#ifdef OCS_AVAILABLE
  dev->llvm_target_triplet = "riscv64"; //riscv64
#ifdef HOST_CPU_FORCED
  dev->llvm_cpu = OCL_KERNEL_TARGET_CPU;
#else
  dev->llvm_cpu = "sifive-u54";
#endif

  dev->spirv_version = "SPIR-V_1.2";
#else /* No compiler, no CPU info */
  dev->llvm_cpu = NULL;
  dev->llvm_target_triplet = "";
#endif

}

unsigned int
pocl_montage_probe(struct pocl_device_ops *ops)
{
  int env_count = pocl_device_get_env_count(ops->device_name);

  /* No env specified, so pthread will be used instead of montage */
  if(env_count < 0)
    return 0;

  return env_count;
}

cl_int
pocl_montage_init (unsigned j, cl_device_id device, const char* parameters)
{
  struct data *d;
  cl_int ret = CL_SUCCESS;
  int err;
  static int first_montage_init = 1;

  if (first_montage_init)
    {
      POCL_MSG_WARN ("INIT dlcache DOTO delete\n");
      pocl_init_dlhandle_cache();
      first_montage_init = 0;
    }
  device->global_mem_id = 0;

  d = (struct data *) calloc (1, sizeof (struct data));
  if (d == NULL)
    return CL_OUT_OF_HOST_MEMORY;

  d->current_kernel = NULL;
  device->data = d;

  pocl_init_riscv_device_infos (device);

  /* hwloc probes OpenCL device info at its initialization in case
     the OpenCL extension is enabled. This causes to printout
     an unimplemented property error because hwloc is used to
     initialize global_mem_size which it is not yet. Just put
     a nonzero there for now. */
  device->global_mem_size = 1;
  err = pocl_topology_detect_device_info(device);
  if (err)
    ret = CL_INVALID_DEVICE;

  POCL_INIT_LOCK (d->cq_lock);

  assert (device->printf_buffer_size > 0);
  d->printf_buffer = pocl_aligned_malloc (MAX_EXTENDED_ALIGNMENT,
                                          device->printf_buffer_size);
  assert (d->printf_buffer != NULL);

  pocl_cpuinfo_detect_device_info(device);
  pocl_set_buffer_image_limits(device);

  /* in case hwloc doesn't provide a PCI ID, let's generate
     a vendor id that hopefully is unique across vendors. */
  const char *magic = "pocl";
  if (device->vendor_id == 0)
    device->vendor_id =
      magic[0] | magic[1] << 8 | magic[2] << 16 | magic[3] << 24;

  device->vendor_id += j;

  /* The montage driver represents only one "compute unit" as
     it doesn't exploit multiple hardware threads. Multiple
     montage devices can be still used for task level parallelism
     using multiple OpenCL devices. */
  device->max_compute_units = 1;

   pocl_uart_config(); //add for pocl uart


  return ret;
}

cl_int
pocl_montage_alloc_mem_obj (cl_device_id device, cl_mem mem_obj, void* host_ptr)
{
  void *b = NULL;
  cl_mem_flags flags = mem_obj->flags;
  unsigned i;
  POCL_MONTAGE_MSG("malloc size :%ld\n",mem_obj->size);
  POCL_MSG_PRINT_MEMORY (" mem %p, dev %d\n", mem_obj, device->dev_id);
  /* check if some driver has already allocated memory for this mem_obj
     in our global address space, and use that*/
  for (i = 0; i < mem_obj->context->num_devices; ++i)
    {
      if (!mem_obj->device_ptrs[i].available)
        continue;

      if (mem_obj->device_ptrs[i].global_mem_id == device->global_mem_id
          && mem_obj->device_ptrs[i].mem_ptr != NULL)
        {
          mem_obj->device_ptrs[device->dev_id].mem_ptr =
            mem_obj->device_ptrs[i].mem_ptr;
          POCL_MSG_PRINT_MEMORY (
              "mem %p dev %d, using already allocated mem\n", mem_obj,
              device->dev_id);
          return CL_SUCCESS;
        }
    }

  /* memory for this global memory is not yet allocated -> do it */
  if (flags & CL_MEM_USE_HOST_PTR)
    {
      // mem_host_ptr must be non-NULL
      assert(host_ptr != NULL);
      b = host_ptr;
    }
  else
    {
      POCL_MSG_PRINT_MEMORY ("!USE_HOST_PTR\n");
      b = pocl_aligned_malloc_global_mem (device, MAX_EXTENDED_ALIGNMENT,
                                          mem_obj->size);
      if (b==NULL)
        return CL_MEM_OBJECT_ALLOCATION_FAILURE;

      mem_obj->shared_mem_allocation_owner = device;
    }

  /* use this dev mem allocation as host ptr */
  if ((flags & CL_MEM_ALLOC_HOST_PTR) && (mem_obj->mem_host_ptr == NULL))
    mem_obj->mem_host_ptr = b;

  if (flags & CL_MEM_COPY_HOST_PTR)
    {
      POCL_MSG_PRINT_MEMORY ("COPY_HOST_PTR\n");
      // mem_host_ptr must be non-NULL
      assert(host_ptr != NULL);
      memcpy (b, host_ptr, mem_obj->size);
    }

  mem_obj->device_ptrs[device->dev_id].mem_ptr = b;
  return CL_SUCCESS;
}


void
pocl_montage_free (cl_device_id device, cl_mem memobj)
{
  cl_mem_flags flags = memobj->flags;

  /* aloocation owner executes freeing */
  if (flags & CL_MEM_USE_HOST_PTR ||
      memobj->shared_mem_allocation_owner != device)
    return;

  void* ptr = memobj->device_ptrs[device->dev_id].mem_ptr;
  size_t size = memobj->size;

  if (memobj->flags | CL_MEM_ALLOC_HOST_PTR)
    memobj->mem_host_ptr = NULL;
  pocl_free_global_mem(device, ptr, size);
}

void
pocl_montage_read (void *data,
                 void *__restrict__ host_ptr,
                 pocl_mem_identifier * src_mem_id,
                 cl_mem src_buf,
                 size_t offset, size_t size)
{
  void *__restrict__ device_ptr = src_mem_id->mem_ptr;
  if (host_ptr == device_ptr)
    return;

  memcpy (host_ptr, (char *)device_ptr + offset, size);
}

extern uint8_t my_arg[MAX_DLFW_PAGE_SIZE];
void
pocl_montage_write (void *data,
                  const void *__restrict__  host_ptr,
                  pocl_mem_identifier * dst_mem_id,
                  cl_mem dst_buf,
                  size_t offset, size_t size)
{
  int i;
  void *__restrict__ device_ptr = dst_mem_id->mem_ptr;
  if (host_ptr == device_ptr)
    return;
  
  memcpy ((char *)device_ptr + offset, host_ptr, size);

  pocl_serial_send(fd,my_arg,MAX_DLFW_PAGE_SIZE);
}



extern char final_lld_path[POCL_FILENAME_LENGTH];
void
pocl_montage_launch(void *data, _cl_command_node *cmd)
{

  struct data *d;
  pthread_t wtid;
  int rd_num=0;
  struct pocl_context *pc = &cmd->command.run.pc;
  cl_kernel kernel = cmd->command.run.kernel;
  unsigned i;
  unsigned j ;
  char *PreprocessedOut = NULL;
  uint64_t PreprocessedSize = 0;

 	uint64_t	pageNums, remainSize ;
	uint64_t	page, offset;

  char elf_cmd [3] = {0x00};
  printf("send elf data cmd\n");
  pocl_read_file(final_lld_path, &PreprocessedOut, &PreprocessedSize);

  elf_cmd[0] = POCL_ELF;
  elf_cmd[1] = (PreprocessedSize - PreprocessedSize % 0x10) / 0x10;
  elf_cmd[2] =  PreprocessedSize % 0x10;

  char* s=(char*)malloc(3+PreprocessedSize+1);
  memcpy(s,elf_cmd,3);
  memcpy(s+3,PreprocessedOut,PreprocessedSize+1);

  pageNums = (PreprocessedSize+3) / MAX_DLFW_PAGE_SIZE ;
  remainSize = (PreprocessedSize+3) % MAX_DLFW_PAGE_SIZE;
	
  for (page = 0; page < pageNums; page++) {
        offset = page * MAX_DLFW_PAGE_SIZE;
        pocl_serial_send(fd,s + offset, MAX_DLFW_PAGE_SIZE);
   
  }
  if (remainSize) {
       offset = pageNums * MAX_DLFW_PAGE_SIZE;
       pocl_serial_send(fd,s + offset, MAX_DLFW_PAGE_SIZE);
  }
  

  free(s);

  printf("send kernel name cmd\n");
  char name_cmd[MAX_DLFW_PAGE_SIZE];
  memset(name_cmd,0x00,MAX_DLFW_PAGE_SIZE);
  name_cmd[0] = POCL_NAME;
  int len =  strlen(kernel->name);
  name_cmd[1] = (len&0xff) ;
  name_cmd[2] = ((len >> 8 )& 0xff);//size;

  memcpy(name_cmd+3,kernel->name,len);
   sleep(1);
  pocl_serial_send(fd,name_cmd,MAX_DLFW_PAGE_SIZE);

  printf("send local work size  cmd\n");
  char local_cmd[MAX_DLFW_PAGE_SIZE];

  memset(local_cmd,0x00,MAX_DLFW_PAGE_SIZE);

  local_cmd[0] = POCL_PC;
  local_cmd[1] = 2;
  local_cmd[2] = 0; 
  local_cmd[3] = 64;
  sleep(1);
  pocl_serial_send(fd,local_cmd,MAX_DLFW_PAGE_SIZE);

  

/*
  //elf data
  montage_elf = (pocl_uart_data*)malloc(sizeof(pocl_uart_data));
  montage_elf->cmd = 0x55;
  montage_elf->len = PreprocessedSize;
  montage_elf->uart_data = PreprocessedOut;

  printf("###### %s ######\n", __func__);
  for( i = 0; i< 10;i++)
  {
     printf("%08x-\n",((char *)montage_elf->uart_data)[i]); //MEMORY
     if (j%10 == 0)
			printf("\n");
		j++;
  }
 
  ;
  sleep(1);
*/
    sleep(1);
    rd_num = pthread_create(&wtid,NULL,monitor_serial_readable,&fd);
    if(rd_num != 0)
    {
        printf("create thread failed\n");
        exit(-1);
    }
    rd_num = pthread_join(wtid,NULL);
    if(rd_num != 0)
    {
        printf("join thread failed\n");
        exit(-1);
    }
   

 // POCL_MONTAGE_MSG("pocl_montage_launch ok!\n");
}

void
pocl_montage_run_native (void *data, _cl_command_node *cmd)
{
  cmd->command.native.user_func(cmd->command.native.args);
}

void
pocl_montage_copy (void *data,
                 pocl_mem_identifier * dst_mem_id,
                 cl_mem dst_buf,
                 pocl_mem_identifier * src_mem_id,
                 cl_mem src_buf,
                 size_t dst_offset,
                 size_t src_offset,
                 size_t size)
{
  char *__restrict__ src_ptr = src_mem_id->mem_ptr;
  char *__restrict__ dst_ptr = dst_mem_id->mem_ptr;
  if ((src_ptr + src_offset) == (dst_ptr + dst_offset))
    return;

  memcpy (dst_ptr + dst_offset, src_ptr + src_offset, size);
}

void
pocl_montage_copy_rect (void *data,
                      pocl_mem_identifier * dst_mem_id,
                      cl_mem dst_buf,
                      pocl_mem_identifier * src_mem_id,
                      cl_mem src_buf,
                      const size_t *__restrict__ const dst_origin,
                      const size_t *__restrict__ const src_origin,
                      const size_t *__restrict__ const region,
                      size_t const dst_row_pitch,
                      size_t const dst_slice_pitch,
                      size_t const src_row_pitch,
                      size_t const src_slice_pitch)
{

  void *__restrict__ src_ptr = src_mem_id->mem_ptr;
  void *__restrict__ dst_ptr = dst_mem_id->mem_ptr;
  char const *__restrict const adjusted_src_ptr =
    (char const*)src_ptr +
    src_origin[0] + src_row_pitch * src_origin[1] + src_slice_pitch * src_origin[2];
  char *__restrict__ const adjusted_dst_ptr =
    (char*)dst_ptr +
    dst_origin[0] + dst_row_pitch * dst_origin[1] + dst_slice_pitch * dst_origin[2];

  POCL_MSG_PRINT_MEMORY (
      "montage COPY RECT \n"
      "SRC %p DST %p SIZE %zu\n"
      "src origin %u %u %u dst origin %u %u %u \n"
      "src_row_pitch %lu src_slice pitch %lu\n"
      "dst_row_pitch %lu dst_slice_pitch %lu\n"
      "reg[0] %lu reg[1] %lu reg[2] %lu\n",
      adjusted_src_ptr, adjusted_dst_ptr,
      region[0] * region[1] * region[2],
      (unsigned)src_origin[0], (unsigned)src_origin[1],
      (unsigned)src_origin[2], (unsigned)dst_origin[0],
      (unsigned)dst_origin[1], (unsigned)dst_origin[2],
      (unsigned long)src_row_pitch, (unsigned long)src_slice_pitch,
      (unsigned long)dst_row_pitch, (unsigned long)dst_slice_pitch,
      (unsigned long)region[0], (unsigned long)region[1], (unsigned long)region[2]);

  size_t j, k;

  /* TODO: handle overlaping regions */
  if ((src_row_pitch == dst_row_pitch && dst_row_pitch == region[0])
      && (src_slice_pitch == dst_slice_pitch && dst_slice_pitch == (region[1]*region[0])))
    {
      memcpy (adjusted_dst_ptr, adjusted_src_ptr,
              region[2] * region[1] * region[0]);
    }
  else
    {
      for (k = 0; k < region[2]; ++k)
        for (j = 0; j < region[1]; ++j)
          memcpy (adjusted_dst_ptr + dst_row_pitch * j + dst_slice_pitch * k,
                  adjusted_src_ptr + src_row_pitch * j + src_slice_pitch * k,
                  region[0]);
    }
}

void
pocl_montage_write_rect (void *data,
                       const void *__restrict__ const host_ptr,
                       pocl_mem_identifier * dst_mem_id,
                       cl_mem dst_buf,
                       const size_t *__restrict__ const buffer_origin,
                       const size_t *__restrict__ const host_origin,
                       const size_t *__restrict__ const region,
                       size_t const buffer_row_pitch,
                       size_t const buffer_slice_pitch,
                       size_t const host_row_pitch,
                       size_t const host_slice_pitch)
{
  void *__restrict__ device_ptr = dst_mem_id->mem_ptr;

  char *__restrict const adjusted_device_ptr
      = (char *)device_ptr + buffer_origin[0]
        + buffer_row_pitch * buffer_origin[1]
        + buffer_slice_pitch * buffer_origin[2];
  char const *__restrict__ const adjusted_host_ptr =
    (char const*)host_ptr +
    host_origin[0] + host_row_pitch * host_origin[1] + host_slice_pitch * host_origin[2];

  POCL_MSG_PRINT_MEMORY (
      "montage WRITE RECT \n"
      "SRC HOST %p DST DEV %p SIZE %zu\n"
      "borigin %u %u %u horigin %u %u %u \n"
      "row_pitch %lu slice pitch \n"
      "%lu host_row_pitch %lu host_slice_pitch %lu\n"
      "reg[0] %lu reg[1] %lu reg[2] %lu\n",
      adjusted_host_ptr, adjusted_device_ptr,
      region[0] * region[1] * region[2],
      (unsigned)buffer_origin[0], (unsigned)buffer_origin[1],
      (unsigned)buffer_origin[2], (unsigned)host_origin[0],
      (unsigned)host_origin[1], (unsigned)host_origin[2],
      (unsigned long)buffer_row_pitch, (unsigned long)buffer_slice_pitch,
      (unsigned long)host_row_pitch, (unsigned long)host_slice_pitch,
      (unsigned long)region[0], (unsigned long)region[1], (unsigned long)region[2]);

  size_t j, k;

  /* TODO: handle overlaping regions */
  if ((buffer_row_pitch == host_row_pitch
            && host_row_pitch == region[0])
      && (buffer_slice_pitch == host_slice_pitch
            && host_slice_pitch == (region[1]*region[0])))
    {
      memcpy (adjusted_device_ptr,
              adjusted_host_ptr,
              region[2] * region[1] * region[0]);
    }
  else
    {
      for (k = 0; k < region[2]; ++k)
        for (j = 0; j < region[1]; ++j)
          memcpy (adjusted_device_ptr
                    + buffer_row_pitch * j
                    + buffer_slice_pitch * k,
                  adjusted_host_ptr
                    + host_row_pitch * j
                    + host_slice_pitch * k,
                  region[0]);
    }
}

void
pocl_montage_read_rect (void *data,
                      void *__restrict__ const host_ptr,
                      pocl_mem_identifier * src_mem_id,
                      cl_mem src_buf,
                      const size_t *__restrict__ const buffer_origin,
                      const size_t *__restrict__ const host_origin,
                      const size_t *__restrict__ const region,
                      size_t const buffer_row_pitch,
                      size_t const buffer_slice_pitch,
                      size_t const host_row_pitch,
                      size_t const host_slice_pitch)
{
  void *__restrict__ device_ptr = src_mem_id->mem_ptr;

  char const *__restrict const adjusted_device_ptr =
    (char const*)device_ptr +
    buffer_origin[2] * buffer_slice_pitch + buffer_origin[1] * buffer_row_pitch + buffer_origin[0];
  char *__restrict__ const adjusted_host_ptr =
    (char*)host_ptr +
    host_origin[2] * host_slice_pitch + host_origin[1] * host_row_pitch + host_origin[0];

  POCL_MSG_PRINT_MEMORY (
      "montage READ RECT \n"
      "SRC DEV %p DST HOST %p SIZE %zu\n"
      "borigin %u %u %u horigin %u %u %u row_pitch %lu slice pitch "
      "%lu host_row_pitch %lu host_slice_pitch %lu\n"
      "reg[0] %lu reg[1] %lu reg[2] %lu\n",
      adjusted_device_ptr, adjusted_host_ptr,
      region[0] * region[1] * region[2],
      (unsigned)buffer_origin[0], (unsigned)buffer_origin[1],
      (unsigned)buffer_origin[2], (unsigned)host_origin[0],
      (unsigned)host_origin[1], (unsigned)host_origin[2],
      (unsigned long)buffer_row_pitch, (unsigned long)buffer_slice_pitch,
      (unsigned long)host_row_pitch, (unsigned long)host_slice_pitch,
      (unsigned long)region[0], (unsigned long)region[1], (unsigned long)region[2]);

  size_t j, k;

  /* TODO: handle overlaping regions */
  if ((buffer_row_pitch == host_row_pitch
            && host_row_pitch == region[0])
      && (buffer_slice_pitch == host_slice_pitch
            && host_slice_pitch == (region[1]*region[0])))
    {
      memcpy (adjusted_host_ptr,
              adjusted_device_ptr,
              region[2] * region[1] * region[0]);
    }
  else
    {
      for (k = 0; k < region[2]; ++k)
        for (j = 0; j < region[1]; ++j)
          memcpy (adjusted_host_ptr
                    + host_row_pitch * j
                    + host_slice_pitch * k,
                  adjusted_device_ptr
                    + buffer_row_pitch * j
                    + buffer_slice_pitch * k,
                  region[0]);
    }
}

void
pocl_montage_memfill (void *data, pocl_mem_identifier *dst_mem_id,
                    cl_mem dst_buf, size_t size, size_t offset,
                    const void *__restrict__ pattern, size_t pattern_size)
{
  void *__restrict__ ptr = dst_mem_id->mem_ptr;
  size_t i;
  unsigned j;

  /* memfill size is in bytes, we wanto make it into elements */
  size /= pattern_size;
  offset /= pattern_size;

  switch (pattern_size)
    {
    case 1:
      {
      uint8_t * p = (uint8_t*)ptr + offset;
      for (i = 0; i < size; i++)
        p[i] = *(uint8_t*)pattern;
      }
      break;
    case 2:
      {
      uint16_t * p = (uint16_t*)ptr + offset;
      for (i = 0; i < size; i++)
        p[i] = *(uint16_t*)pattern;
      }
      break;
    case 4:
      {
      uint32_t * p = (uint32_t*)ptr + offset;
      for (i = 0; i < size; i++)
        p[i] = *(uint32_t*)pattern;
      }
      break;
    case 8:
      {
      uint64_t * p = (uint64_t*)ptr + offset;
      for (i = 0; i < size; i++)
        p[i] = *(uint64_t*)pattern;
      }
      break;
    case 16:
      {
      uint64_t * p = (uint64_t*)ptr + (offset << 1);
      for (i = 0; i < size; i++)
        for (j = 0; j < 2; j++)
          p[(i<<1) + j] = *((uint64_t*)pattern + j);
      }
      break;
    case 32:
      {
      uint64_t * p = (uint64_t*)ptr + (offset << 2);
      for (i = 0; i < size; i++)
        for (j = 0; j < 4; j++)
          p[(i<<2) + j] = *((uint64_t*)pattern + j);
      }
      break;
    case 64:
      {
      uint64_t * p = (uint64_t*)ptr + (offset << 3);
      for (i = 0; i < size; i++)
        for (j = 0; j < 8; j++)
          p[(i<<3) + j] = *((uint64_t*)pattern + j);
      }
      break;
    case 128:
      {
      uint64_t * p = (uint64_t*)ptr + (offset << 4);
      for (i = 0; i < size; i++)
        for (j = 0; j < 16; j++)
          p[(i<<4) + j] = *((uint64_t*)pattern + j);
      }
      break;
    default:
      assert (0 && "Invalid pattern size");
      break;
    }
}

cl_int
pocl_montage_map_mem (void *data,
                    pocl_mem_identifier * src_mem_id,
                    cl_mem src_buf,
                    mem_mapping_t *map)
{
  void *__restrict__ src_device_ptr = src_mem_id->mem_ptr;
  void *host_ptr = map->host_ptr;
  size_t offset = map->offset;
  size_t size = map->size;

  if (host_ptr == NULL)
    {
      map->host_ptr = ((char *)src_device_ptr + offset);
      return CL_SUCCESS;
    }

  if (map->map_flags & CL_MAP_WRITE_INVALIDATE_REGION)
    return CL_SUCCESS;

  /* If the buffer was allocated with CL_MEM_ALLOC_HOST_PTR |
   * CL_MEM_COPY_HOST_PTR,
   * the dst_host_ptr is not the same memory as pocl's device_ptrs[], and we
   * need to copy pocl's buffer content back to dst_host_ptr. */
  if (host_ptr != (src_device_ptr + offset))
    {
      POCL_MSG_PRINT_MEMORY ("device: MAP memcpy() "
                             "src_device_ptr %p + offset %zu"
                             "to dst_host_ptr %p\n",
                             src_device_ptr, offset, host_ptr);
      memcpy ((char *)host_ptr, (char *)src_device_ptr + offset, size);
    }
  return CL_SUCCESS;
}

cl_int
pocl_montage_unmap_mem(void *data,
                     pocl_mem_identifier * dst_mem_id,
                     cl_mem dst_buf,
                     mem_mapping_t *map)
{
  void *__restrict__ dst_device_ptr = dst_mem_id->mem_ptr;
  /* it could be CL_MAP_READ | CL_MAP_WRITE(..invalidate) which has to be
   * handled like a write */
  if (map->map_flags == CL_MAP_READ)
    return CL_SUCCESS;

  void *host_ptr = map->host_ptr;
  assert (host_ptr != NULL);
  size_t offset = map->offset;
  size_t size = map->size;

  /* If the buffer was allocated with CL_MEM_ALLOC_src_host_ptr |
   * CL_MEM_COPY_src_host_ptr,
   * the src_host_ptr is not the same memory as pocl's device_ptrs[], and we
   * need to copy src_host_ptr content back to  pocl's device_ptrs[]. */
  if (host_ptr != (dst_device_ptr + offset))
    {
      POCL_MSG_PRINT_MEMORY ("device: UNMAP memcpy() "
                             "host_ptr %p to buf_ptr %p + offset %zu\n",
                             host_ptr, dst_device_ptr, offset);
      memcpy ((char *)dst_device_ptr + offset, (char *)host_ptr, size);
    }
  return CL_SUCCESS;
}

cl_int
pocl_montage_uninit (unsigned j, cl_device_id device)
{
  struct data *d = (struct data*)device->data;
  POCL_DESTROY_LOCK (d->cq_lock);
  pocl_aligned_free (d->printf_buffer);
  POCL_MEM_FREE(d);
  close(fd);
  device->data = NULL;
  return CL_SUCCESS;
}

cl_int
pocl_montage_reinit (unsigned j, cl_device_id device)
{
  struct data *d = (struct data *)calloc (1, sizeof (struct data));
  if (d == NULL)
    return CL_OUT_OF_HOST_MEMORY;

  d->current_kernel = NULL;

  assert (device->printf_buffer_size > 0);
  d->printf_buffer = pocl_aligned_malloc (MAX_EXTENDED_ALIGNMENT,
                                          device->printf_buffer_size);
  assert (d->printf_buffer != NULL);

  POCL_INIT_LOCK (d->cq_lock);
  device->data = d;
  return CL_SUCCESS;
}


static void montage_command_scheduler (struct data *d)
{
  _cl_command_node *node;

  /* execute commands from ready list */
  while ((node = d->ready_list))
    {
      assert (pocl_command_is_ready(node->event));
      assert (node->event->status == CL_SUBMITTED);
      CDL_DELETE (d->ready_list, node);
      POCL_UNLOCK (d->cq_lock);
      pocl_exec_command (node);
      POCL_LOCK (d->cq_lock);
    }

  return;
}

void
pocl_montage_submit (_cl_command_node *node, cl_command_queue cq)
{
  struct data *d = node->device->data;

  if (node != NULL && node->type == CL_COMMAND_NDRANGE_KERNEL)
    pocl_check_kernel_dlhandle_cache (node, 1, 1);

  node->ready = 1;
  POCL_LOCK (d->cq_lock);
  pocl_command_push(node, &d->ready_list, &d->command_list);

  POCL_UNLOCK_OBJ (node->event);
  montage_command_scheduler (d);
  POCL_UNLOCK (d->cq_lock);

  return;
}

void pocl_montage_flush (cl_device_id device, cl_command_queue cq)
{
  struct data *d = (struct data*)device->data;

  POCL_LOCK (d->cq_lock);
  montage_command_scheduler (d);
  POCL_UNLOCK (d->cq_lock);
}

void
pocl_montage_join (cl_device_id device, cl_command_queue cq)
{
  struct data *d = (struct data*)device->data;

  POCL_LOCK (d->cq_lock);
  montage_command_scheduler (d);
  POCL_UNLOCK (d->cq_lock);

  return;
}

void
pocl_montage_notify (cl_device_id device, cl_event event, cl_event finished)
{
  struct data *d = (struct data*)device->data;
  _cl_command_node * volatile node = event->command;

  if (finished->status < CL_COMPLETE)
    {
      pocl_update_event_failed (event);
      return;
    }

  if (!node->ready)
    return;

  if (pocl_command_is_ready (event))
    {
      if (event->status == CL_QUEUED)
        {
          pocl_update_event_submitted (event);
          POCL_LOCK (d->cq_lock);
          CDL_DELETE (d->command_list, node);
          CDL_PREPEND (d->ready_list, node);
          montage_command_scheduler (d);
          POCL_UNLOCK (d->cq_lock);
        }
      return;
    }
}

void
pocl_montage_compile_kernel (_cl_command_node *cmd, cl_kernel kernel,
                           cl_device_id device, int specialize)
{
  if (cmd != NULL && cmd->type == CL_COMMAND_NDRANGE_KERNEL)
    pocl_check_kernel_dlhandle_cache (cmd, 0, specialize);
}

/*********************** IMAGES ********************************/

cl_int pocl_montage_copy_image_rect( void *data,
                                   cl_mem src_image,
                                   cl_mem dst_image,
                                   pocl_mem_identifier *src_mem_id,
                                   pocl_mem_identifier *dst_mem_id,
                                   const size_t *src_origin,
                                   const size_t *dst_origin,
                                   const size_t *region)
{

  size_t px = src_image->image_elem_size * src_image->image_channels;
  const size_t adj_src_origin[3]
      = { src_origin[0] * px, src_origin[1], src_origin[2] };
  const size_t adj_dst_origin[3]
      = { dst_origin[0] * px, dst_origin[1], dst_origin[2] };
  const size_t adj_region[3] = { region[0] * px, region[1], region[2] };

  POCL_MSG_PRINT_MEMORY (
      " montage COPY IMAGE RECT \n"
      "dst_image %p dst_mem_id %p \n"
      "src_image %p src_mem_id %p \n"
      "dst_origin [0,1,2] %zu %zu %zu \n"
      "src_origin [0,1,2] %zu %zu %zu \n"
      "region [0,1,2] %zu %zu %zu \n"
      "px %zu\n",
      dst_image, dst_mem_id,
      src_image, src_mem_id,
      dst_origin[0], dst_origin[1], dst_origin[2],
      src_origin[0], src_origin[1], src_origin[2],
      region[0], region[1], region[2],
      px);

  pocl_montage_copy_rect(data,
                       dst_mem_id,
                       NULL,
                       src_mem_id,
                       NULL,
                       adj_dst_origin,
                       adj_src_origin,
                       adj_region,
                       dst_image->image_row_pitch,
                       dst_image->image_slice_pitch,
                       src_image->image_row_pitch,
                       src_image->image_slice_pitch);

  return CL_SUCCESS;
}

/* copies a region from host or device buffer to device image */
cl_int pocl_montage_write_image_rect (  void *data,
                                      cl_mem dst_image,
                                      pocl_mem_identifier *dst_mem_id,
                                      const void *__restrict__ src_host_ptr,
                                      pocl_mem_identifier *src_mem_id,
                                      const size_t *origin,
                                      const size_t *region,
                                      size_t src_row_pitch,
                                      size_t src_slice_pitch,
                                      size_t src_offset)
{
  POCL_MSG_PRINT_MEMORY (
      "montage WRITE IMAGE RECT \n"
      "dst_image %p dst_mem_id %p \n"
      "src_hostptr %p src_mem_id %p \n"
      "origin [0,1,2] %zu %zu %zu \n"
      "region [0,1,2] %zu %zu %zu \n"
      "row %zu slice %zu offset %zu \n",
      dst_image, dst_mem_id,
      src_host_ptr, src_mem_id,
      origin[0], origin[1], origin[2],
      region[0], region[1], region[2],
      src_row_pitch, src_slice_pitch, src_offset);

  const void *__restrict__ ptr
      = src_host_ptr ? src_host_ptr : src_mem_id->mem_ptr;
  ptr += src_offset;
  const size_t zero_origin[3] = { 0 };
  size_t px = dst_image->image_elem_size * dst_image->image_channels;
  if (src_row_pitch == 0)
    src_row_pitch = px * region[0];
  if (src_slice_pitch == 0)
    src_slice_pitch = src_row_pitch * region[1];

  const size_t adj_origin[3] = { origin[0] * px, origin[1], origin[2] };
  const size_t adj_region[3] = { region[0] * px, region[1], region[2] };

  pocl_montage_write_rect (data,
                         ptr,
                         dst_mem_id,
                         NULL,
                         adj_origin,
                         zero_origin,
                         adj_region,
                         dst_image->image_row_pitch,
                         dst_image->image_slice_pitch,
                         src_row_pitch,
                         src_slice_pitch);
  return CL_SUCCESS;
}

/* copies a region from device image to host or device buffer */
cl_int pocl_montage_read_image_rect(  void *data,
                                    cl_mem src_image,
                                    pocl_mem_identifier *src_mem_id,
                                    void *__restrict__ dst_host_ptr,
                                    pocl_mem_identifier *dst_mem_id,
                                    const size_t *origin,
                                    const size_t *region,
                                    size_t dst_row_pitch,
                                    size_t dst_slice_pitch,
                                    size_t dst_offset)
{
  POCL_MSG_PRINT_MEMORY (
      "montage READ IMAGE RECT \n"
      "src_image %p src_mem_id %p \n"
      "dst_hostptr %p dst_mem_id %p \n"
      "origin [0,1,2] %zu %zu %zu \n"
      "region [0,1,2] %zu %zu %zu \n"
      "row %zu slice %zu offset %zu \n",
      src_image, src_mem_id,
      dst_host_ptr, dst_mem_id,
      origin[0], origin[1], origin[2],
      region[0], region[1], region[2],
      dst_row_pitch, dst_slice_pitch, dst_offset);

  void *__restrict__ ptr = dst_host_ptr ? dst_host_ptr : dst_mem_id->mem_ptr;
  ptr += dst_offset;
  const size_t zero_origin[3] = { 0 };
  size_t px = src_image->image_elem_size * src_image->image_channels;
  if (dst_row_pitch == 0)
    dst_row_pitch = px * region[0];
  if (dst_slice_pitch == 0)
    dst_slice_pitch = dst_row_pitch * region[1];
  const size_t adj_origin[3] = { origin[0] * px, origin[1], origin[2] };
  const size_t adj_region[3] = { region[0] * px, region[1], region[2] };

  pocl_montage_read_rect (data,
                        ptr,
                        src_mem_id,
                        NULL,
                        adj_origin,
                        zero_origin,
                        adj_region,
                        src_image->image_row_pitch,
                        src_image->image_slice_pitch,
                        dst_row_pitch,
                        dst_slice_pitch);
  return CL_SUCCESS;
}


cl_int pocl_montage_map_image (void *data,
                             pocl_mem_identifier *mem_id,
                             cl_mem src_image,
                             mem_mapping_t *map)
{
  if (map->host_ptr == NULL)
    {
      map->host_ptr = (char *)mem_id->mem_ptr + map->offset;
      return CL_SUCCESS;
    }

  if (map->map_flags & CL_MAP_WRITE_INVALIDATE_REGION)
    return CL_SUCCESS;

  if (map->host_ptr != ((char *)mem_id->mem_ptr + map->offset))
    {
      pocl_montage_read_image_rect (data, src_image, mem_id, map->host_ptr,
                                  NULL, map->origin, map->region,
                                  map->row_pitch, map->slice_pitch, 0);
    }
  return CL_SUCCESS;
}

cl_int pocl_montage_unmap_image(void *data,
                              pocl_mem_identifier *mem_id,
                              cl_mem dst_image,
                              mem_mapping_t *map)
{
  if (map->map_flags == CL_MAP_READ)
    return CL_SUCCESS;

  if (map->host_ptr != ((char *)mem_id->mem_ptr + map->offset))
    {
      pocl_montage_write_image_rect (data, dst_image, mem_id, map->host_ptr,
                                   NULL, map->origin, map->region,
                                   map->row_pitch, map->slice_pitch, 0);
    }
  return CL_SUCCESS;
}

cl_int
pocl_montage_fill_image (void *data, cl_mem image,
                       pocl_mem_identifier *image_data, const size_t *origin,
                       const size_t *region,
                       const void *__restrict__ fill_pixel, size_t pixel_size)
{
   POCL_MSG_PRINT_MEMORY ("montage / FILL IMAGE \n"
                          "image %p data %p \n"
                          "origin [0,1,2] %zu %zu %zu \n"
                          "region [0,1,2] %zu %zu %zu \n"
                          "pixel %p size %zu \n",
                          image, image_data,
                          origin[0], origin[1], origin[2],
                          region[0], region[1], region[2],
                          fill_pixel, pixel_size);

  size_t row_pitch = image->image_row_pitch;
  size_t slice_pitch = image->image_slice_pitch;
  char *__restrict const adjusted_device_ptr
      = (char *)image_data->mem_ptr
        + origin[0] * pixel_size
        + row_pitch * origin[1]
        + slice_pitch * origin[2];

  size_t i, j, k;

  for (k = 0; k < region[2]; ++k)
    for (j = 0; j < region[1]; ++j)
      for (i = 0; i < region[0]; ++i)
        memcpy (adjusted_device_ptr
                  + pixel_size * i
                  + row_pitch * j
                  + slice_pitch * k,
                fill_pixel,
                pixel_size);
  return CL_SUCCESS;
}

void
pocl_montage_svm_free (cl_device_id dev, void *svm_ptr)
{
  /* TODO we should somehow figure out the size argument
   * and call pocl_free_global_mem */
  pocl_aligned_free (svm_ptr);
}

void *
pocl_montage_svm_alloc (cl_device_id dev, cl_svm_mem_flags flags, size_t size)
{
  return pocl_aligned_malloc (MAX_EXTENDED_ALIGNMENT, size);
}

void
pocl_montage_svm_copy (cl_device_id dev, void *__restrict__ dst,
                     const void *__restrict__ src, size_t size)
{
  memcpy (dst, src, size);
}

void
pocl_montage_svm_fill (cl_device_id dev, void *__restrict__ svm_ptr, size_t size,
                     void *__restrict__ pattern, size_t pattern_size)
{
  pocl_mem_identifier temp;
  temp.mem_ptr = svm_ptr;
  pocl_montage_memfill (dev->data, &temp, NULL, size, 0, pattern, pattern_size);
}
