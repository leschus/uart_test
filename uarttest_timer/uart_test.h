#pragma once
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <stdint.h>

#define UARTTEST_ITIMER_VER "2020_1204"

// 提高程序可移植性: 32位机器和64位机器
#define u32 uint32_t
#define u64 uint64_t



#define BUFSIZE 10

// 打印timespec结构变量, 精确到us
#define PRINT_TIMESPEC(ts) printf("%ld(s)%ld(us)\n", ts.tv_sec, ts.tv_nsec/1000) 
// 计算两个timespec结构变量之间相差多少us
#define GET_TIMESPEC_INTERVAL(ts1, ts2) (((ts2.tv_sec-ts1.tv_sec)*1000000) + ((ts2.tv_nsec-ts1.tv_nsec)/1000))

struct uart_param
{
	int databits;
	int stopbits;
	char parity;
};

/*========================================================================================*/

// 中断处理回调函数, 定时达到时会产生一个SIGALRM时钟中断, 处理该中断时, 执行写串口动作
void WriteUart(int signo);

void SetDefaultParams();

// 设置串口波特率
void SetUartBaudrate(int fd, int baud);
// 设置串口数据位, 停止位, 奇偶校验等属性
int SetUartParam(int fd, struct uart_param param);

void SetTimer(int interval);


//unsigned long GetTickCountUs();
struct timespec GetTickCountUs();
void PrintDateTime();

int uart_test(const char* devname, int &baudrate, struct uart_param &param);