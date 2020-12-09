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

// ��߳������ֲ��: 32λ������64λ����
#define u32 uint32_t
#define u64 uint64_t



#define BUFSIZE 10

// ��ӡtimespec�ṹ����, ��ȷ��us
#define PRINT_TIMESPEC(ts) printf("%ld(s)%ld(us)\n", ts.tv_sec, ts.tv_nsec/1000) 
// ��������timespec�ṹ����֮��������us
#define GET_TIMESPEC_INTERVAL(ts1, ts2) (((ts2.tv_sec-ts1.tv_sec)*1000000) + ((ts2.tv_nsec-ts1.tv_nsec)/1000))

struct uart_param
{
	int databits;
	int stopbits;
	char parity;
};

/*========================================================================================*/

// �жϴ���ص�����, ��ʱ�ﵽʱ�����һ��SIGALRMʱ���ж�, ������ж�ʱ, ִ��д���ڶ���
void WriteUart(int signo);

void SetDefaultParams();

// ���ô��ڲ�����
void SetUartBaudrate(int fd, int baud);
// ���ô�������λ, ֹͣλ, ��żУ�������
int SetUartParam(int fd, struct uart_param param);

void SetTimer(int interval);


//unsigned long GetTickCountUs();
struct timespec GetTickCountUs();
void PrintDateTime();

int uart_test(const char* devname, int &baudrate, struct uart_param &param);