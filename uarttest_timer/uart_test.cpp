#include "uart_test.h"

const char* pDevName;
struct uart_param uParam;
int baud_rate;

const char strBuf[BUFSIZE + 1] = "\r\n12345678"; //每次写入串口的数据

int arrSpeed[] = {
    B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int arrName[] = {
    115200, 57600, 38400,  19200,  9600,  4800,  2400,  1200,  300};


struct itimerval newVal, oldVal;
u64 nTimeTotal; // 定时器累计计时时长(us)

u64 nTimeSleep; //预设值, 写串口的时长(s)

bool bStopWrite; //标志位, 是否停止写串口

int hUart; //串口文件描述符
u64 nWriteCount = 0;
u64 nWriteErrCount = 0;
u64 nWriteLostCount = 0;

u64 nBytesWritten = 0;
u64 nBytesLost = 0;

struct timespec tkWriteStart, tkWriteFinish;

void WriteUart(int signo)
{
	long n;
	switch (signo)
	{
	case SIGALRM:
	{
		nTimeTotal += (u64)newVal.it_interval.tv_usec;

		if (nTimeTotal >= nTimeSleep * 1000000) { //达到时间后停止定时器, 并退出
			newVal.it_value.tv_sec = 0;
			newVal.it_value.tv_usec = 0;
			newVal.it_interval.tv_sec = 0;
			newVal.it_interval.tv_usec = 0;
			if (setitimer(ITIMER_REAL, &newVal, &oldVal) != 0) {
				perror("cancel itimer failed");
			}
            printf("nTimeTotal: %llu\n", nTimeTotal);
            PrintDateTime();
			bStopWrite = true;
			return;
		}

		n = write(hUart, strBuf, BUFSIZE);
		if (n == -1) {
			nWriteErrCount++;
		}
		if (n < BUFSIZE) {
			nBytesLost += BUFSIZE - n;
			nWriteLostCount++;
		}
		nWriteCount++;
        nBytesWritten += n;

        //if (nWriteCount % 100 == 0) {
        //    printf("%llu: %llu\n", nWriteCount, nTimeTotal);
        //}

		break;
	}
	}
}


void SetDefaultParams()
{
    baud_rate = 9600;

    uParam.databits = 8;
    uParam.stopbits = 1;
    uParam.parity = 'n';

    nTimeSleep = 10;
    bStopWrite = false;

    nWriteCount = 0;
    nWriteErrCount = 0;
    nWriteLostCount = 0;

    nBytesWritten = 0;
    nBytesLost = 0;

    nTimeTotal = 0;
}

void SetUartBaudrate(int fd, int baud)
{
	unsigned long i;
	int status;
	struct termios Opt;

	tcgetattr(fd, &Opt);
	for (i = 0; i < sizeof(arrSpeed) / sizeof(int); i++) {
		if (baud == arrName[i]) {
			/**
			 * tcflush函数刷清(抛弃)输入缓存(终端驱动程序已接收到，但用户程序尚未读)或输出缓存(用户程序已经写，但尚未发送)。queue参数应是下列三个常数之一：
			 * TCIFLUSH刷清输入队列。
			 * TCOFLUSH刷清输出队列。
			 * TCIOFLUSH刷清输入、输出队列。
			 */
			tcflush(fd, TCIOFLUSH);//设置前flush    

			cfsetispeed(&Opt, arrSpeed[i]);
			cfsetospeed(&Opt, arrSpeed[i]);

			//通过tcsetattr函数把新的属性设置到串口上。
			//tcsetattr(串口描述符，立即使用或者其他标示，指向termios的指针)	
			status = tcsetattr(fd, TCSANOW, &Opt);

			if (status != 0)
			{
				perror("tcsetattr fd1");
				return;
			}
			tcflush(fd, TCIOFLUSH);  //设置后flush
		}
	}
}

int SetUartParam(int fd, struct uart_param param)

{
    int databits = param.databits;
    int stopbits = param.stopbits;
    int parity = param.parity;

    struct termios options;

    if (tcgetattr(fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return false;
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)
        /*设置数据位数*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return false;
    }

    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB; /* Clear parity enable */
        options.c_iflag &= ~INPCK; /* Enable parity checking */
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
        options.c_iflag |= INPCK; /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB; /* Enable parity */
        options.c_cflag &= ~PARODD; /* 转换为偶效验*/
        options.c_iflag |= INPCK; /* Disnable parity checking */
        break;
    case 'S':
    case 's': /*as no parity*/
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return false;
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
        fprintf(stderr, "Unsupported stop bits\n");
        return false;
    }

    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;

    tcflush(fd, TCIFLUSH);
    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */

    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("SetupSerial 3");
        return false;
    }
    return false;

}

void SetTimer(int interval)
{
    newVal.it_value.tv_sec = 0;
    newVal.it_value.tv_usec = 1;
    newVal.it_interval.tv_sec = 0;
    newVal.it_interval.tv_usec = interval;

    printf("interval: %llu\n", u64(newVal.it_interval.tv_usec));
}

//u64 GetTickCountUs()
//{
//    struct timespec ts;
//    clock_gettime(CLOCK_MONOTONIC, &ts);
//
//    return (u64)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000); //返回系统启动到当前运行的微秒数(us)
//}

struct timespec GetTickCountUs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

struct timespec GetTickCountUs_Process()
{
    struct timespec ts;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return ts;
}

struct timespec GetTickCountUs_Thread()
{
    struct timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return ts;
}



// 打印系统当前日期时间, 目前还不能转换到正确的时区
void PrintDateTime()
{
    time_t tt;
    time(&tt);
    tt += 8 * 3600; //转换时区
    tm* t = gmtime(&tt);

    printf("%d-%02d-%02d %02d:%02d:%02d\n",
        t->tm_year + 1900,
        t->tm_mon + 1,
        t->tm_mday,
        t->tm_hour,
        t->tm_min,
        t->tm_sec);
}

int uart_test(const char* devname, int &baudrate, struct uart_param &param)
{
    int bpB; //向串口每发送一个字节, 实际发送的bit数
    int nWriteInterval; //向串口写数据的时间间隔, 需要设定这个值以使得向串口数据的速率与设定的输出波特率相匹配
    bpB = param.databits + param.stopbits + 1;
    nWriteInterval = (BUFSIZE * 1000000) / (baudrate / bpB);

    printf("baudrate:%d, databits:%d, stopbits:%d, parity:%c, nTimeSleep:%llu, nWriteInterval: %d, nTimeTotal: %llu\n", 
        baudrate, param.databits, param.stopbits, param.parity, nTimeSleep, nWriteInterval, nTimeTotal);

    hUart = open(devname, O_RDWR);
    if (hUart == -1) {
        perror("uart open failed");
        return -1;
    }

    // 设置波特率,数据位,停止位,奇偶校验等属性
    SetUartBaudrate(hUart, baudrate);
    SetUartParam(hUart, uParam);

    signal(SIGALRM, WriteUart); // 绑定信号处理函数
    
    SetTimer(nWriteInterval);

    printf("start timer and writing uart: %s...\n", devname);
    PrintDateTime();

    tkWriteStart = GetTickCountUs();
    //struct timespec tkStartProc, tkStopProc, tkStartThread, tkStopThread;
    //tkStartProc = GetTickCountUs_Process();
    //tkStartThread = GetTickCountUs_Thread();
    if (setitimer(ITIMER_REAL, &newVal, &oldVal) == -1)
    {
        perror("setitimer failed");
    }

    while (!bStopWrite)
    {
        // usleep(1000); //睡眠1ms
        // pause();
    }

    tkWriteFinish = GetTickCountUs();
    //tkStopProc = GetTickCountUs_Process();
    //tkStopThread = GetTickCountUs_Thread();
    printf("test finished\n");
    PRINT_TIMESPEC(tkWriteStart);
    PRINT_TIMESPEC(tkWriteFinish);
    printf("tkTotal[%ld]\n", GET_TIMESPEC_INTERVAL(tkWriteStart, tkWriteFinish));
    //printf("tkProc[%ld]\n", GET_TIMESPEC_INTERVAL(tkStartProc, tkStopProc));
    //printf("tkThread[%ld]\n", GET_TIMESPEC_INTERVAL(tkStartThread, tkStopThread));
    printf("nTimeTotal[%llu]\n", nTimeTotal);
    printf("nWriteCount[%llu], nWriteErrCount[%llu], nWriteLostCount[%llu]\n", nWriteCount, nWriteErrCount, nWriteLostCount);
    printf("nBytesWritten[%llu], nBytesLost[%llu]\n", nBytesWritten, nBytesLost);
    printf("RealBaudrate[%llu]\n", nBytesWritten * bpB * 1000 / (nTimeTotal / 1000));


    close(hUart); //关闭串口
    return 0;
}



int main(int argc, char* argv[])
{
    //printf("sizeof(unsigned long): %d\n", sizeof(unsigned long));
    //printf("sizeof(u32): %d, sizeof(u64): %d\n", sizeof(u32), sizeof(u64));
    //unsigned long end = 224536311, start = 2802348433;
    //printf("%lu\n", end - start);
    //u64 tsleep = atoi("36000");
    //u64 tTotal = 1640262204;
    //unsigned long tTotalUL = 1640262204, tsleepUL = atoi("36000");
    //printf("UL: %d\n", tTotalUL >  (tsleepUL * 1000000));
    //printf("u64: %d\n", tTotal > (tsleep * 1000000));
    //unsigned long nBWriteUL = 18897020;
    //u64 nBWrite = 18897020;
    //int b = 10;
    //printf("UL: %lu\n", nBWriteUL * b * 1000 / (tTotalUL / 1000));
    //printf("u64: %llu\n", nBWrite * b * 1000 / (tTotal / 1000));

    //printf("length of u_int64_t: %d; unsigned long: %d, unsigned int: %d\n", sizeof(u_int64_t), sizeof(unsigned long), sizeof(unsigned int));
    //return 0;
    
    //PrintDateTime();
    // 打印帮助信息
    if (argc == 1) {
        printf("  usage: ./test [devname] *[TestTime] *[baudrate] *[databits]\n\n \
                  Existed uart device: (/dev/) ttyS1, ttyS2\n\n \
                  example: ./test /dev/ttyS1 10 9600\n\n");
        return 0;
    }

    // pDevName = "/dev/ttyS1"; // 默认测试串口: 后置PORT1

    SetDefaultParams();

    //参数1: 测试串口设备名称
    if (argc > 1) {
        pDevName = argv[1];
    }

    //参数2: 测试时长
    if (argc > 2) {
        nTimeSleep = atoi(argv[2]);
    }

    // 参数3: 波特率, 默认为9600
    if (argc > 3)
    {
        unsigned long i;
        int tmp = atoi(argv[3]);
        for (i = 0; i < sizeof(arrName); i++) {
            if (arrName[i] == tmp) {
                break;
            }
        }

        if (i == sizeof(arrName)) {
            printf("wrong baud rate.\n");
            return -1;
        }

        baud_rate = tmp;
    }

    // 参数4: 数据位, 默认为8
    if (argc > 4)
    {
        uParam.databits = atoi(argv[4]);
    }

    uart_test(pDevName, baud_rate, uParam);

    return 0;
}