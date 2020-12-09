#include "uart_test.h"

const char* pDevName;
struct uart_param uParam;
int baud_rate;

const char strBuf[BUFSIZE + 1] = "\r\n12345678"; //ÿ��д�봮�ڵ�����

int arrSpeed[] = {
    B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int arrName[] = {
    115200, 57600, 38400,  19200,  9600,  4800,  2400,  1200,  300};


struct itimerval newVal, oldVal;
u64 nTimeTotal; // ��ʱ���ۼƼ�ʱʱ��(us)

u64 nTimeSleep; //Ԥ��ֵ, д���ڵ�ʱ��(s)

bool bStopWrite; //��־λ, �Ƿ�ֹͣд����

int hUart; //�����ļ�������
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

		if (nTimeTotal >= nTimeSleep * 1000000) { //�ﵽʱ���ֹͣ��ʱ��, ���˳�
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
			 * tcflush����ˢ��(����)���뻺��(�ն����������ѽ��յ������û�������δ��)���������(�û������Ѿ�д������δ����)��queue����Ӧ��������������֮һ��
			 * TCIFLUSHˢ��������С�
			 * TCOFLUSHˢ��������С�
			 * TCIOFLUSHˢ�����롢������С�
			 */
			tcflush(fd, TCIOFLUSH);//����ǰflush    

			cfsetispeed(&Opt, arrSpeed[i]);
			cfsetospeed(&Opt, arrSpeed[i]);

			//ͨ��tcsetattr�������µ��������õ������ϡ�
			//tcsetattr(����������������ʹ�û���������ʾ��ָ��termios��ָ��)	
			status = tcsetattr(fd, TCSANOW, &Opt);

			if (status != 0)
			{
				perror("tcsetattr fd1");
				return;
			}
			tcflush(fd, TCIOFLUSH);  //���ú�flush
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
        /*��������λ��*/
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
        options.c_cflag |= (PARODD | PARENB); /* ����Ϊ��Ч��*/
        options.c_iflag |= INPCK; /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB; /* Enable parity */
        options.c_cflag &= ~PARODD; /* ת��ΪżЧ��*/
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

    /* ����ֹͣλ*/
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
    options.c_cc[VTIME] = 150; /* ���ó�ʱ15 seconds*/
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
//    return (u64)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000); //����ϵͳ��������ǰ���е�΢����(us)
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



// ��ӡϵͳ��ǰ����ʱ��, Ŀǰ������ת������ȷ��ʱ��
void PrintDateTime()
{
    time_t tt;
    time(&tt);
    tt += 8 * 3600; //ת��ʱ��
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
    int bpB; //�򴮿�ÿ����һ���ֽ�, ʵ�ʷ��͵�bit��
    int nWriteInterval; //�򴮿�д���ݵ�ʱ����, ��Ҫ�趨���ֵ��ʹ���򴮿����ݵ��������趨�������������ƥ��
    bpB = param.databits + param.stopbits + 1;
    nWriteInterval = (BUFSIZE * 1000000) / (baudrate / bpB);

    printf("baudrate:%d, databits:%d, stopbits:%d, parity:%c, nTimeSleep:%llu, nWriteInterval: %d, nTimeTotal: %llu\n", 
        baudrate, param.databits, param.stopbits, param.parity, nTimeSleep, nWriteInterval, nTimeTotal);

    hUart = open(devname, O_RDWR);
    if (hUart == -1) {
        perror("uart open failed");
        return -1;
    }

    // ���ò�����,����λ,ֹͣλ,��żУ�������
    SetUartBaudrate(hUart, baudrate);
    SetUartParam(hUart, uParam);

    signal(SIGALRM, WriteUart); // ���źŴ�����
    
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
        // usleep(1000); //˯��1ms
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


    close(hUart); //�رմ���
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
    // ��ӡ������Ϣ
    if (argc == 1) {
        printf("  usage: ./test [devname] *[TestTime] *[baudrate] *[databits]\n\n \
                  Existed uart device: (/dev/) ttyS1, ttyS2\n\n \
                  example: ./test /dev/ttyS1 10 9600\n\n");
        return 0;
    }

    // pDevName = "/dev/ttyS1"; // Ĭ�ϲ��Դ���: ����PORT1

    SetDefaultParams();

    //����1: ���Դ����豸����
    if (argc > 1) {
        pDevName = argv[1];
    }

    //����2: ����ʱ��
    if (argc > 2) {
        nTimeSleep = atoi(argv[2]);
    }

    // ����3: ������, Ĭ��Ϊ9600
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

    // ����4: ����λ, Ĭ��Ϊ8
    if (argc > 4)
    {
        uParam.databits = atoi(argv[4]);
    }

    uart_test(pDevName, baud_rate, uParam);

    return 0;
}