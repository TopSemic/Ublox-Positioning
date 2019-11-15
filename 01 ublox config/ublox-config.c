/***********************************************
 * @{
 * @file  : ublox_config.c
 * @brief : 
 * @author: Wenxue Wang
 * @email : 
 * @date  : 2019-11-13 
***********************************************/

//--------------------------------------------------
// Copyright (c) 
//--------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <memory.h>
#include <signal.h>

 
#define DEV_NAME  "/dev/ttymxc1"


unsigned char  set_ublox_buard_115200[]={
0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E
 };
 
 unsigned char  set_ublox_buard_9600[]={
0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xA2,0xB5
 };

unsigned char  set_ublox_10hz[]={
 0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12
};

unsigned char  set_ublox_5hz[]={
 0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A
};

unsigned char  set_ublox_1hz[]={
 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39  
};


unsigned char  set_ublox_save[]={
 0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x21,0xAF
};

static int set_port_speed(const int fd, const int speed)
{
	struct termios newtio,oldtio;

	if (tcgetattr(fd,&oldtio) != 0) { //tcgetattr get current termal in oldtio
		//error
		return -1;
	}

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL|CREAD;
	newtio.c_cflag &=~CSIZE;
	newtio.c_cflag |= CS8;
	newtio.c_cflag &= ~PARENB;

	switch(speed) {
	case 9600:
		cfsetispeed(&newtio,B9600);
		cfsetospeed(&newtio,B9600);
		break;
	case 19200:
		cfsetispeed(&newtio,B19200);
		cfsetospeed(&newtio,B19200);
		break;
	case 38400:
		cfsetispeed(&newtio,B38400);
		cfsetospeed(&newtio,B38400);
		break;
	case 115200:
		cfsetispeed(&newtio,B115200);
		cfsetospeed(&newtio,B115200);
		break;
	}
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 6;
	tcflush(fd, TCIFLUSH);
	if((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
		// error
		return -1;
	}
	return 0;
}

void usage(void)
{
    printf("ublox-config :\r\n\
           ublox-config serial rate1 rate2 frequency\r\n\
           serial : serial port \r\n\
           rate1   :original baud rate \r\n\
           rate2     : modified baud rate \r\n\
           frequency   : output frequency \r\n\
Usage : \r\n\
    ublox-config /dev/ttymxc1 9600 115200 10\r\n");
}

int main (int argc, char *argv[])
{
    int fd;
	int len, i,ret;
	char *serial_port;
	int origin_baud_rate,modify_baud_rate;
	int modify_frequency_number;
    //char buf[] = "Hello TopSemic! \n";
	
	//printf("argc = %d\r\n",argc);
	if (argc != 5 )
    {
        usage();
        exit(0);
    }
	
	serial_port = argv[1];  // linux 串口号
    origin_baud_rate = atoi(argv[2]); // ublox 模组初始波特率
	modify_baud_rate = atoi(argv[3]); // 需要修改的波特率  
	modify_frequency_number = atoi(argv[4]); // 需要修改的输出频率
	
    fd = open(serial_port, O_RDWR | O_NOCTTY|O_NDELAY);
    if(fd < 0)
    {
        perror(serial_port);
        return -1;
    }
	
    // 以模块的原始波特率设置处理器串口波特率
	if(set_port_speed(fd, origin_baud_rate) < 0)
		return -3;

	// 修改模块波特率
	if(modify_baud_rate == 115200)
		len = write(fd, set_ublox_buard_115200, sizeof(set_ublox_buard_115200));
	else if(modify_baud_rate == 9600)
		len = write(fd, set_ublox_buard_9600, sizeof(set_ublox_buard_115200));
	
	if (len < 0)
	{
		printf("write data error \n");
	}
	sleep(1);
    close(fd);
    fd = -1;
	
    fd = open(serial_port, O_RDWR | O_NOCTTY|O_NDELAY);
    if(fd < 0)
    {
        perror(serial_port);
        return -1;
    }
	
	// 以修改后的波特率设置处理器串口波特率
	if(set_port_speed(fd, modify_baud_rate) < 0)
		return -3;
	
    // 修改频率
    if(modify_frequency_number == 10)	
		len = write(fd, set_ublox_10hz, sizeof(set_ublox_10hz));
	else if(modify_frequency_number == 5)
		len = write(fd, set_ublox_5hz, sizeof(set_ublox_5hz));
	else if(modify_frequency_number == 1)
		len = write(fd, set_ublox_1hz, sizeof(set_ublox_1hz));
	
	if (len < 0)
	{
		printf("write data error \n");
	}
	sleep(1);
	
	// 保存到flash里,断电有效
    len = write(fd, set_ublox_save, sizeof(set_ublox_save));
	if (len < 0)
	{
		printf("write data error \n");
	}
	sleep(1);
	
	close(fd);
    fd = -1;
    printf("Config successful! Exit ublox-config Proc\n");
    return 0;
}
