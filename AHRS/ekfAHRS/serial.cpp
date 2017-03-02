#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>   /**/
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX终端控制定义*/
#include     <errno.h>      /*错误号定义*/

#include <stdint.h>
#include <stdbool.h>

/***@brief  设置串口通信速率
 *@param  fd     类型 int  打开串口的文件句柄
 *@param  speed  类型 int  串口速度
 *@return  void*/

int speed_arr[] = {B460800, B115200,  B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[] = { 460800, 115200, 38400,  19200,  9600,  4800,  2400,  1200,  300};

void set_speed(int fd, int speed)
{
  int   i;
  int   status;
  struct termios   Opt;
  int ret = -1;

  ret = tcgetattr(fd, &Opt);
  if(ret < 0) {
    printf("Unable to get the attribute.\n");
    return;
  }

  for ( i= 0;  i < sizeof(speed_arr)/sizeof(int);  i++) {
    if  (speed == name_arr[i]) {
      tcflush(fd, TCIOFLUSH);
      cfsetispeed(&Opt, speed_arr[i]);
      cfsetospeed(&Opt, speed_arr[i]);
      status = tcsetattr(fd, TCSANOW, &Opt);
      if  (status != 0)
	perror("tcsetattr fd1");
      return;
    }
    tcflush(fd,TCIOFLUSH);
  }
}

/**
 *@brief   设置串口数据位，停止位和效验位
 *@param  fd     类型  int  打开的串口文件句柄*
 *@param  databits 类型  int 数据位   取值 为 7 或者8*
 *@param  stopbits 类型  int 停止位   取值为 1 或者2*
 *@param  parity  类型  int  效验类型 取值为N,E,O,,S
 */
int set_parity(int fd,int databits,int stopbits,int parity)
{
  struct termios options;
  if  ( tcgetattr( fd,&options)  !=  0)
    {
      perror("SetupSerial 1");
      return(false);
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
      return (false);
    }
  switch (parity)
    {
    case 'n':
    case 'N':
      //control
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~HUPCL;
      //input
      options.c_iflag &= ~IGNBRK;
      options.c_iflag &= ~ICRNL;
      options.c_iflag &= ~IXON;
      //out put
      options.c_oflag &= ~OPOST; 
      options.c_oflag &= ~ONLCR;
      //local mode
      options.c_lflag &= ~ISIG;
      options.c_lflag &= ~ICANON;
      options.c_lflag &= ~IEXTEN;
      options.c_lflag &= ~ECHO;
      options.c_lflag &= ~ECHOE;
      options.c_lflag &= ~ECHOK;
      options.c_lflag &= ~ECHOCTL;
      options.c_lflag &= ~ECHOKE;
      
    case 'o':
    case 'O':
      options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/ 
      options.c_iflag |= INPCK;             /* Disnable parity checking */
      break;
    case 'e':
    case 'E':
      options.c_cflag |= PARENB;     /* Enable parity */
      options.c_cflag &= ~PARODD;   /* 转换为偶效验*/  
      options.c_iflag |= INPCK;       /* Disnable parity checking */
      break;
    case 'S':
    case 's':  /*as no parity*/
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    default:
      fprintf(stderr,"Unsupported parity\n");
      return (false);
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
      return (false);
    }
  /* Set input parity option */
  if (parity != 'n')
    options.c_iflag |= INPCK;
  //options.c_cc[VTIME] = 150; // 15 seconds
  //options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 5;
  options.c_cc[VMIN] = 1;

  tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
      perror("SetupSerial 3");
      return (false);
    }
  return (true);
}


int open_dev(char *dev)
{
  int fd = open( dev, O_RDWR | O_NOCTTY | O_NDELAY );         //| O_NOCTTY | O_NDELAY
  if (-1 == fd)
    { /*设置数据位数*/
      perror("Can't Open Serial Port");
      return -1;
    }
  else
    return fd;

}

int main_(int argc, char **argv)
{
  int i = 0;
  int fd;
  int nread;
  unsigned char buff[128] = {0};

  char *dev ="/dev/ttyUSB0";
  fd = open_dev(dev);
  if (fd>0)
    set_speed(fd,460800);
  else {
    printf("Can't Open Serial Port!\n");
    exit(0);
  }

  if (set_parity(fd,8,1,'N')== false) {
    printf("Set Parity Error\n");
    exit(1);
  }

  int a = 0;
  unsigned char buff2[1024] = {0};
  while(1) {
    while((nread = read(fd,buff,sizeof(buff)))>0) {
      buff[nread] = '\0';
      printf("len:%d  ",nread);
      for(i=0;i<nread;i++)
	printf("%x",buff[i]);
      printf("\n");
    }

    
  }

  close(fd);
  return 0;
}
