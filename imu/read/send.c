#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <unistd.h>
#include <thread>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "packet.h"
#include "serial.h"

using namespace std;
using namespace cv;

double sa = 0.000244*9.8;
double sg = 0.061035*3.141592653/180.0;

unsigned long long gstime = 0;

unsigned long long micros() 
{
  struct timeval dwTime;
  gettimeofday(&dwTime, NULL);
  unsigned long long us = (unsigned long long)(1000000 * dwTime.tv_sec + dwTime.tv_usec);

  return us;
}

typedef struct imu_data
{
  int16_t accl[3];
  int16_t gyro[3];
  int16_t mag[3];
  uint16_t yaw;
  int16_t pitch;
  int16_t roll;
  int32_t presure;
}imu_data_t;

void phase_packet_data(Packet_t *p,imu_data_t *data)
{
  memset(data,0,sizeof(imu_data_t));
  memcpy(data->accl,&p->buf[3],6);
  memcpy(data->gyro,&p->buf[10],6);
  //memcpy(data->mag,&buf[23],6);
  return;
}

void send_command(int fd,char *cmd)
{
  char str[128] = {0};
  sprintf(str,"%s\r\n",cmd);
  printf("%s",str);

  int n = write(fd,str,strlen(str));
  unsigned char c;
  while(1) {
    if (read(fd,&c,1) >0) {
      printf("%c",c);
    }
  }
}

int main(int argc, char ** argv)
{
  if(argc < 2) {
      printf("command:\n\
      \tAT+EOUT=0     关闭输出\n\
      \tAT+EOUT=1     打开输出\n\
      \tAT+INFO       模块信息\n\
      \tAT+ODR        设置帧率\n\
      \tAT+RST        复位\n");
      return -1;
  }

  //open serial port
  char *dev ="/dev/ttyUSB0";
  int fd = open_dev(dev);
  if (fd>0)
    set_speed(fd,460800);
  else {
    printf("Can't Open Serial Port!\n");
    return -1;
  }

  if (set_parity(fd,8,1,'N') == false) {
    printf("Set Parity Error\n");
    return -1;
  }


  //write 500HZ
  send_command(fd,argv[1]);

}

