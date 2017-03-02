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
#include "att_ekf.h"

using namespace std;
using namespace cv;

double sa = 0.000244*9.8;
double sg = 0.061035*3.141592653/180.0;
//double sm = 6.0000; 
double sm = 0.006;
sm = 1;

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
  float att[3];
  int32_t presure;
}imu_data_t;

void phase_packet_data(Packet_t *p,imu_data_t *data)
{
  memset(data,0,sizeof(imu_data_t));
  memcpy(data->accl,&p->buf[3],6);
  memcpy(data->gyro,&p->buf[10],6);
  memcpy(data->mag,&p->buf[17],6);
  //memcpy(data->att,&p->buf[24],6);
  data->att[1] = ((float)(int16_t)(p->buf[24] + (p->buf[25]<<8))/100);
  data->att[0] = ((float)(int16_t)(p->buf[26] + (p->buf[27]<<8))/100);
  data->att[2] = ((float)(int16_t)(p->buf[28] + (p->buf[29]<<8))/10);
  return;
}

int main(int argc, char ** argv)
{
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
  const char *eout = "AT+EOUT=0\r\n";
  const char *ein = "AT+EOUT=1\r\n";
  const char *info = "AT+INFO\r\n";
  const char *setF60 = "AT+ODR=60\r\n";
  const char *setF500 = "AT+ODR=500\r\n";
  const char *reset = "AT+RST\r\n";

  int n,a = 100000;
  unsigned char c[64] = {0};

  printf("\n---------disable out----------\n");
  n = write(fd,eout,strlen(eout));
  while(a-- > 0) {
    memset(c,0,sizeof(c));
    if (read(fd,c,64) >0) {
      for(int i=0;i<64;i++)
	printf("%c",c[i]);
    }
  }

  printf("\n---------get info----------\n");
  n = write(fd,info,strlen(info));
  a = 100000;
  while(a--) {
    memset(c,0,sizeof(c));
    if (read(fd,c,64) >0) {
      for(int i=0;i<64;i++)
	printf("%c",c[i]);
    }
  }

  printf("\n---------set 60HZ----------\n");
  n = write(fd,setF60,strlen(setF60));
  a = 100000;
  while(a--) {
    memset(c,0,sizeof(c));
    if (read(fd,c,64) >0) {
      for(int i=0;i<64;i++)
	printf("%c",c[i]);
    }
  }

  printf("\n---------reset----------\n");
  n = write(fd,reset,strlen(reset));
  while(1) {
    memset(c,0,sizeof(c));
    if (read(fd,c,sizeof(c)) >0) {
      for(int i=0;i<64;i++)
	printf("%c",c[i]);
      break;
    }
  }

  printf("\n---------disable out----------\n");
  n = write(fd,eout,strlen(eout));
  a = 100000;
  while(a-- > 0) {
    memset(c,0,sizeof(c));
    if (read(fd,c,64) >0) {
      for(int i=0;i<64;i++)
	printf("%c",c[i]);
    }
  }


  printf("\n---------set 500HZ----------\n");
  n = write(fd,setF500,strlen(setF500));
  a = 100000;
  while(a-- > 0) {
    memset(c,0,sizeof(c));
    if (read(fd,c,sizeof(c)) >0) {
      for(int i=0;i<64;i++)
	printf("%c",c[i]);
    }
  }

  printf("\n---------enable out----------\n");
  n = write(fd,ein,strlen(ein));
  a = 100000;
  while(a-- > 0) {
    memset(c,0,sizeof(c));
    if (read(fd,c,64) >0) {
      for(int i=0;i<64;i++)
	printf("%c",c[i]);
    }
  }

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(fd,&rd);

  Att_ekf filter;

  unsigned char ch;
  while(1) {
    int ret = select(fd+1,&rd,NULL,NULL,NULL);
    if( ret < 0) {
      perror("select error!\n");
      continue;
    } else if (ret == 0) {
      perror("time out!\n");
      continue;
    } 

    if(FD_ISSET(fd,&rd)) {
      int nread = 0;
      while(nread = read(fd,&ch,sizeof(char)) > 0) {
	Packet_t *p = Packet_Decode(ch);
	if(NULL == p)
	  continue;

	unsigned long long timestamp = micros() * 1000;
	imu_data_t imu;
	phase_packet_data(p,&imu);
	char str[128];
	sprintf(str,"%llu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", timestamp,
		imu.gyro[0]*sg, imu.gyro[1]*sg, imu.gyro[2]*sg,
		imu.accl[0]*sa, imu.accl[1]*sa, imu.accl[2]*sa,
		imu.mag[0] *sm, imu.mag[1]* sm, imu.mag[2] *sm,
		imu.att[0], imu.att[1], imu.att[2]);
	if(argc >2)
	  printf(str,"%s",str); 
	
	Eigen::Vector3d acc(imu.accl[0]*sa, imu.accl[1]*sa, imu.accl[2]*sa);
	Eigen::Vector3d gyro(imu.gyro[0]*sg, imu.gyro[1]*sg, imu.gyro[2]*sg);
	Eigen::Vector3d mag(imu.mag[0] *sm, imu.mag[1]* sm, imu.mag[2] *sm);
	//Eigen::Vector3d mag(imu.mag[1] *sm, imu.mag[0]* sm, -imu.mag[2] *sm);
	filter.update(acc,gyro,mag,(double)timestamp/1000000000);
	
      }
    }
  }

  return 0;
}

