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

#include "MadgwickAHRS.h"

using namespace std;
using namespace cv;

double sa = 0.000244*9.8;
double sg = 0.061035*3.141592653/180.0;
//double sg = 0.061035;

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

  unsigned char ch;
  while(1) {
    if(read(fd,&ch,sizeof(char)) > 0) {
      Packet_t *p = Packet_Decode(ch);
      if(NULL == p)
	continue;
      unsigned long long timestamp = micros() * 1000;
      imu_data_t imu;
      phase_packet_data(p,&imu);
#if 0
      char str[128];
      sprintf(str,"%llu,%lf,%lf,%lf,%lf,%lf,%lf\n", timestamp,
	      imu.gyro[0]*sg, imu.gyro[1]*sg, imu.gyro[2]*sg,
	      imu.accl[0]*sa, imu.accl[1]*sa, imu.accl[2]*sa);
      printf(str,"%s",str); 
#endif
      //filter.updateIMU(gx, gy, gz, ax, ay, az);
      MadgwickAHRSupdateIMU(imu.gyro[0]*sg, imu.gyro[1]*sg, imu.gyro[2]*sg,
		       imu.accl[0]*sa, imu.accl[1]*sa, imu.accl[2]*sa);
      // print the heading, pitch and roll
      computeAngles();
      printf("R/P/Y:%lf,%lf,%lf\n",roll,pitch,yaw); 
    }
  }

  return 0;
}
