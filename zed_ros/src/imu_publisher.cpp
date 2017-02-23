#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
//#include "serial/serial.h"

#include "packet.h"
#include "serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

double sa = 0.000244*9.8;
double sg = 0.061035*3.141592653/180.0;

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

unsigned long long nanosec()
{
  struct timespec time_start={0, 0},time_end={0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);    //有4组稍微大于7或者小于3的
  //clock_gettime(CLOCK_MONOTONIC, &time_start); //有很多组间隔小于1的
  //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time_start); //很多组大于10的
  //clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time_start);
  unsigned long long ns = (unsigned long long)(time_start.tv_sec * 1000000000 + time_start.tv_nsec);

  return ns;
}

void my_sleep(unsigned long milliseconds) {

  usleep(milliseconds*1000); // 100 ms

}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  srand (time(NULL));

  ros::init(argc, argv, "imu_publisher");

  ros::NodeHandle n;
  ros::Publisher imu0_pub = n.advertise<sensor_msgs::Imu>("imu0", 1000);


  
  /*Open serial port*/
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

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(fd,&rd);

  unsigned char ch;
  sensor_msgs::Imu imu_msg;
  //ros::Rate loop_rate(100);


  while(n.ok()) {
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
      while( nread=read(fd,&ch,sizeof(char)) > 0) {
	Packet_t *p = Packet_Decode(ch);
	if(NULL == p)
	  continue;
	imu_data_t imu;
	phase_packet_data(p,&imu);
	char str[128];
	sprintf(str,"%llu,%lf,%lf,%lf,%lf,%lf,%lf\n", nanosec(),
		imu.gyro[0]*sg, imu.gyro[1]*sg, imu.gyro[2]*sg,
		imu.accl[0]*sa, imu.accl[1]*sa, imu.accl[2]*sa);
      
	//printf(str,"%s",str); 

	imu_msg.header.stamp = ros::Time::now();
	imu_msg.angular_velocity.x = imu.gyro[0] * sg;
	imu_msg.angular_velocity.y = imu.gyro[1] * sg;
	imu_msg.angular_velocity.z = imu.gyro[2] * sg;
	imu_msg.linear_acceleration.x = imu.accl[0] * sa;
	imu_msg.linear_acceleration.y = imu.accl[1] * sa;
	imu_msg.linear_acceleration.z = imu.accl[2] * sa;
     
	imu0_pub.publish( imu_msg);

	ros::spinOnce();

	//loop_rate.sleep();
      }
    }
  }

  return 0;
}
