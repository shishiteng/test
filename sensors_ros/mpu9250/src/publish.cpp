#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
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
//double sg = 0.061035;
double sm = 6.0000;

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
  //Eular:x y z (pitch roll yaw)
  data->att[1] = ((float)(int16_t)(p->buf[24] + (p->buf[25]<<8))/100); //pitch
  data->att[0] = ((float)(int16_t)(p->buf[26] + (p->buf[27]<<8))/100); //roll
  data->att[2] = ((float)(int16_t)(p->buf[28] + (p->buf[29]<<8))/10);  //yaw
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

// 由欧拉角创建四元数
//euler:
//q:w x y z
void Euler2Quaternion(float *angle,float *q)
{
  float heading = angle[0];
  float attitude = angle[1];
  float bank = angle[2];

  float c1 = cos(heading/2);
  float s1 = sin(heading/2);
  float c2 = cos(attitude/2);
  float s2 = sin(attitude/2);
  float c3 = cos(bank/2);
  float s3 = sin(bank/2);
  float c1c2 = c1*c2;
  float s1s2 = s1*s2;

  //w,x,y,z
  q[0] = c1c2*c3 - s1s2*s3;
  q[1] =c1c2*s3 + s1s2*c3;
  q[2] =s1*c2*c3 + c1*s2*s3;
  q[3] =c1*s2*c3 - s1*c2*s3;
}


void test(int fd)
{
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
  ros::Publisher mag0_pub = n.advertise<geometry_msgs::Vector3Stamped>("mag0", 1000);
  ros::Publisher pose0_pub = n.advertise<geometry_msgs::PoseStamped>("pose0", 1000);
  
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


  //
  test(fd);

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(fd,&rd);

  unsigned char ch;
  sensor_msgs::Imu imu_msg;
  geometry_msgs::Vector3Stamped mag_msg;
  geometry_msgs::PoseStamped pose_msg;
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
	//fisheye7251和imuV2的time offset是23ms
	ros::Duration offset(0.023061189);
	imu_msg.header.stamp = ros::Time::now() + offset;
	
	imu_msg.angular_velocity.x = imu.gyro[0] * sg;
	imu_msg.angular_velocity.y = imu.gyro[1] * sg;
	imu_msg.angular_velocity.z = imu.gyro[2] * sg;
	imu_msg.linear_acceleration.x = imu.accl[0] * sa;
	imu_msg.linear_acceleration.y = imu.accl[1] * sa;
	imu_msg.linear_acceleration.z = imu.accl[2] * sa;

	mag_msg.header.stamp = imu_msg.header.stamp;
	mag_msg.vector.x = imu.mag[0] * sm;
	mag_msg.vector.y = imu.mag[1] * sm;
	mag_msg.vector.z = imu.mag[2] * sm;

	pose_msg.header.stamp = imu_msg.header.stamp;
	pose_msg.header.frame_id = "/world";
	pose_msg.pose.position.x = 0;
	pose_msg.pose.position.y = 0;
	pose_msg.pose.position.z = 0;
	float a[3],q[4];
	a[0] = imu.att[0]/57.3f;
	a[1] = imu.att[1]/57.3f;
	a[2] = imu.att[2]/57.3f;
	//Euler2Quaternion(imu.att,q);
	Euler2Quaternion(a,q);
	pose_msg.pose.orientation.w = q[0];
	pose_msg.pose.orientation.x = q[1];
	pose_msg.pose.orientation.y = q[2];
	pose_msg.pose.orientation.z = q[3];
	
	imu0_pub.publish( imu_msg);
	mag0_pub.publish( mag_msg);
	pose0_pub.publish( pose_msg);

	ros::spinOnce();

	//loop_rate.sleep();
      }
    }
  }

  return 0;
}
