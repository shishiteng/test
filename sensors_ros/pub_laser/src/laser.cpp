#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include <fstream>//for file save
#include <iostream>//std::cout
#include <pthread.h>
#include <limits>

int SerialCom;
unsigned char readbuff[11];
unsigned char insert_buf;
int wread = 0;

unsigned short Dist;
unsigned short Angle;
unsigned short Strength;

float ranges_buff[240];
float intensities_buff[240];


int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_scan_publisher"); 
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("base_scan", 100);

  ///////////////////////////////////////////////////////////////////
  const char *COMName = "/dev/ttyUSB0";
  SerialCom = open(COMName, O_RDWR);	//设置串口名
  if(SerialCom == -1)
    {
      printf("Can't open serial port!\n");
    }
  //////////////////////////////////////////////////////////////////
  struct termios options;
  if(tcgetattr(SerialCom, &options) != 0)
    {
      printf("Can't get serial port sets!\n");
    }

  tcflush(SerialCom, TCIFLUSH);
  cfsetispeed(&options, B115200);	//设置串口接受比特率
  cfsetospeed(&options, B115200);	//设置串口发送比特率
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
  options.c_oflag &= ~(ONLCR | OCRNL);

  if(tcsetattr(SerialCom, TCSANOW, &options) != 0)
    {
      printf("Can't set serial port options!\n");
    }
  //////////////////////////////////////////////////////////////////
  unsigned char writebuff[] = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x02, 0x02};
  printf("%02x %02x %02x %02x %02x %02x %02x %02x\n", writebuff[0], writebuff[1], writebuff[2], writebuff[3], writebuff[4], writebuff[5], writebuff[6], writebuff[7]);

  wread = write(SerialCom, writebuff, sizeof(writebuff));
  printf("%d\n", wread);

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(SerialCom,&rd);

  while(n.ok()) {
    int ret = select(SerialCom+1,&rd,NULL,NULL,NULL);
    if( ret < 0) {
      perror("select error!\n");
      continue;
    } else if (ret == 0) {
      perror("time out!\n");
      continue;
    } 

    if(FD_ISSET(SerialCom,&rd)) {
      wread=read(SerialCom,&insert_buf,1);
      readbuff[0]=readbuff[1];
      readbuff[1]=readbuff[2];
      readbuff[2]=readbuff[3];
      readbuff[3]=readbuff[4];
      readbuff[4]=readbuff[5];
      readbuff[5]=readbuff[6];
      readbuff[6]=readbuff[7];
      readbuff[7]=readbuff[8];
      readbuff[8]=readbuff[9];
      readbuff[9]=readbuff[10];
      readbuff[10]=insert_buf;

      if(readbuff[0]==0x59 && readbuff[1]==0x59) {       
	unsigned short checksum=0;  
	for(int j=0;j<10;j++) {				
	  checksum+=readbuff[j];
	}  
	if( ((checksum&0x00ff)==readbuff[10]) ) {	
	  Dist=readbuff[2]+readbuff[3]*256;
	  Strength=readbuff[4]+readbuff[5]*256;
	  Angle=readbuff[6]+readbuff[7]*256;

	  if(Strength < 6)
	    continue;

	  if( (Angle%15==0)&&(Angle>0)&&(Angle<=3600) ) {	
	    if(Dist==0)
	      ranges_buff[Angle/15-1]=std::numeric_limits<float>::infinity();	
	    else
	      ranges_buff[Angle/15-1]=(float)(Dist)/100.0;

	    intensities_buff[Angle/15-1]=(float)(Strength);
	  }

	  if(Angle == 3600) {
	    //populate the LaserScan message
	    sensor_msgs::LaserScan scan;
		
	    ros::Time time_start = ros::Time::now();
	    scan.ranges.resize(240);
	    scan.intensities.resize(240);
#if 0
	    for(int i = 0; i < 240; i++) {
	      scan.ranges[i] = ranges_buff[i];
	      scan.intensities[i] = intensities_buff[i];	
	    }
#endif
	    memcpy((void*)&scan.ranges[0],(void*)ranges_buff,sizeof(float)*240);
	    memcpy((void*)&scan.intensities[0],(void*)intensities_buff,sizeof(float)*240);

	    scan.header.stamp = time_start;
	    scan.header.frame_id = "world";

	    scan.angle_min = 1.5/180.0*3.141593;
	    scan.angle_max = 360.0/180.0*3.141593;
	    scan.angle_increment = (scan.angle_max - scan.angle_min)/(240-1);

	    scan.time_increment = 1/4/(240-1)*1000;
	    //scan.scan_time= (time_end-time_start).toSec()*(1e-3);

	    scan.range_min = 0.0;
	    scan.range_max = 10.0;

	    //printf(".");
	    scan_pub.publish(scan);

	    memset((void*)ranges_buff,0,sizeof(ranges_buff));
	    memset((void*)intensities_buff,0,sizeof(intensities_buff));
		
	    //clear cycle data
	    /*
	      for(int i = 0; i < 240; i++)
	      {
	      ranges_buff[i]=std::numeric_limits<float>::infinity();
	      intensities_buff[i]=0.0;
	      }
	    */
	  }
	}
      }
    }
  }
  close(SerialCom);
  exit(0);
}




