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
bool frame_update_flag=0;
bool cycle_update_flag=0;

float ranges_buff[240];
float intensities_buff[240];

void *myreadframe_thread(void *pt)
{
	while(1)
        {	
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

for(int i=0;i<11;i++)
printf("%0x",readbuff[i]);
printf("\n");	
		if(readbuff[0]==0x59 && readbuff[1]==0x59)
		{       
			unsigned short checksum=0;  
			for(int j=0;j<10;j++)
			{				
				checksum+=readbuff[j];
			}  
			if( ((checksum&0x00ff)==readbuff[10]) )     		
			{	
				Dist=readbuff[2]+readbuff[3]*256;
				Strength=readbuff[4]+readbuff[5]*256;
				Angle=readbuff[6]+readbuff[7]*256;
				if(Strength>=20)
				{	
					frame_update_flag=1;	;				
				}
			}
		}
	}
}

void *myreadcycle_thread(void *pt)
{
	//init cycle data
	for(int i=0;i<240;i++)		
	{
		ranges_buff[i]=std::numeric_limits<float>::infinity();
		intensities_buff[i]=0.0;
	}
	//init discard uncomplete head
	while(1)
	{
		if(frame_update_flag==1)
		{
			if(Angle==3600) break;

			frame_update_flag=0;
		}
	}
	
	//update cycle data
	while(1)
	{	
		if(frame_update_flag==1)
		{
			if(cycle_update_flag==0)	
				if( (Angle%15==0)&&(Angle>0)&&(Angle<=3600) )
				{	
					if(Dist==0)
						ranges_buff[Angle/15-1]=std::numeric_limits<float>::infinity();	
					else
						ranges_buff[Angle/15-1]=(float)(Dist)/100.0;

					intensities_buff[Angle/15-1]=(float)(Strength);
				
					if(Angle==3600) cycle_update_flag=1;				
				}
			frame_update_flag=0;
		}
	}
}

int main(int argc, char** argv)
{
    	ros::init(argc, argv, "laser_scan_publisher"); 
    	ros::NodeHandle n;
    	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 100);

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

	pthread_t id_1,id_2;
	int ret1=pthread_create(&id_1,NULL,*myreadframe_thread,NULL);
	int ret2=pthread_create(&id_2,NULL,*myreadcycle_thread,NULL);

    	while(n.ok())
    	{
		
		if(cycle_update_flag==1)
		{
		//populate the LaserScan message
       		sensor_msgs::LaserScan scan;
		
       		ros::Time time_start = ros::Time::now();
		scan.ranges.resize(240);
       		scan.intensities.resize(240);
       		for(int i = 0; i < 240; i++)
		{
         		scan.ranges[i] = ranges_buff[i];
         		scan.intensities[i] = intensities_buff[i];	
       		}
		ros::Time time_end = ros::Time::now();

       		scan.header.stamp = time_start;
		scan.header.frame_id = "base_laser_link";

       		scan.angle_min = 1.5/180.0*3.141593;
       		scan.angle_max = 360.0/180.0*3.141593;
       		scan.angle_increment = (scan.angle_max - scan.angle_min)/(240-1);

       		scan.time_increment = (time_end-time_start).toSec()*(1e-3)/(240-1);
       		scan.scan_time= (time_end-time_start).toSec()*(1e-3);

       		scan.range_min = 0.2;
       		scan.range_max = 10.0;

       		scan_pub.publish(scan);
		
		//clear cycle data
		/*
		for(int i = 0; i < 240; i++)
		{
			ranges_buff[i]=std::numeric_limits<float>::infinity();
			intensities_buff[i]=0.0;
		}
		*/
		cycle_update_flag=0;	
		}
		
	}

	close(SerialCom);
	exit(0);
}




