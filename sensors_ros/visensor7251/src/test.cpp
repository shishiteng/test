#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <opencv2/opencv.hpp>

#include "device.h"
#include "snTool.h"

using namespace std;
using namespace sn;
using namespace cv;

/******************全局配置 start**********************/
int img_frame_rate = 20;
int imu_frame_rate = 200;
int auto_exposure = 1;
int exposure_time = 3;
int gain = 3;
int key = -1;

void imu_process(device *cam)
{
  imuData imudata;

  while(key != 'q') {
    if(cam->readIMU(&imudata)){
      fprintf(stderr,"%ld %4d %4d %4d %4d %4d %4d %d\n",imudata.imu_timestamp,
	imudata.gyr_x,
	imudata.gyr_y,
	imudata.gyr_z,
	imudata.accel_x,
	imudata.accel_y,
	imudata.accel_z,
	imudata.temperature
    }
  }

}

int main(int argc, char** argv)
{
  //1.open device
  device *cam = new device();
  if (!cam->init()) {
    return 0;
  }


  //2.device settings
  //帧率、自动曝光配置
  cam->config(img_frame_rate,imu_frame_rate,max_exposure_time,min_exposure_time,cur_exposure_time);

  // imu线程
  std::thread thread_imu{imu_process, cam};

  // 图像设置
  resolution rn = cam->getImageSize();
  Mat left8u = Mat(Size(rn.width, rn.height), CV_8UC1);
  Mat right8u = Mat(Size(rn.width, rn.height), CV_8UC1);
  Mat stereoFrame(Size(rn.width*2, rn.height), CV_8UC1);
  uint64 imgTimeStamp = 0;  

  // start capture image
  cam->start();

  //main loop
  while (key != 'q') {
    if((cam->QueryFrame(left8u, right8u, imgTimeStamp)) {
      imshow("left",left8u);
      imshow("right",right8u);
    }
    key = waitKey(10) & 0xffff;
    //printf("image  :%lf,   %lf\n",timestamp.toSec(),(double)imgTimeStamp/100000000);
  }

  
  //等待线程结束
  thread_imu.join();
  cerr<<"exit imu thread."<<endl;
    
  cam->close();
  delete cam;
  cerr<<"close camera."<<endl;

  //等待1ms
  usleep(1000);
  cerr<<"exit."<<endl;

  return 0;
}
