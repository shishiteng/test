#include <stdio.h>
#include <thread>

#include <opencv2/opencv.hpp>
#include "device.h"

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
int showimu = 0;

void imu_process(device *cam)
{
  imuData imudata;

  
  while(key != 'q') {
    if(cam->readIMU(&imudata)){
      if(showimu) {
        fprintf(stderr,"%lld %4d %4d %4d %4d %4d %4d %d\n",imudata.imu_timestamp,
	      imudata.gyr_x,
	      imudata.gyr_y,
	      imudata.gyr_z,
	      imudata.accel_x,
	      imudata.accel_y,
	      imudata.accel_z,
	      imudata.temperature);
      }
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
  
  int cur_exposure_time = 0x1df0;
  int max_exposure_time = 0x1c20; //1c2*20us = 9ms
  int min_exposure_time = 0x1;
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
    if(cam->QueryFrame(left8u, right8u, imgTimeStamp)) {
      imshow("left",left8u);
      imshow("right",right8u);
    }
    key = waitKey(10) & 0xffff;
    if(key == 's')
      showimu ^= 1;
  }


  //等待线程结束
  thread_imu.join();
  cerr<<"exit imu thread."<<endl;

  cam->close();
  delete cam;
  cerr<<"close camera."<<endl;

  //等待1ms
  //usleep(1000);
  cerr<<"exit."<<endl;

  return 0;
}

