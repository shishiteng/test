#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "device.h"
#include "snTool.h"

using namespace std;
using namespace sn;
using namespace cv;


#define BASESEC 10000000
#define toRosTime(x) (ros::Time(BASESEC+x/100000000,(x-x/100000000*100000000)*10))


typedef struct{
  double timestamp;
  Mat left;
  Mat right;
}imageData;

//imu0:完全标定过的（温漂补偿、bias补偿、尺度标定、轴间对齐）
//bias temperature scale axis
ros::Publisher imu_pub;
ros::Publisher imu_pub_b;
ros::Publisher imu_pub_tb;
ros::Publisher imu_pub_tbs;

ros::Publisher img_pub0;
ros::Publisher img_pub1;
//ros::Publisher img_pub2;


image_transport::Publisher img_pub2;


/******************全局配置 start**********************/
int img_frame_rate = 20;
int imu_frame_rate = 200;
int auto_exposure = 1;
int exposure_time = 3;
int gain = 3;

//温漂补偿、常数噪声补偿、尺度偏差、轴间偏差
int temperature_compensation = 1;
int bias_correction = 1;
int scale_correction = 1;
int axis_alignment = 1;

//校正公式：a = T*K*(a_meas - a_bias - a_tempcompesation)
Mat tempdriftscale_acc,bias_acc,scale_acc,alignment_acc;
Mat tempdriftscale_gyr,bias_gyr,scale_gyr,alignment_gyr;

int cur_exposure_time = 0x1df0;
int max_exposure_time = 0x1f60;
int min_exposure_time = 0x1;
/******************全局配置 end**********************/


void registerPub(ros::NodeHandle &n)
{
  imu_pub = n.advertise<sensor_msgs::Imu>("imu0", 100);

  //不能使用image_transport,退出时ros会崩溃
  img_pub0 = n.advertise<sensor_msgs::Image>("cam0/image_raw", 1);
  img_pub1 = n.advertise<sensor_msgs::Image>("cam1/image_raw", 1);

  image_transport::ImageTransport it(n);
  img_pub2 = it.advertise("stereo_remap/image_raw", 1);
}


int readImuConfig(char *config_file)
{
  cout << "---config settings---"<<endl;
  FileStorage fsettings(string(config_file), FileStorage::READ);
  if(!fsettings.isOpened()) {
    fprintf(stderr,"打开配置文件失败:%s\n",config_file);
    return -1;
  }

  img_frame_rate = (int)fsettings["img_frame_rate"];
  cout << " img_frame_rate:"<<img_frame_rate <<endl;
  imu_frame_rate = (int)fsettings["imu_frame_rate"];
  cout << " imu_frame_rate:"<<imu_frame_rate <<endl;
  auto_exposure = (int)fsettings["auto_exposure"];
  cout << " auto_exposure:"<<auto_exposure <<endl;
  exposure_time = (int)fsettings["exposure_time"];
  cout << " exposure_time:"<<exposure_time <<endl;
  gain = (int)fsettings["gain"];
  cout << " gain:"<<gain <<endl;

  temperature_compensation = (int)fsettings["temperature_compensation"];
  cout << " temperature_compensation:"<< temperature_compensation <<endl;
  if(temperature_compensation) {
    fsettings["tempdriftscale_acc"] >> tempdriftscale_acc;
    fsettings["tempdriftscale_gyr"] >> tempdriftscale_gyr;
  } else {
    tempdriftscale_acc = Mat::zeros(3,1,CV_64FC1);
    tempdriftscale_gyr = Mat::zeros(3,1,CV_64FC1);
  }

  bias_correction = (int)fsettings["bias_correction"];
  cout<< " bias_correction:" << bias_correction <<endl;
  if(bias_correction) {
    fsettings["bias_acc"] >> bias_acc;
    fsettings["bias_gyr"] >> bias_gyr;
  } else {
    bias_acc = Mat::zeros(3,1,CV_64FC1);
    bias_gyr = Mat::zeros(3,1,CV_64FC1);
  }

  scale_correction = (int)fsettings["scale_correction"];
  cout<< " scale_correction:" << scale_correction <<endl;
  if(scale_correction) {
    fsettings["Ka"] >> scale_acc;
    fsettings["Kg"] >> scale_gyr;    
  } else {
    // imu数据比例系数
    //double accS = 0.000244141 * 9.8 = 0.002392582; // m/s^2,量程+-8g
    //double gyrS = 0.06103515625*3.141592653/180.0 = 0.001065264; // rad/s
    scale_acc = Mat::eye(3,3,CV_64FC1) * 0.000244141 * 9.8;
    scale_gyr = Mat::eye(3,3,CV_64FC1) * 0.06103515625 * 3.141592653 / 180.0;
  }

  axis_alignment = (int)fsettings["axis_alignment"];
  cout<< " axis_alignment:" << axis_alignment <<endl;
  if(axis_alignment) {
    fsettings["Ta"] >> alignment_acc;
    fsettings["Tg"] >> alignment_gyr;
  } else {
    alignment_acc = Mat::eye(3,3,CV_64FC1);
    alignment_gyr = Mat::eye(3,3,CV_64FC1);
  }

  cout <<" tempdriftscale_acc:\n    "<<tempdriftscale_acc.t()<<endl;
  cout <<" tempdriftscale_gyr:\n    "<<tempdriftscale_gyr.t()<<endl;
  cout <<" bias_acc:\n    "<<bias_acc.t()<<endl;
  cout <<" bias_gyr:\n    "<<bias_gyr.t()<<endl;
  cout <<" scale_acc:\n"<<scale_acc<<endl;
  cout <<" scale_gyr:\n"<<scale_gyr<<endl;
  cout <<" alignment_acc:\n"<<alignment_acc<<endl;
  cout <<" alignment_gyr:\n"<<alignment_gyr<<endl;
  cout << "-----------------\n"<<endl;

  return 0;
}

int readImageConfig(char *config_file,Mat &remapX0, Mat &remapY0, Mat &remapX1, Mat &remapY1)
{
  cout<<"read image config:"<<config_file<<endl;
  FileStorage fsSettings(string(config_file), FileStorage::READ);
  if(!fsSettings.isOpened()) {
    fprintf(stderr,"打开配置文件失败:%s\n",config_file);
    return -1;
  }
  
  string distortion_model;
  Mat K1, K2, D1, D2, R, T, R1, R2, P1, P2, Q;
  fsSettings["K1"] >> K1;
  fsSettings["K2"] >> K2;
  fsSettings["D1"] >> D1;
  fsSettings["D2"] >> D2;
  fsSettings["R"] >> R;
  fsSettings["T"] >> T;
  int rows = fsSettings["height"];
  int cols = fsSettings["width"];
  fsSettings["distortion_model"] >> distortion_model;

  Size imageSize = Size(cols,rows);

  if (K1.empty() || K2.empty() || D1.empty() || D2.empty() || R.empty() || T.empty() || rows == 0 || cols == 0) {
    cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
    return -1;
  }

  cout<<"Size:\n"<< imageSize<<endl;
  cout<<"distortion_model:"<< distortion_model <<endl;
  cout<<"K1:\n"<< K1 <<endl;
  cout<<"D1:\n"<< D1 <<endl;
  cout<<"K2:\n"<< K2 <<endl;
  cout<<"D2:\n"<< D2 <<endl;
  cout<<"R:\n"<< R <<endl;
  cout<<"T:\n"<< T <<endl;

  if(distortion_model == "equidistance") {
    //鱼眼,等距畸变
    Size size;
    
    fisheye::stereoRectify(K1, D1, K2, D2, imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY,size,1.0, 0.05);
    fisheye::initUndistortRectifyMap(K1, D1, R1, P1.rowRange(0, 3).colRange(0, 3), imageSize, CV_32F, remapX0, remapY0);
    fisheye::initUndistortRectifyMap(K2, D2, R2, P2.rowRange(0, 3).colRange(0, 3), imageSize, CV_32F, remapX1,remapY1);
  } else if (distortion_model == "radtan") {
    //针孔，径切畸变
    cout<<"distortion_model:"<< distortion_model <<endl;
    stereoRectify(K1, D1, K2, D2, imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY,0);
    //stereoRectify(K1, D1, K2, D2, imgsize(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0);
    initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_32F, remapX0, remapY0);
    //initUndistortRectifyMap(K1, D1, R1, P1, imgsize(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_32F, remapX1,remapY1);
  } else {
    cout<<"distortion model not found:"<<distortion_model<<endl;
    return -1;
  }

  cout<<"R1:\n"<< R1 <<endl;
  cout<<"R2:\n"<< R2 <<endl;
  cout<<"P1:\n"<< P1 <<endl;
  cout<<"P2:\n"<< P2 <<endl;
    
  cout<<"------------"<<endl;
  return 0;
}

//计算温漂
int get_temperature_drift(float temperature, float scale)
{
  //以50°为基准进行温度补偿
  float drift = (temperature - 50.0f) * scale;
  return int(drift);
}

//温漂
Mat get_temperature_drift(float temperature, Mat scale)
{
  //以50°为基准进行温度补偿
  Mat drift = (temperature - 50.0f) * scale;
  return drift;
}

void publishOthers(imuData *imudata)
{
  ros::Time timestamp = toRosTime(imudata->imu_timestamp);
  float temperature = ((float)imudata->temperature) / 333.87f + 21.0f;

  //a = TK(a_meas - a_offset -a_tempcompesation)
  double acc_[3] = {(double)imudata->accel_x,(double)imudata->accel_y,(double)imudata->accel_z};
  double gyr_[3] = {(double)imudata->gyr_x,(double)imudata->gyr_y,(double)imudata->gyr_z};
  Mat meas_acc(3,1,CV_64FC1,acc_);
  Mat meas_gyr(3,1,CV_64FC1,gyr_);
  Mat temp_drift_acc = get_temperature_drift(temperature, tempdriftscale_acc);
  Mat temp_drift_gyr = get_temperature_drift(temperature, tempdriftscale_gyr);

  //Mat acc = alignment_acc * scale_acc * (meas_acc - bias_acc - temp_drift_acc);
  //Mat gyr = alignment_gyr * scale_gyr * (meas_gyr - bias_gyr - temp_drift_gyr);

  double sa = 0.000244141 * 9.8;
  double sg = 0.06103515625 * 3.141592653 / 180.0;

  //only bias
  sensor_msgs::Imu imu_msg;
  Mat acc = (meas_acc - bias_acc ) * sa;
  Mat gyr = (meas_gyr - bias_gyr ) * sg;
  imu_msg.header.frame_id = "imu";
  imu_msg.header.stamp = timestamp;
  imu_msg.angular_velocity.x = gyr.at<double>(0);
  imu_msg.angular_velocity.y = gyr.at<double>(1);
  imu_msg.angular_velocity.z = gyr.at<double>(2);
  imu_msg.linear_acceleration.x = acc.at<double>(0);
  imu_msg.linear_acceleration.y = acc.at<double>(1);
  imu_msg.linear_acceleration.z = acc.at<double>(2);
  imu_pub_b.publish(imu_msg);
  
  //bias+temperature_compensation
  acc = (meas_acc - bias_acc - temp_drift_acc) * sa;
  gyr = (meas_gyr - bias_gyr - temp_drift_gyr) * sg;
  imu_msg.header.frame_id = "imu";
  imu_msg.header.stamp = timestamp;
  imu_msg.angular_velocity.x = gyr.at<double>(0);
  imu_msg.angular_velocity.y = gyr.at<double>(1);
  imu_msg.angular_velocity.z = gyr.at<double>(2);
  imu_msg.linear_acceleration.x = acc.at<double>(0);
  imu_msg.linear_acceleration.y = acc.at<double>(1);
  imu_msg.linear_acceleration.z = acc.at<double>(2);
  imu_pub_tb.publish(imu_msg);

  //bias + temperature + scale
  acc = scale_acc * (meas_acc - bias_acc - temp_drift_acc);
  gyr = scale_gyr * (meas_gyr - bias_gyr - temp_drift_gyr);
  imu_msg.header.frame_id = "imu";
  imu_msg.header.stamp = timestamp;
  imu_msg.angular_velocity.x = gyr.at<double>(0);
  imu_msg.angular_velocity.y = gyr.at<double>(1);
  imu_msg.angular_velocity.z = gyr.at<double>(2);
  imu_msg.linear_acceleration.x = acc.at<double>(0);
  imu_msg.linear_acceleration.y = acc.at<double>(1);
  imu_msg.linear_acceleration.z = acc.at<double>(2);
  imu_pub_tbs.publish(imu_msg);
}

void imu_process(ros::NodeHandle *nh ,device *cam)
{
  imuData imudata;

  while(nh->ok()) {
    if(cam->readIMU(&imudata)){
      //ubuntu16下，sensor_msgs/Image的时间戳是用32位表示的，需要分别给sec和nsec赋值，否则会出错
      ros::Time timestamp = toRosTime(imudata.imu_timestamp);
      //硬件不同步时，需要手动给imu加time offset
      //timestamp = timestamp + ros::Duration(imu_timeoffset); 
      //printf("imu  :%lf\n",timestamp.toSec());

      //imu data
      float temperature = ((float)imudata.temperature) / 333.87f + 21.0f;

      //a = TK(a_meas - a_offset -a_tempcompesation)
      double acc_[3] = {(double)imudata.accel_x,(double)imudata.accel_y,(double)imudata.accel_z};
      double gyr_[3] = {(double)imudata.gyr_x,(double)imudata.gyr_y,(double)imudata.gyr_z};
      Mat meas_acc(3,1,CV_64FC1,acc_);
      Mat meas_gyr(3,1,CV_64FC1,gyr_);
      Mat temp_drift_acc = get_temperature_drift(temperature, tempdriftscale_acc);
      Mat temp_drift_gyr = get_temperature_drift(temperature, tempdriftscale_gyr);

      Mat acc = alignment_acc * scale_acc * (meas_acc - bias_acc - temp_drift_acc);
      Mat gyr = alignment_gyr * scale_gyr * (meas_gyr - bias_gyr - temp_drift_gyr);
      sensor_msgs::Imu imu_msg;
      imu_msg.header.frame_id = "imu";
      imu_msg.header.stamp = timestamp;
      imu_msg.angular_velocity.x = gyr.at<double>(0);
      imu_msg.angular_velocity.y = gyr.at<double>(1);
      imu_msg.angular_velocity.z = gyr.at<double>(2);
      imu_msg.linear_acceleration.x = acc.at<double>(0);
      imu_msg.linear_acceleration.y = acc.at<double>(1);
      imu_msg.linear_acceleration.z = acc.at<double>(2);
      imu_pub.publish(imu_msg);

      //发布用来调试对比的imu topic
      //publishOthers(&imudata);

#if 1
      Mat acc__ = alignment_acc * (meas_acc - bias_acc - temp_drift_acc);
      Mat gyr__ = alignment_gyr * (meas_gyr - bias_gyr - temp_drift_gyr);
      fprintf(stderr,"%f %4d %4d %4d %4d %4d %4d %f\n",timestamp.toSec(),
	      (int)gyr__.at<double>(0),
	      (int)gyr__.at<double>(1),
	      (int)gyr__.at<double>(2),
	      (int)acc__.at<double>(0),
	      (int)acc__.at<double>(1),
	      (int)acc__.at<double>(2),
	      temperature);
#endif
    }
  }

}

int main(int argc, char** argv)
{
  //1.ros init
  ros::init(argc, argv, "visensor7251");
  ros::NodeHandle nh;

  //调试参数，默认情况下不使用
  //interval:隔多少帧publish一次图像，用来控制相机帧率(不是sensor的真是帧率)
  //imu_timeoffset:camera相对imu的延时，imu = camera + offset
  int interval = 1;
  if(argc < 3) {
    fprintf(stderr,"参数错误,正确形式:\n    rosrun visensor7251 visensor7251 [imu_config] [image_config] [...] \n");
    return -1;
  }

  // read config files
  if(readImuConfig(argv[1])){
    fprintf(stderr,"read imu config file failed.\n");
    return -1;
  }

  Mat remapX0,remapY0,remapX1,remapY1;
  if(readImageConfig(argv[2],remapX0,remapY0,remapX1,remapY1)){
    fprintf(stderr,"read image config file failed.\n");
    return -1;
  }

  if(argc == 4)
    interval = atoi(argv[3]);

  // publisher settings
  registerPub(nh);

  
  //2.open device
  device *cam = new device();
  if (!cam->init()) {
    return 0;
  }


  //3.device settings
  //帧率、自动曝光配置
  cam->config(img_frame_rate,imu_frame_rate,max_exposure_time,min_exposure_time,cur_exposure_time);

#if 0
  //关闭自动曝光,
  cam->setExposureMode(auto_exposure);  //aoto exposure 0-off,1-on
  cam->setExposureTime(exposure_time);  //曝光时间，ms
  cam->setGain(gain);          //设置增益，3倍
#endif
  
  // imu线程
  std::thread thread_imu{imu_process, &nh, cam};

  // 图像设置
  resolution rn = cam->getImageSize();
  Mat left8u = Mat(Size(rn.width, rn.height), CV_8UC1);
  Mat right8u = Mat(Size(rn.width, rn.height), CV_8UC1);
  Mat stereo8u = Mat(Size(rn.width, rn.height*2), CV_8UC1);
  Mat stereo0 = stereo8u.rowRange(0,rn.height);
  Mat stereo1 = stereo8u.rowRange(rn.height,rn.height*2);
  uint64 imgTimeStamp = 0;  
  imageData imagedata;

  // start capture image
  cam->start();

  //main loop
  int aa = 0;
  while (nh.ok()) {
    if (cam->QueryFrame(left8u, right8u, imgTimeStamp)) {
      //隔多少帧发送一次，测试时使用
      if (aa++ % interval != 0)
	continue;

      ros::Time timestamp = toRosTime(imgTimeStamp);
      //printf("image  :%lf,   %lf\n",timestamp.toSec(),(double)imgTimeStamp/100000000);

      //remap image
      cv::remap(left8u, stereo0, remapX0, remapY0, cv::INTER_LINEAR);
      cv::remap(right8u, stereo1, remapX1, remapY1, cv::INTER_LINEAR);
      
      cv_bridge::CvImage cvi0, cvi1, cvi2;
      cvi0.header.stamp =  timestamp;
      cvi0.header.frame_id = "image0";
      cvi0.encoding = "mono8";
      cvi0.image = left8u;
      //cvi0.image = stereo0;

      cvi1.header.stamp =  cvi0.header.stamp;
      cvi1.header.frame_id = "image1";
      cvi1.encoding = "mono8";
      cvi1.image = right8u;
      //cvi1.image = stereo1;
      
      cvi2.header.stamp =  cvi0.header.stamp;
      cvi2.header.frame_id = "stereo_remap";
      cvi2.encoding = "mono8";
      cvi2.image = stereo8u;

      sensor_msgs::Image msg0, msg1, msg2;
      cvi0.toImageMsg(msg0);
      cvi1.toImageMsg(msg1);
      cvi2.toImageMsg(msg2);

      //publish
      img_pub0.publish(msg0);
      img_pub1.publish(msg1);
      img_pub2.publish(msg2);
    }
  }

  ros::spin();
  
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
