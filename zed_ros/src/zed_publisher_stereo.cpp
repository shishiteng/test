#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <iostream>
#include <string>

#define WIDTH 672
#define HEIGHT 376

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  if(argv[1] == NULL) {
    ROS_INFO("argv[1]=NULL\n");
    return -1;
  }

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub0 = it.advertise("stereo/image_raw", 1);
  image_transport::Publisher pub1 = it.advertise("stereo_remap/image_raw", 1);

  int dev = atoi(argv[1]);
  VideoCapture cap(dev);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,WIDTH*2);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
  if(!cap.isOpened()) {
    ROS_INFO("can not opencv video device\n");
    return 1;
  }

  Mat frame,frame0,frame1;
  Mat stereo(Size(WIDTH, HEIGHT*2), CV_8UC1);
  Mat stereo0 = stereo.rowRange(0,HEIGHT);
  Mat stereo1 = stereo.rowRange(HEIGHT,HEIGHT*2);

  Mat stereo_remap(Size(WIDTH, HEIGHT*2), CV_8UC1);
  Mat stereo0_remap = stereo_remap.rowRange(0,HEIGHT);
  Mat stereo1_remap = stereo_remap.rowRange(HEIGHT,HEIGHT*2);

  string configFile("/home/sst/catkin_ws/src/zed/config/zed_stereo.yaml");
  cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);
  if (fsSettings.isOpened() == false) {
    cerr << "Cannot load the config file from " << configFile << endl;
  }

  // rectification parameters
  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
  fsSettings["LEFT.K"] >> K_l;
  fsSettings["RIGHT.K"] >> K_r;

  fsSettings["LEFT.P"] >> P_l;
  fsSettings["RIGHT.P"] >> P_r;

  fsSettings["LEFT.R"] >> R_l;
  fsSettings["RIGHT.R"] >> R_r;

  fsSettings["LEFT.D"] >> D_l;
  fsSettings["RIGHT.D"] >> D_r;

  int rows_l = fsSettings["LEFT.height"];
  int cols_l = fsSettings["LEFT.width"];
  int rows_r = fsSettings["RIGHT.height"];
  int cols_r = fsSettings["RIGHT.width"];

  if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
      D_r.empty() ||
      rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
    cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
    return 1;
  }

  cv::Mat M1l, M2l, M1r, M2r;
  cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
  cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
    

  sensor_msgs::ImagePtr msg;

  //ros::Rate loop_rate(25);
  while (nh.ok()) {
    cap >> frame;
    if(!frame.empty()) {
      ros::Time timestamp = ros::Time::now();// - ros::Duration(0,150000000);

      cvtColor(frame,frame,CV_BGR2GRAY);
      Rect roi_rect0 = cv::Rect(0,0,WIDTH,HEIGHT);
      Rect roi_rect1 = cv::Rect(WIDTH,0,WIDTH,HEIGHT);

      //publish origin image
      frame0 = frame(roi_rect0);
      frame1 = frame(roi_rect1);

      frame0.copyTo(stereo0);
      frame1.copyTo(stereo1);

      cv_bridge::CvImage cvi0;
      cvi0.header.stamp = timestamp;
      cvi0.header.frame_id = "stereo";
      cvi0.encoding = "mono8";
      cvi0.image = stereo;
      sensor_msgs::Image msg0;
      cvi0.toImageMsg(msg0);
      pub0.publish(msg0);

      //publish remap image
      cv::remap(stereo0, frame0, M1l, M2l, cv::INTER_LINEAR);
      cv::remap(stereo1, frame1, M1r, M2r, cv::INTER_LINEAR);

      frame0.copyTo(stereo0_remap);
      frame1.copyTo(stereo1_remap);
	
      cv_bridge::CvImage cvi1;
      cvi1.header.stamp = timestamp;
      cvi1.header.frame_id = "stereo_remap";
      cvi1.encoding = "mono8";
      cvi1.image = stereo_remap;
      sensor_msgs::Image msg1;
      cvi1.toImageMsg(msg1);
      pub1.publish(msg1);

      imshow("stereo",stereo);
      imshow("stereo_remap",stereo_remap);
      waitKey(5);
    }

    ros::spinOnce();
    //loop_rate.sleep();
  }

  //////////
  //save();
}
