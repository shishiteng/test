#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

#define WIDTH 672
#define HEIGHT 376

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
  image_transport::Publisher pub0 = it.advertise("cam0/image_raw", 1);
  image_transport::Publisher pub1 = it.advertise("cam1/image_raw", 1);

  int dev = atoi(argv[1]);
  VideoCapture cap(dev);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,WIDTH*2);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
  if(!cap.isOpened()) {
    ROS_INFO("can not opencv video device\n");
    return 1;
  }

  cv::Mat frame,frame0,frame1;
  sensor_msgs::ImagePtr msg;

  //ros::Rate loop_rate(25);
  while (nh.ok()) {
    cap >> frame;
    if(!frame.empty()) {
      cvtColor(frame,frame,CV_BGR2GRAY);
      Rect roi_rect0 = cv::Rect(0,0,WIDTH,HEIGHT);
      Rect roi_rect1 = cv::Rect(WIDTH,0,WIDTH,HEIGHT);
      frame0 = frame(roi_rect0);
      frame1 = frame(roi_rect1);

      ros::Time timestamp = ros::Time::now();// - ros::Duration(0,150000000);

      cv_bridge::CvImage cvi0;
      cvi0.header.stamp = timestamp;
      cvi0.header.frame_id = "image";
      cvi0.encoding = "mono8";
      cvi0.image = frame0;

      cv_bridge::CvImage cvi1;
      cvi1.header.stamp = timestamp;
      cvi1.header.frame_id = "image";
      cvi1.encoding = "mono8";
      cvi1.image = frame1;

      sensor_msgs::Image msg0, msg1;
      cvi0.toImageMsg(msg0);
      cvi1.toImageMsg(msg1);

      pub0.publish(msg0);
      pub1.publish(msg1);
    }

    ros::spinOnce();
    //loop_rate.sleep();
  }

  //////////
  //save();
}
