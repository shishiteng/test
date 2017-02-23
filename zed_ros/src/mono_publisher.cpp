#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

using namespace cv;

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  if(argv[1] == NULL) {
    ROS_INFO("argv[1]=NULL\n");
    return 1;
  }

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

  // Convert the passed as command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;
  // Check if it is indeed a number
  if(!(video_sourceCmd >> video_source)) {
    ROS_INFO("video_sourceCmd is %d\n",video_source);
    return 1;
  }

  cv::VideoCapture cap(video_source);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) {
    ROS_INFO("can not opencv video device\n");
    return 1;
  }

  int width = 640;
  int height = 480;
  //cap.set(CV_CAP_PROP_FRAME_WIDTH, width * 2);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);

  cv::Mat frame,frame0;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(20);
  while (nh.ok()) {
    cap >> frame;
    if(!frame.empty()) {
      Rect roi_rect0 = cv::Rect(0,0,width,height);
      frame0 = frame(roi_rect0);
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame0).toImageMsg();
      pub.publish(msg);
      //cv::Wait(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
