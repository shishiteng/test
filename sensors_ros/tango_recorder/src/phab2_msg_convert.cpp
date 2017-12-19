#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

nav_msgs::Path path;

ros::Publisher imu_pub;
ros::Publisher path_pub;
image_transport::Publisher img_pub;


int frame_count = 0;
int a = 1;


void compressedImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    if(0 == (frame_count++)%a) {
      img_pub.publish(*msg);
      cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);
      cv::waitKey(5);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = msg->header.stamp;
  imu_msg.angular_velocity.x = msg->angular_velocity.x;
  imu_msg.angular_velocity.y = msg->angular_velocity.y;
  imu_msg.angular_velocity.z = msg->angular_velocity.z;
  imu_msg.linear_acceleration.x = msg->linear_acceleration.x;
  imu_msg.linear_acceleration.y = msg->linear_acceleration.y;
  imu_msg.linear_acceleration.z = msg->linear_acceleration.z;

  imu_pub.publish(imu_msg);
}

void pathCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.orientation = msg->transform.rotation;
  pose_stamped.pose.position.x = msg->transform.translation.x;
  pose_stamped.pose.position.y = msg->transform.translation.y;
  pose_stamped.pose.position.z = msg->transform.translation.z;

  path.poses.push_back(pose_stamped);
  
  path_pub.publish(path);
  //pose_pub.publish(pose_stamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "phab2");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  if(2 == argc)
    a = atoi(argv[1]);
  ROS_INFO("publish image interval:%d",a);

  ros::Subscriber sub_imu = nh.subscribe("android/imu", 100, imuCallback);
  ros::Subscriber sub_pose = nh.subscribe("/tango/transform/start_of_service_T_device", 100, pathCallback); 
  image_transport::Subscriber sub_compressed_img = it.subscribe("tango/camera/fisheye_1/image_raw", 10, compressedImageCallback,ros::VoidPtr(),image_transport::TransportHints("compressed"));

  imu_pub = nh.advertise<sensor_msgs::Imu>("imu0", 10);
  img_pub = it.advertise("cam0/image_raw", 1);
  path_pub = nh.advertise<nav_msgs::Path>("path",10);
   
  ros::spin();
  cv::destroyWindow("view");
}
