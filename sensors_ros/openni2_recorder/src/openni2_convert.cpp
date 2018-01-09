#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


ros::Publisher odom_pub;
image_transport::Publisher rgb_pub;
image_transport::Publisher depth_pub;


int count_rgb = 0;
int count_depth = 0;

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(0 == count_rgb++ % 3) {
    sensor_msgs::Image img_msg;
    img_msg = *msg;
    img_msg.header.stamp = ros::Time::now();
    rgb_pub.publish(img_msg);
  }
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    if(0 == count_depth++ % 3) {
      sensor_msgs::Image img_msg;
      img_msg = *msg;
      img_msg.header.stamp = ros::Time::now();
      depth_pub.publish(img_msg);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("depth image catch exception");
  }
}

 void vinsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  nav_msgs::Odometry odom_msg;
  odom_msg = *msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_pub.publish(odom_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "openni2_recorder");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  ros::Subscriber sub_pose = nh.subscribe("/vins_estimator/odometry", 10, vinsOdomCallback); 
  image_transport::Subscriber sub_rgb_img = it.subscribe("/camera/rgb/image_raw", 10, rgbImageCallback);
  image_transport::Subscriber sub_depth_img = it.subscribe("/camera/depth/image", 10, depthImageCallback);

  rgb_pub = it.advertise("/asus/rgb/image_raw", 1);
  depth_pub = it.advertise("/asus/depth/image", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/asus/odometry",1);
   
  ros::spin();
}
