#include <time.h>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <queue>

using namespace std;
using namespace Eigen;


nav_msgs::Path path;
ros::Publisher pose_pub;
ros::Publisher path_pub;


int init = 0;
Quaterniond baseR;
Vector3d baseT;

queue <geometry_msgs::TransformStamped::ConstPtr> tango_buf;


void printQuaternion(char *str, Quaterniond q)
{
  //ROS_INFO("%s %.3f %.3f %.3f %.3f",str, q.w(),q.x(),q.y(),q.z());
  fprintf(stderr,"%s %.3f %.3f %.3f %.3f\n",str, q.w(),q.x(),q.y(),q.z());
}


void tangoPoseCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
  tango_buf.push(msg);
  //ROS_INFO("received  tango pose:%d",tango_buf.size());
}

void vinsOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  //ROS_INFO("received  vins odom:%lf",msg->header.stamp.toSec());
  geometry_msgs::TransformStamped::ConstPtr msg2;

  //get message pairs
  int n = 0;
  bool match_flag = false;
  while(!match_flag) {
    if(tango_buf.empty()) {
      ROS_INFO("no tango message received");
      return;
    }

    if(n++ > 1000) {
      ROS_INFO("can not find matching messages");
      return;
    }
    
    //if((tango_buf.front()->header.stamp + ros::Duration(0.010)) < msg->header.stamp ) {
    if(tango_buf.front()->header.stamp < msg->header.stamp ) {
      tango_buf.pop();
      //}else if(!match_flag && (tango_buf.front()->header.stamp + ros::Duration(0.010)) >= msg->header.stamp){
      }else if(!match_flag && tango_buf.front()->header.stamp >= msg->header.stamp){
      msg2 = tango_buf.front();
      tango_buf.pop();
      ROS_INFO("matches:  %lf  %lf",msg->header.stamp.toSec(), msg2->header.stamp.toSec());
      match_flag = true;
    }
  }


  if(!match_flag)
    return;
  
  //compute base RT
  if(init++ < 20) {
    //R = Rvins * Rtango.inv
    //T = Tvins - R*Ttango
    Quaterniond vins_startR = Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    Vector3d vins_startT = Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);

    Quaterniond tango_startR = Quaterniond(msg2->transform.rotation.w,msg2->transform.rotation.x,msg2->transform.rotation.y,msg2->transform.rotation.z);
    Vector3d tango_startT = Vector3d(msg2->transform.translation.x,msg2->transform.translation.y,msg2->transform.translation.z);

    //rt from tango to vins
    baseR = vins_startR * tango_startR.inverse();
    baseT = vins_startT - baseR * tango_startT;

    //printQuaternion("vins : ",vins_startR);
    //printQuaternion("tango: ",tango_startR);
    //printQuaternion("baseR: ",baseR);
    //printQuaternion("vins2: ", baseR * tango_startR);
  } else {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.header.frame_id = "world";
    Quaterniond tmp_R = baseR *  Quaterniond(msg2->transform.rotation.w,msg2->transform.rotation.x,msg2->transform.rotation.y,msg2->transform.rotation.z);
    //Quaterniond tmp_R = Quaterniond(msg2->transform.rotation.w,msg2->transform.rotation.x,msg2->transform.rotation.y,msg2->transform.rotation.z);
    Vector3d tmp_T = baseT + baseR * Vector3d(msg2->transform.translation.x,msg2->transform.translation.y, msg2->transform.translation.z);
    
    pose_stamped.pose.orientation.w = tmp_R.w();
    pose_stamped.pose.orientation.x = tmp_R.x();
    pose_stamped.pose.orientation.y = tmp_R.y();
    pose_stamped.pose.orientation.z = tmp_R.z();
    pose_stamped.pose.position.x = tmp_T.x();
    pose_stamped.pose.position.y = tmp_T.y();
    pose_stamped.pose.position.z = tmp_T.z();

    pose_pub.publish(pose_stamped);

    //path
    path.header = msg->header;
    path.header.frame_id = "world";
    path.poses.push_back(pose_stamped);
    path_pub.publish(path);
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "phab2_pose");
  ros::NodeHandle nh;

  //ros::Subscriber sub_pose_vins = nh.subscribe("/vins_estimator/camera_pose", 10, vinsPoseCallback);
  ros::Subscriber sub_odom_vins = nh.subscribe("/vins_estimator/odometry", 10, vinsOdomCallback);
  ros::Subscriber sub_pose_tango = nh.subscribe("/tango/transform/start_of_service_T_device", 10000, tangoPoseCallback); 

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("tango_pose",1000);
  path_pub = nh.advertise<nav_msgs::Path>("tango_path",10);
   
  ros::spin();
}
