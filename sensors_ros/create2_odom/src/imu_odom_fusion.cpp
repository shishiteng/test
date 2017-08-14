#include <stdio.h>
#include <math.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "Eigen/Eigen"

#include "imu_odom_fusion.h"

using namespace std;
using namespace Eigen;

/************全局变量*********/
std::mutex m_buf;
std::condition_variable con;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<nav_msgs::Odometry::ConstPtr> odom_buf;

IntegrationOdometry *pOdomIntegration = NULL;
Vector3d vel_0, gyr_0;
double current_time = 0;
bool first_odom = false;

ros::Publisher pub_odom;
ros::Publisher pub_imu_pose;
ros::Publisher pub_odom_pose;
/************全局变量 end*********/

/**************imu预积分 start*****************/
/**************imu预积分 end*****************/


/********************执行函数*****************/

std::vector< std::pair<std::vector<sensor_msgs::ImuConstPtr>, nav_msgs::Odometry::ConstPtr> >
getMeasurements()
{
  std::vector< std::pair<std::vector<sensor_msgs::ImuConstPtr>, nav_msgs::Odometry::ConstPtr> >measurements;
  while (true) {
    if (imu_buf.empty() || odom_buf.empty())
      return measurements;

    if (!(imu_buf.back()->header.stamp > odom_buf.front()->header.stamp)) {
      ROS_WARN("wait for imu, only should happen at the beginning");
      return measurements;
    }

    if (!(imu_buf.front()->header.stamp < odom_buf.front()->header.stamp)) {
      ROS_WARN("throw odom, only should happen at the beginning");
      odom_buf.pop();
      continue;
    }
    nav_msgs::Odometry::ConstPtr odom_msg = odom_buf.front();
    odom_buf.pop();

    std::vector<sensor_msgs::ImuConstPtr> IMUs;
    while (imu_buf.front()->header.stamp <= odom_msg->header.stamp) {
      IMUs.emplace_back(imu_buf.front());
      imu_buf.pop();
    }

    measurements.emplace_back(IMUs, odom_msg);
  }
  return measurements;
}

void publishResult(const nav_msgs::Odometry::ConstPtr &odom_msg,Vector3d p, Quaterniond qIMU,Quaterniond qOdom)
{
  if (pOdomIntegration) {
    std_msgs::Header header = odom_msg->header;
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.pose.pose.position.x = pOdomIntegration->delta_p.x();
    odometry.pose.pose.position.y = pOdomIntegration->delta_p.y();
    odometry.pose.pose.position.z = pOdomIntegration->delta_p.z();
    odometry.pose.pose.orientation.x = Quaterniond(pOdomIntegration->delta_q).x();
    odometry.pose.pose.orientation.y = Quaterniond(pOdomIntegration->delta_q).y();
    odometry.pose.pose.orientation.z = Quaterniond(pOdomIntegration->delta_q).z();
    odometry.pose.pose.orientation.w = Quaterniond(pOdomIntegration->delta_q).w();
    pub_odom.publish(odometry);

    //odom帧间的角度变化
    geometry_msgs::PoseStamped pose_odom;
    pose_odom.header = header;
    pose_odom.pose.orientation.x = qOdom.x();
    pose_odom.pose.orientation.y = qOdom.y();
    pose_odom.pose.orientation.z = qOdom.z();
    pose_odom.pose.orientation.w = qOdom.w();
    pose_odom.pose.position.x = 0;
    pose_odom.pose.position.y = 1;
    pose_odom.pose.position.z = 0;
    pub_odom_pose.publish(pose_odom);

    //imu预积分的角度变化量
    geometry_msgs::PoseStamped pose_imu;
    pose_imu.header = header;
    pose_imu.pose.orientation.x = qIMU.x();
    pose_imu.pose.orientation.y = qIMU.y();
    pose_imu.pose.orientation.z = qIMU.z();
    pose_imu.pose.orientation.w = qIMU.w();
    pose_imu.pose.position.x = 0;
    pose_imu.pose.position.y = 0;
    pose_imu.pose.position.z = 0;
    pub_imu_pose.publish(pose_imu);
  }
}

void processOdom(double dt, const Vector3d &linear_velocity, const Vector3d &angular_velocity)
{
  if (!first_odom) {
    first_odom = true;
    vel_0 = linear_velocity;
    gyr_0 = angular_velocity;
  }

  if (!pOdomIntegration) {
    pOdomIntegration = new IntegrationOdometry(vel_0, gyr_0);
  }

  pOdomIntegration->propagate(dt, linear_velocity, angular_velocity);
}

void process()
{
  double last_angular = 98765.4321;
  double last_time = 0;

  while (true) {
    std::vector< std::pair<std::vector<sensor_msgs::ImuConstPtr>, nav_msgs::Odometry::ConstPtr> >measurements;
    std::unique_lock<std::mutex> lk(m_buf);
    con.wait(lk, [&]
	     {
	       return (measurements = getMeasurements()).size() != 0;
	     });
    lk.unlock();

    for (auto &measurement : measurements) {
      //IMU process
      IntegrationIMU *pIMU = NULL;
      double current_time = 0;

      for (auto &imu_msg : measurement.first) {
	double t = imu_msg->header.stamp.toSec();
	//cout<<"imu  time:"<<imu_msg->header.stamp<<endl;

	double ba[]{0.000, 0.000, 0.000};
	double bg[]{0.000, 0.0000, 0.0000};
	double dx = imu_msg->linear_acceleration.x - ba[0];
	double dy = imu_msg->linear_acceleration.y - ba[1];
	double dz = imu_msg->linear_acceleration.z - ba[2];
	double rx = imu_msg->angular_velocity.x - bg[0];
	double ry = imu_msg->angular_velocity.y - bg[1];
	double rz = imu_msg->angular_velocity.z - bg[2];
	Vector3d acc(dx, dy, dz);
	Vector3d gyr(rx, ry, rz);
	Vector3d acc_bias(0, 0, 0);
	Vector3d gyr_bias(0, 0, 0);

	if(!pIMU) {
	  current_time = t;
	  pIMU = new IntegrationIMU(acc, gyr, acc_bias, gyr_bias);
	}

	double dt = t - current_time;
	current_time = t;
	pIMU->propagate(dt, acc, gyr); 
      }
      Vector3d imu_delta_p = pIMU->delta_p;
      Quaterniond imu_delta_q = pIMU->delta_q;
      Vector3d imu_linear_velocity = pIMU->delta_p/pIMU->sum_dt;
      double omega_imu = acos(pIMU->delta_q.w())*2; //imu计算出的角度,单位弧度,这是个正数
      double ss = imu_delta_q.z()==0 ? 1 : imu_delta_q.z()/fabs(imu_delta_q.z());//我的imu是绕着z轴转的，角轴表示法：omega是转动角度，ss的正负代表朝左或者朝右转
      omega_imu *= ss;
      delete pIMU;
      pIMU = NULL;

      //odometry process
      auto odom_msg = measurement.second;
      double dx = odom_msg->twist.twist.linear.x;
      double dy = odom_msg->twist.twist.linear.y;
      double dz = odom_msg->twist.twist.linear.z;
      double rx = odom_msg->twist.twist.angular.x;
      double ry = odom_msg->twist.twist.angular.y;
      double rz = odom_msg->twist.twist.angular.z;

      //记录上一帧odom的时间和角速度,用了一个魔鬼数字
      if ( 98765.4321 == last_angular) {
	last_angular = odom_msg->twist.twist.angular.z;
	last_time = odom_msg->header.stamp.toSec();
      }

      //cout<<"odom time:"<<odom_msg->header.stamp<<endl;
      double dt = odom_msg->header.stamp.toSec() - last_time;
      double omega_odom = 0;
      if(dt != 0 )
	//omega_odom = (odom_msg->twist.twist.angular.z + last_angular)/2*dt; //odom 计算出的角度，单位弧度
	omega_odom = odom_msg->twist.twist.angular.z*dt; //odom 角度变化量，单位弧度

      //轮子打滑判断
      //odometry得到的旋转量与imu得到的旋转量有偏差，如果偏差超过1度，相信imu，位移也要用imu的
      double offset = (omega_odom - omega_imu)*57.3;
      if(fabs(offset) > 1) {
	rz = 2*omega_imu/dt - last_angular;
	//速度可以用imu表示，但是需要知道imu和odom的RT，此处设置为0
	dx = 0;
      }
      cout<<"  offset: "<<offset<< "     rz: "<<rz<<"   imu:  "<<omega_imu*57.3f<<"  odom:"<<omega_odom*57.3f <<endl;
      cout<<"imu deltap :"<<imu_delta_p.transpose()<<endl;
      cout<<"imu vel    :"<<imu_linear_velocity.transpose()<<endl;
      processOdom(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz)); 

      //publish result
      Quaterniond odom_delta_q;
      odom_delta_q.w() = cos(omega_odom/2);
      odom_delta_q.x() = 0;
      odom_delta_q.y() = 0;
      odom_delta_q.z() = sin(omega_odom/2);
      publishResult(odom_msg, imu_delta_p, imu_delta_q , odom_delta_q);

      last_angular = odom_msg->twist.twist.angular.z;
      last_time = odom_msg->header.stamp.toSec();
    }
  }
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
  m_buf.lock();
  imu_buf.push(imu_msg);
  m_buf.unlock();
  con.notify_one();
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  m_buf.lock();
  odom_buf.push(odom_msg);
  m_buf.unlock();
  con.notify_one();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_imu_fusion");
  ros::NodeHandle n("~");

  ros::Subscriber sub_imu = n.subscribe("/imu0", 2000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_odom = n.subscribe("/odom", 2000, odom_callback);

  pub_odom = n.advertise<nav_msgs::Odometry>("/odom_imu_combined", 1000);
  pub_imu_pose = n.advertise<geometry_msgs::PoseStamped>("/imu_preIntergration_pose", 1000);//
  pub_odom_pose = n.advertise<geometry_msgs::PoseStamped>("/odom_diff_pose", 1000);//odom两帧之间的pose

  std::thread measurement_process{process};

  ros::spin();

  return 0;
}
