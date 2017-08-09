#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

/**************预积分***********************/
class IntegrationBase
{
public:
  Eigen::Vector3d acc_0,acc_1;
  Eigen::Vector3d gyr_0,gyr_1;
  Eigen::Vector3d linearized_acc;
  Eigen::Vector3d linearized_gyr;
  Eigen::Vector3d linearized_ba;
  Eigen::Vector3d linearized_bg;
  double dt;
  double sum_dt;
  Eigen::Vector3d delta_p;
  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_v;

public:
  IntegrationBase() = delete;
  IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
		  const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
    linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
    sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

  {

  }


  void midPointIntegration(double _dt, 
			   const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
			   const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
			   const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
			   const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
			   Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
			   Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
  {
    //ROS_INFO("midpoint integration");
    Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;         
  }

  void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
  {
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Vector3d result_delta_p;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;

    //
    midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
			linearized_ba, linearized_bg,
			result_delta_p, result_delta_q, result_delta_v,
			result_linearized_ba, result_linearized_bg, 1);

    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;  
  }
      
};
/*********************************/

ros::Publisher pub_path;
ros::Publisher pub_pose;

//
IntegrationBase *pPreItegration = NULL;
Vector3d acc_0, gyr_0;
Vector3d bias_acc,bias_gyr;
double current_time = 0;
bool first_imu = false;
int frame_count;
int COUNT =500;

//结果
Vector3d Ps;
Vector3d Vs;
Matrix3d Rs;
Vector3d Bas;
Vector3d Bgs;

void processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
  if (!first_imu) {
    first_imu = true;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
  }

  if (!pPreItegration) {
    pPreItegration = new IntegrationBase(acc_0, gyr_0, bias_acc, bias_gyr);
  }
  if (frame_count > COUNT) {
    first_imu = false;
    frame_count = 0;
    delete pPreItegration;
    pPreItegration = NULL;
    return;
  }

  pPreItegration->propagate(dt, linear_acceleration, angular_velocity);
  frame_count++;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
  double t = imu_msg->header.stamp.toSec();
  if (current_time < 0)
    current_time = t;
  double dt = t - current_time;
  current_time = t;

  double ba[]{0.0, 0.0, 0.0};
  double bg[]{0.0, 0.0, 0.0};

  double dx = imu_msg->linear_acceleration.x - ba[0];
  double dy = imu_msg->linear_acceleration.y - ba[1];
  double dz = imu_msg->linear_acceleration.z - ba[2];

  double rx = imu_msg->angular_velocity.x - bg[0];
  double ry = imu_msg->angular_velocity.y - bg[1];
  double rz = imu_msg->angular_velocity.z - bg[2];
  //ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);
  
  processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz)); 

  //publish result
  if(pPreItegration) {
    //cout<<"p:"<<pPreItegration->delta_p.transpose()<<endl;
    std_msgs::Header header = imu_msg->header;
    header.frame_id = "world";

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = pPreItegration->delta_p.x();
    odometry.pose.pose.position.y = pPreItegration->delta_p.y();
    odometry.pose.pose.position.z = pPreItegration->delta_p.z();
    odometry.pose.pose.orientation.x = Quaterniond(pPreItegration->delta_q).x();
    odometry.pose.pose.orientation.y = Quaterniond(pPreItegration->delta_q).y();
    odometry.pose.pose.orientation.z = Quaterniond(pPreItegration->delta_q).z();
    odometry.pose.pose.orientation.w = Quaterniond(pPreItegration->delta_q).w();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;

    //path
    nav_msgs::Path path;
    path.header = header;
    path.header.frame_id = "world";
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    //pose
    pose_stamped.pose.position.x = 0;
    pose_stamped.pose.position.y = 0;
    pose_stamped.pose.position.z = 0;
    pub_pose.publish(pose_stamped);

    //cout<<pPreItegration->sum_dt<<" "<<pPreItegration->delta_p.norm()<<endl;
  }
}

//测试imu预积分随时间的变化，这里不考虑噪声
//拿了10秒的数据，matlab做数据拟合，position=4.785*t^2
int main(int argc, char **argv)
{
  ros::init(argc, argv, "preItegration");
  ros::NodeHandle n;

  Vector3d acc_0, gyr_0, bias_acc, bias_gyr;

  if(argc >1){
    COUNT = atoi(argv[1]);
  }

  ros::Subscriber sub = n.subscribe("/imu0", 1000, imuCallback);
  pub_path = n.advertise<nav_msgs::Path>("/imu_preIntegration_path", 1000);
  pub_pose = n.advertise<geometry_msgs::PoseStamped>("/imu_preIntegration_pose", 1000);

  ros::spin();

  return 0;
}
