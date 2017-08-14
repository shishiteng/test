#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

/**************imu预积分***********************/
class IntegrationIMU
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
  IntegrationIMU() = delete;
  IntegrationIMU(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
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



/************************odom积分**************************************/
class IntegrationOdometry
{
public:
  Eigen::Vector3d vel_0,vel_1;
  Eigen::Vector3d gyr_0,gyr_1;
  double dt;
  double sum_dt;
  Eigen::Vector3d delta_p;
  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_v;

public:
  IntegrationOdometry() = delete;
  IntegrationOdometry(const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_gyr_0)
    : vel_0{_vel_0}, gyr_0{_gyr_0},sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, 
    delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

  {

  }


  void midPointIntegration(double _dt, 
			   const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_gyr_0,
			   const Eigen::Vector3d &_vel_1, const Eigen::Vector3d &_gyr_1,
			   const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
			   Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v)
  {
    //ROS_INFO("midpoint integration");
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1);
    result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    Vector3d un_vel_0 = delta_q * (_vel_0);
    Vector3d un_vel_1 = result_delta_q * (_vel_1);
    Vector3d un_vel = 0.5 * (un_vel_0 + un_vel_1);
    result_delta_p = delta_p + delta_v * _dt;
    //result_delta_v = delta_v;
    result_delta_v = un_vel;
  }

  void propagate(double _dt, const Eigen::Vector3d &_vel_1, const Eigen::Vector3d &_gyr_1)
  {
    dt = _dt;
    vel_1 = _vel_1;
    gyr_1 = _gyr_1;
    Vector3d result_delta_p;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;

    //
    midPointIntegration(_dt, vel_0, gyr_0, _vel_1, _gyr_1, delta_p, delta_q, delta_v,
			result_delta_p, result_delta_q, result_delta_v);

    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    delta_q.normalize();
    sum_dt += dt;
    vel_0 = vel_1;
    gyr_0 = gyr_1;  
  }
      
};

