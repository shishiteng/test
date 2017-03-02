#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
class Att_ekf
{
 public:
  Att_ekf();
  ~Att_ekf();

  void predict(double t);
  void update(Vector3d &acc, Vector3d & gyro, Vector3d& mag, double t);
  void update_magnetic(Vector3d& mag, double t);
  void update_imu(Vector3d &acc, Vector3d & gyro, double t);
  Matrix3d get_rotation_matrix();
  double get_time() {return curr_t;}

 private:
  const double acc_cov = 0.015;
  const double gyro_cov = 0.0001;
  const double mag_cov = 0.00001;

  double curr_t;
  Matrix<double, 12, 1> x;//anguler velocity, angular acceleration velocity, gravity field, magnetic field 
  Matrix<double, 12, 12> P;//covariance
  const Matrix<double, 12, 12> Q = MatrixXd::Identity(12, 12)*0.01;//predict noise
  Matrix<double, 9, 9> R;//measurement noise

  const Matrix<double, 3, 3> R_acc = MatrixXd::Identity(3, 3)*acc_cov;
  const Matrix<double, 3, 3> R_gyro = MatrixXd::Identity(3, 3)*gyro_cov;
  const Matrix<double, 3, 3> R_mag = MatrixXd::Identity(3, 3)*mag_cov;

  bool initialized;
};
