#include "att_ekf.h"
#include <iostream>
#include <Eigen/Eigen>
#include "conversion.h"

using namespace std;
using namespace Eigen;

Matrix3d skew_symmetric(Vector3d& v)
{
  Matrix3d m;
  m << 0, -v(2), v(1),
    v(2), 0, -v(0),
    -v(1), v(0), 0;
  return m;
}

Att_ekf::Att_ekf()
{
  x.setZero();
  initialized = false;

  R.setZero();
  R.block<3, 3>(0, 0) = R_gyro;
  R.block<3, 3>(3, 3) = R_acc;
  R.block<3, 3>(3, 3) = R_mag;

  P.setZero();
  P.block<3, 3>(0, 0) = R_gyro;
  P.block<3, 3>(3, 3) = R_gyro;
  P.block<3, 3>(6, 6) = R_acc;
  P.block<3, 3>(9, 9) = R_mag;
}

Att_ekf::~Att_ekf()
{

}

//
void Att_ekf::predict(double t)
{
  double dt = t - curr_t;
  //cout << "dt :" << dt << endl;
  Vector3d w = x.head(3);
  Vector3d wa  = x.segment<3>(3);
  Vector3d ra = x.segment<3>(6); //取反
  Vector3d rm = x.tail(3);

  //state:[w,w',a,m]
  x.segment<3>(0) += wa*dt;
  x.segment<3>(6) += skew_symmetric(w)*ra*dt;
  x.segment<3>(9) += skew_symmetric(w)*rm*dt;

  //Jacbian of f
  Matrix<double, 12, 12> A = MatrixXd::Identity(12, 12);
  A.block<3, 3>(0, 3) = MatrixXd::Identity(3, 3)*dt;
  A.block<3, 3>(6, 0) = -skew_symmetric(ra)*dt;
  A.block<3, 3>(6, 6) += skew_symmetric(w)*dt;
  A.block<3, 3>(9, 0) = -skew_symmetric(rm)*dt;
  A.block<3, 3>(9, 9) += skew_symmetric(w)*dt;

  //P
  P = A*P*A.transpose() + Q;
  curr_t = t;
}

void Att_ekf::update(Vector3d &acc, Vector3d & gyro, Vector3d& mag, double t)
{
  if(!initialized) {
    x.head(3) = gyro;
    x.segment<3>(3) = Vector3d(0,0,0);
    x.segment<3>(6) = acc;
    x.tail(3) = mag;
    if(mag == Vector3d(0,0,0))
      return;
    initialized = true;
    curr_t = t;
    return;
  }
  if(t < curr_t) {
    cout << "t is smaller than curr_t" << endl;
    return;
  }

  predict(t);

  //z
  Matrix<double, 9, 1> z;
  z.head(3) = gyro;
  z.segment<3>(3) = acc;
  z.tail(3) = mag;

  //H
  MatrixXd H = MatrixXd::Zero(9, 12);
  H.block<3, 3>(0, 0) = Matrix3d::Identity();
  H.block<3, 3>(3, 6) = Matrix3d::Identity();
  H.block<3, 3>(6, 9) = Matrix3d::Identity();

  //Kalman Gain
  MatrixXd K = P.inverse()*H.transpose()*(H*P.inverse()*H.transpose() + R).inverse();
  MatrixXd I = MatrixXd::Identity(12, 12);

  //update state and P
  x = x + K*(z - H*x);
  P = (I- K*H)*P.inverse();
  cout << "update: " << x.transpose() <<"    " <<mat2euler(get_rotation_matrix()).transpose()*57.3 << endl;
  Quaterniond q = mat2quaternion(get_rotation_matrix());
  //cout << "q:" <<q.w() <<" "<<q.x() <<" "<<q.y() <<" "<<q.z() <<" "<< endl;
  //cout<< " matrix:\n"<<get_rotation_matrix()<<endl;
}


Matrix3d Att_ekf::get_rotation_matrix()
{
  if(!initialized) 
    return Matrix3d::Identity();

  Matrix3d Rbn;
  Vector3d ra = x.segment<3>(6);
  Vector3d rm = x.segment<3>(9);

  Vector3d Iz = ra;
  Vector3d Ez = -Iz/Iz.norm();

  Vector3d Iy = skew_symmetric(Ez)*rm;
  Vector3d Ey = Iy/Iy.norm();

  Vector3d Ix = skew_symmetric(Ey)*Ez;
  Vector3d Ex = Ix/Ix.norm();

  Rbn.col(0) = Ex;
  Rbn.col(1) = Ey;
  Rbn.col(2) = Ez;
  //Rbn:Abi
  return Rbn.transpose();
  //return Rbn;
}
