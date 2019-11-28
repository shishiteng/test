#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;

// template <typename Derived>
// inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d)
// {
//   Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
//   m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
//       v3d.z(), typename Derived::Scalar(0), -v3d.x(),
//       -v3d.y(), v3d.x(), typename Derived::Scalar(0);
//   return m;
// }

Eigen::Matrix3d ric = Eigen::Matrix3d::Identity();
std::vector<Eigen::Matrix3d> Rc;
std::vector<Eigen::Matrix3d> Rc_g;
std::vector<Eigen::Matrix3d> Rimu;
int frame_count = 0;

/*************************************utils start********************************************/
static const Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
  ans << typename Derived::Scalar(0), -q(2), q(1),
      q(2), typename Derived::Scalar(0), -q(0),
      -q(1), q(0), typename Derived::Scalar(0);
  return ans;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived> &q)
{
  Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
  typename Derived::Scalar q4 = q.w();
  m.block(0, 0, 3, 3) << q4 * I3x3 + skewSymmetric(vq);
  m.block(3, 0, 1, 3) << -vq.transpose();
  m.block(0, 3, 3, 1) << vq;
  m(3, 3) = q4;
  return m;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(const Eigen::QuaternionBase<Derived> &p)
{
  Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
  typename Derived::Scalar p4 = p.w();
  m.block(0, 0, 3, 3) << p4 * I3x3 - skewSymmetric(vp);
  m.block(3, 0, 1, 3) << -vp.transpose();
  m.block(0, 3, 3, 1) << vp;
  m(3, 3) = p4;
  return m;
}

template <typename T>
inline Eigen::Matrix<T, 4, 4> LeftQuatMatrix(const Eigen::Matrix<T, 4, 1> &q)
{
  Eigen::Matrix<T, 4, 4> m;
  Eigen::Matrix<T, 3, 1> vq{q.x(), q.y(), q.z()};
  T q4 = q.w();
  m.block(0, 0, 3, 3) << q4 * I3x3 + skewSymmetric(vq);
  m.block(3, 0, 1, 3) << -vq.transpose();
  m.block(0, 3, 3, 1) << vq;
  m(3, 3) = q4;
  return m;
}

template <typename T>
inline Eigen::Matrix<T, 4, 4> RightQuatMatrix(const Eigen::Matrix<T, 4, 1> &p)
{
  Eigen::Matrix<T, 4, 4> m;
  Eigen::Matrix<T, 3, 1> vp{p.x(), p.y(), p.z()};
  T p4 = p.w();
  m.block(0, 0, 3, 3) << p4 * I3x3 - skewSymmetric(vp);
  m.block(3, 0, 1, 3) << -vp.transpose();
  m.block(0, 3, 3, 1) << vp;
  m(3, 3) = p4;
  return m;
}

/*************************************utils end********************************************/

bool CalibrationExRotation(Eigen::Quaterniond delta_q_lidar,
                           Eigen::Quaterniond delta_q_imu,
                           Eigen::Matrix3d &calib_ric_result)
{
  Rc.push_back(delta_q_lidar.toRotationMatrix());
  Rimu.push_back(delta_q_imu.toRotationMatrix());
  Rc_g.push_back(ric.inverse() * delta_q_imu * ric);
  frame_count++;

  Eigen::MatrixXd A(frame_count * 4, 4);
  A.setZero();
  int sum_ok = 0;
  for (int i = 0; i < frame_count; i++)
  {
    Eigen::Quaterniond r1(Rc[i]);
    Eigen::Quaterniond r2(Rc_g[i]);

    double angular_distance = 57.3f * r1.angularDistance(r2);
    double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
    printf("%d %f huber_%f\n", i, angular_distance, huber);

    ++sum_ok;
    Eigen::Matrix4d L, R;

    double w = Eigen::Quaterniond(Rc[i]).w();
    Eigen::Vector3d q = Eigen::Quaterniond(Rc[i]).vec();
    L.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() + skewSymmetric(q);
    L.block<3, 1>(0, 3) = q;
    L.block<1, 3>(3, 0) = -q.transpose();
    L(3, 3) = w;

    Eigen::Quaterniond R_ij(Rimu[i]);
    w = R_ij.w();
    q = R_ij.vec();
    R.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() - skewSymmetric(q);
    R.block<3, 1>(0, 3) = q;
    R.block<1, 3>(3, 0) = -q.transpose();
    R(3, 3) = w;

    A.block<4, 4>(i * 4, 0) = huber * (L - R);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Eigen::Quaterniond estimated_R(x);
  ric = estimated_R.toRotationMatrix().inverse();
  //cout << svd.singularValues().transpose() << endl;
  cout << "ric:\n"
       << ric << endl;

  Eigen::Vector3d ric_cov;
  ric_cov = svd.singularValues().tail<3>();
  cout << "ric_cov:" << ric_cov(1) << endl;

  calib_ric_result = ric;

  // if (frame_count >= 4 && ric_cov(1) > 0.25)
  if (ric_cov(1) > 0.25)
  {
    //calib_ric_result = ric;
    return true;
  }
  else
    return false;
}

bool EstimateExtrinsicRotation(Eigen::Quaterniond delta_q_lidar,
                               Eigen::Quaterniond delta_q_imu,
                               Eigen::Quaterniond &transform_lb)
{
  Rc.push_back(delta_q_lidar.toRotationMatrix());
  Rimu.push_back(delta_q_imu.toRotationMatrix());
  Rc_g.push_back(ric.inverse() * delta_q_imu * ric);
  frame_count++;

  // bool EstimateExtrinsicRotation(std::vector<Eigen::Quaterniond> all_laser_transforms,
  //                                Eigen::Quaterniond transform_lb)
  // {
  Eigen::Quaterniond rot_bl = transform_lb.inverse();
  // size_t window_size = all_laser_transforms.size();

  Eigen::MatrixXd A(frame_count * 4, 4);
  A.setZero();

  for (size_t i = 0; i < frame_count; ++i)
  {
    Eigen::Quaterniond delta_qij_imu(Rimu[i]);
    Eigen::Quaterniond delta_qij_laser(Rc[i]);
    Eigen::Quaterniond delta_qij_laser_from_imu = rot_bl.conjugate() * delta_qij_imu * rot_bl;
    double angular_distance = 57.3f * delta_qij_laser.angularDistance(delta_qij_laser_from_imu);
    double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
    printf("%d %f huber_%f\n", i, angular_distance, huber);

    Eigen::Matrix4d lq_mat = LeftQuatMatrix(delta_qij_laser);
    Eigen::Matrix4d rq_mat = RightQuatMatrix(delta_qij_imu);

    A.block<4, 4>(i * 4, 0) = huber * (lq_mat - rq_mat);

    // cout << (lq_mat * transform_lb.toRotationMatrix().transpose()) << endl;
    // cout << (delta_qij_laser * transform_lb).coeffs().transpose() << endl;
    // cout << (rq_mat * transform_lb.toRotationMatrix()).transpose() << endl;
    // cout << (transform_lb * delta_qij_imu).coeffs().transpose() << endl;
  }

  //  DLOG(INFO) << ">>>>>>> A <<<<<<<" << endl << A;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Eigen::Quaterniond estimated_qlb(x);

  transform_lb = estimated_qlb;

  Eigen::Vector3d cov = svd.singularValues().tail<3>();
  cout << "ric_cov:" << cov(1) << endl;
  //  DLOG(INFO) << "x: " << x.transpose();
  //DLOG(INFO) << "extrinsic rotation: " << transform_lb.rot.coeffs().transpose();
  std::cout << "extrinsic rotation: \n"
            << transform_lb.toRotationMatrix() << std::endl;
  std::cout << "singular values: " << svd.singularValues().transpose() << std::endl;
  //  cout << x.transpose << endl;
  //  DLOG(INFO) << "singular values: " << svd.singularValues().transpose();
  //  cout << cov << endl;

  // NOTE: covariance 0.25
  if (cov(1) > 0.25)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int main()
{
  int iter_count = 1;
  while (1)
  {
    cout << "------\niter_count: " << iter_count++ << endl;
    double q[4];
    printf("qi(wxyz): ");
    scanf("%lf%lf%lf%lf", &q[0], &q[1], &q[2], &q[3]);
    Eigen::Quaterniond qi(q[0], q[1], q[2], q[3]);

    printf("ql(wxyz): ");
    scanf("%lf%lf%lf%lf", &q[0], &q[1], &q[2], &q[3]);
    Eigen::Quaterniond ql(q[0], q[1], q[2], q[3]);

    qi = qi.normalized();
    ql = ql.normalized();

    Eigen::Quaterniond qi2l(1, 0, 0, 0);

#if 0
    Eigen::Matrix3d calib_ric_result;
    if (CalibrationExRotation(ql, qi, calib_ric_result))
    {
      qi2l = calib_ric_result.transpose();
      cout << "calib_I2L_result:\n"
           << qi2l.toRotationMatrix() << endl;
    }
    else
      cout << "failed:\n";
    qi2l = calib_ric_result.transpose();
#else

    if (EstimateExtrinsicRotation(ql, qi, qi2l))
    {
      cout << "calib_I2L_result:\n"
           << qi2l.toRotationMatrix() << endl;
    }
    else
      cout << "failed:\n";
#endif

    cout << "calib_I2L_result:\n"
         << qi2l.toRotationMatrix() << endl;

    Eigen::Quaterniond q_I2L = qi2l;
    Eigen::Quaterniond qi_ref, ql_ref;
    qi_ref = q_I2L.inverse() * ql * q_I2L;
    ql_ref = q_I2L * qi * q_I2L.inverse();
    // ql_ref =  q_I2L.inverse() * qi * q_I2L;
    printf("qi_ref: %lf %lf %lf %lf\n", qi_ref.w(), qi_ref.x(), qi_ref.y(), qi_ref.z());
    printf("ql_ref: %lf %lf %lf %lf\n", ql_ref.w(), ql_ref.x(), ql_ref.y(), ql_ref.z());

    Eigen::Quaterniond qleft = q_I2L * qi;
    Eigen::Quaterniond qright = ql * q_I2L;
    printf("qleft: %lf %lf %lf %lf\n", qleft.w(), qleft.x(), qleft.y(), qleft.z());
    printf("qright: %lf %lf %lf %lf\n", qright.w(), qright.x(), qright.y(), qright.z());
  }

  return 0;
}
