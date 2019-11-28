#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d) 
{
  Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
  m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
      v3d.z(), typename Derived::Scalar(0), -v3d.x(),
      -v3d.y(), v3d.x(), typename Derived::Scalar(0);
  return m;
}

int main()
{
  Eigen::Matrix3d i2l,rl,ri;
  rl<<0.75046891, 0.66090399, 0.00065571791, 
-0.66088754, 0.75044239, 0.0078817792, 
0.0047170408, -0.0063483808, 0.99996871;
  ri<<0.99506235, -0.093398564, -0.033579297,
0.093548991, 0.76955521, 0.63169074,
-0.0331579, -0.63171303, 0.7744928;
  i2l = rl*ri.inverse();
  cout<<"i2l:\n"<<i2l<<endl;

  Eigen::Quaterniond qi(ri);
  Eigen::Quaterniond ql(rl);
  qi = qi.normalized();
  ql = ql.normalized();
  printf("qi_wxyz: %f %f %f %f\n",qi.w(),qi.x(),qi.y(),qi.z());
  printf("ql_wxyz: %f %f %f %f\n",ql.w(),ql.x(),ql.y(),ql.z());
  return 0;
	

  Eigen::Matrix4d trans;
  trans<<
    0.999861,   -0.0166562, -8.88107e-06,    -0.888705,
   0.0166562,     0.999855,   0.00356171,     0.104492,
-5.04448e-05,  -0.00356136,     0.999994,   -0.0138095,
          0 ,          0,           0,           1;

Eigen::Matrix4d trans_inv;
  trans_inv<<
    0.999862,   0.0164032,  0.00264081,    0.857841,
 -0.0163966,    0.999862,  -0.0024895,   -0.118087,
-0.00268128,  0.00244586,    0.999993,   0.0174152,
          0 ,          0,           0,           1;

  Eigen::Matrix3d r = trans.block(0,0,3,3);
  Eigen::Vector3d t = trans.block(0,3,3,1);

  cout<<"trans:\n"<<trans<<endl;
  cout<<"r:\n"<<r<<endl;
  cout<<"eular:\n"<<r.eulerAngles(0,1,2).transpose()<<endl;

  cout<<"inverse:\n"<<trans.inverse()<<endl;
 
  Eigen::Matrix4d E = trans*trans_inv;
  Eigen::Matrix3d rr = E.block(0,0,3,3);
  cout<<"E:\n"<<E<<endl;
  
  Eigen::AngleAxisd r_vec(rr);
  cout<<"angle:\n"<<r_vec.angle()<<endl;
  cout<<"axis:\n"<<r_vec.axis()<<endl;

  Eigen::Matrix3d theta;
  theta<<
    0.99500418, -0.099833421, 0,
    0.099833421, 0.99500418, 0,
    0, 0, 1;
  Eigen::Matrix3d tt;
  tt<<
    -0.69492054, 0.713521, 0.089292862,
    -0.19200698, -0.30378506, 0.933192,
    0.69297814, 0.63134968, 0.34810749;
  cout<<"theta0:\n"<<theta<<endl;
  cout<<"theta:\n"<<tt.inverse()*theta*tt<<endl;

  Eigen::Matrix3d Rlw = theta;
  Eigen::Vector3d x(0.1,0.2,0.3);
  cout<<"[Rlw*X]x:\n"<<SkewSymmetric(Rlw*x)<<endl;
  cout<<"Rlw*[X]x*Rwl:\n"<<Rlw*SkewSymmetric(x)*Rlw.inverse()<<endl;
  cout<<"[X]x*Rlw:\n"<<SkewSymmetric(x)*Rlw<<endl;
  cout<<"Rwl*X:\n"<<Rlw.inverse()*x<<endl;

  Eigen::Quaterniond q0,q1;
  q0 = theta;
  q1 = tt;
  cout<<"q0*q1:\n"<<(q0*q1).toRotationMatrix()<<endl;
  cout<<"q1*q0:\n"<<(q1*q0).toRotationMatrix()<<endl;
  cout<<"m0*m1:\n"<<theta*tt<<endl;
  cout<<"m1*m0:\n"<<tt*theta<<endl;
}
