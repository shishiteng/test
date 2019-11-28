#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define PI 3.141593

//旋转矩阵得到四元数
cv::Mat Matrix2Quaternion(cv::Mat matrix)
{
  float tr, qx, qy, qz, qw;

  // 计算矩阵轨迹
  float a[4][4] = {0};
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      a[i][j]=matrix.at<float>(i,j);
  
  // I removed + 1.0f; see discussion with Ethan
  float trace = a[0][0] + a[1][1] + a[2][2]; 
  if( trace > 0 ) {
    // I changed M_EPSILON to 0
    float s = 0.5f / sqrtf(trace+ 1.0f);
    qw = 0.25f / s;
    qx = ( a[2][1] - a[1][2] ) * s;
    qy = ( a[0][2] - a[2][0] ) * s;
    qz = ( a[1][0] - a[0][1] ) * s;
  } else {
    if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
      float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
      qw = (a[2][1] - a[1][2] ) / s;
      qx = 0.25f * s;
      qy = (a[0][1] + a[1][0] ) / s;
      qz = (a[0][2] + a[2][0] ) / s;
    } else if (a[1][1] > a[2][2]) {
      float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
      qw = (a[0][2] - a[2][0] ) / s;
      qx = (a[0][1] + a[1][0] ) / s;
      qy = 0.25f * s;
      qz = (a[1][2] + a[2][1] ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
      qw = (a[1][0] - a[0][1] ) / s;
      qx = (a[0][2] + a[2][0] ) / s;
      qy = (a[1][2] + a[2][1] ) / s;
      qz = 0.25f * s;
    }    
  }

  float q[] = {qw,qx,qy,qz};
  //cout<< "\n quaternion:"<<cv::Mat(4,1,CV_32FC1,q).t()<<endl;
  return cv::Mat(4,1,CV_32FC1,q).clone();
}

//四元数得到欧拉角
cv::Mat Quaternion2Euler(float *q)
{
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  float ret[3] = {0};
  //cv::Mat ret(3,1,CV_32FC1);
 
  float test = x*y + z*w;
  if (test > 0.4999f) {
    ret[2] = 2.0f * atan2f(x, w);
    ret[1] = PI/2;
    ret[0] = 0.0f;
    return cv::Mat(3,1,CV_32FC1,ret).clone();
  }
  if (test < -0.4999f) {
    ret[2] = 2.0f * atan2f(x, w);
    ret[1] = -PI/2;
    ret[0] = 0.0f;
    return cv::Mat(3,1,CV_32FC1,ret).clone();
  }
  float sqx = x * x;
  float sqy = y * y;
  float sqz = z * z;
  ret[2] = atan2f(2.0f * y * w - 2.0f * x * z, 1.0f - 2.0f * sqy - 2.0f * sqz);
  ret[1] = asin(2.0f * test);
  ret[0] = atan2f(2.0f * x * w - 2.0f * z * y, 1.0f - 2.0f * sqx - 2.0f * sqz);
      
  ret[0] *= 180/PI;
  ret[1] *= 180/PI;
  ret[2] *= 180/PI;

  return cv::Mat(3,1,CV_32FC1,ret).clone();
}

//四元数得到欧拉角
cv::Mat Quaternion2Euler(cv::Mat q)
{
  float w = q.at<float>(0);
  float x = q.at<float>(1);
  float y = q.at<float>(2);
  float z = q.at<float>(3);

  float ret[3] = {0};
  //cv::Mat ret(3,1,CV_32FC1);
 
  float test = x*y + z*w;
  if (test > 0.4999f) {
    ret[2] = 2.0f * atan2f(x, w);
    ret[1] = PI/2;
    ret[0] = 0.0f;
    return cv::Mat(3,1,CV_32FC1,ret).clone();
  }
  if (test < -0.4999f) {
    ret[2] = 2.0f * atan2f(x, w);
    ret[1] = -PI/2;
    ret[0] = 0.0f;
    return cv::Mat(3,1,CV_32FC1,ret).clone();
  }
  float sqx = x * x;
  float sqy = y * y;
  float sqz = z * z;

  ret[0] = atan2f(2.0f*(y*w-x*z), 1.0f-2.0f*(sqy+sqz));
  ret[1] = asin(2.0f * test);
  ret[2] = atan2f(2.0f*(x*w - y*z), 1.0f-2.0f*(sqx+sqz));

  return cv::Mat(3,1,CV_32FC1,ret).clone();
}

cv::Mat Matrix2Euler(cv::Mat matrix)
{
  cv::Mat q = Matrix2Quaternion(matrix);
  cv::Mat angle = Quaternion2Euler(q);
  return angle.clone();

  float m[4][4] = {0};
  for(int a=0;a<4;a++)
    for(int b=0;b<4;b++)
      m[a][b]=matrix.at<float>(a,b);
  
  float a[3];
  a[0] = atan2f(m[2][1],m[2][2]) *180/PI;
  a[1] = atan2f(-m[2][0], sqrtf(m[2][1]*m[2][1] +  m[2][2]*m[2][2])) *180/PI;
  a[2] = atan2f(m[1][0], m[0][0]) *180/PI;
  return cv::Mat(3,1,CV_32FC1,a).clone();
}

// 由欧拉角创建四元数
cv::Mat Euler2Quaternion(float *angle)
{
  float heading = angle[0];
  float attitude = angle[1];
  float bank = angle[2];

  float c1 = cos(heading/2);
  float s1 = sin(heading/2);
  float c2 = cos(attitude/2);
  float s2 = sin(attitude/2);
  float c3 = cos(bank/2);
  float s3 = sin(bank/2);
  float c1c2 = c1*c2;
  float s1s2 = s1*s2;
  float w =c1c2*c3 - s1s2*s3;
  float x =c1c2*s3 + s1s2*c3;
  float y =s1*c2*c3 + c1*s2*s3;
  float z =c1*s2*c3 - s1*c2*s3;
  float q[4] = {w,x,y,z};
  cv::Mat ret(4,1,CV_32FC1,q);
  return ret.clone();

}

cv::Mat Euler2Quaternion(cv::Mat Angle)
{
  //angle:roll pitch yaw
  //    q:w x y z
  float angle[3];
  angle[0] = Angle.at<float>(0);
  angle[1] = Angle.at<float>(1);
  angle[2] = Angle.at<float>(2);

  return Euler2Quaternion(angle).clone();
}

// 由旋转四元数推导出矩阵
cv::Mat Quaternion2Matrix (cv::Mat q)
{
  float w = q.at<float>(0);
  float x = q.at<float>(1);
  float y = q.at<float>(2);
  float z = q.at<float>(3);

  float xx = x*x;
  float yy = y*y;
  float zz = z*z;
  float xy = x*y;
  float wz = w*z;
  float wy = w*y;
  float xz = x*z;
  float yz = y*z;
  float wx = w*x;

  float ret[4][4];
  ret[0][0] = 1.0f-2*(yy+zz);
  ret[0][1] = 2*(xy-wz);
  ret[0][2] = 2*(wy+xz);
  ret[0][3] = 0.0f;
 
  ret[1][0] = 2*(xy+wz);
  ret[1][1] = 1.0f-2*(xx+zz);
  ret[1][2] = 2*(yz-wx);
  ret[1][3] = 0.0f;
 
  ret[2][0] = 2*(xz-wy);
  ret[2][1] = 2*(yz+wx);
  ret[2][2] = 1.0f-2*(xx+yy);
  ret[2][3] = 0.0f;
 
  ret[3][0] = 0.0f;
  ret[3][1] = 0.0f;
  ret[3][2] = 0.0f;
  ret[3][3] = 1.0f;
 
  return cv::Mat(4,4,CV_32FC1,ret).clone();
}

//欧拉角转旋转矩阵,Euler:弧度表示
cv::Mat Euler2Matrix(float *angle)
{
  cv::Mat q = Euler2Quaternion(angle);
  return Quaternion2Matrix(q).clone();
}

//欧拉角转旋转矩阵
cv::Mat Euler2Matrix(cv::Mat angle)
{
  cv::Mat q = Euler2Quaternion(angle);
  return Quaternion2Matrix(q).clone();
}


int main()
{
  float r[] = {
               1, -0.000604868,  8.96213e-05,  0.000380829,
 0.000604867,            1, -8.55095e-05,  -0.00495968,
-8.95696e-05,  8.55638e-05,            1,   0.00250944,
  0, 0, 0, 1
  };

  cv::Mat R(4,4,CV_32FC1,r);
  cout<< "\n Matrix:\n"<<R<<endl;
  
  cv::Mat Rvec(3,1,CV_32FC1);
  Rodrigues(R.rowRange(0,3).colRange(0,3),Rvec);
  cout<< "\n Rodrigues:"<<Rvec.t()<<endl;
  
  cv::Mat a1 = Matrix2Euler(R);
  cout<< "\n Matrix 2 Eular:"<<a1.t()*57.3f<<endl;
  
  //矩阵推四元数,四元数推矩阵,OK
  cout<<"\n----------矩阵--四元数-------------"<<endl;
  Mat quaternion = Matrix2Quaternion(R);
  Mat matrix = Quaternion2Matrix(quaternion);
  Mat quaternion2 = Matrix2Quaternion(matrix);
  cout<<" quaternion(qw qx qy qz):"<<quaternion.t()<<endl;
  cout<<" matrix:\n"<<matrix<<endl;
  cout<<" quaternion--:"<<quaternion2.t()<<endl;

  cout<<"\n---------四元数--欧拉角------------"<<endl;
  //四元数推欧拉角，欧拉角推四元数
  Mat euler = Quaternion2Euler(quaternion);
  Mat qq = Euler2Quaternion(euler);
  Mat euler2 = Quaternion2Euler(qq);
  cout<<" euler:"<<euler.t()*57.3f<<endl;
  cout<<" quaternion 2:"<<qq.t()<<endl;
  cout<<" euler2:"<<euler2.t()*57.3f<<endl;

  cout<<"\n---------欧拉角--矩阵----------"<<endl;
  cv::Mat matrix2 = Euler2Matrix(euler2);
  cv::Mat euler3 = Matrix2Euler(matrix2);
  cout<< " euler:"<<euler2.t()*57.3f<<endl;
  cout<< " matirx:\n"<<matrix2<<endl;
  cout<< " euler--:"<<euler3.t()*57.3f<<endl;
  cv::Mat rod;
  Rodrigues(matrix2.rowRange(0,3).colRange(0,3),rod);
  cout<< " rod angle1:"<<rod.t()*57.3f<<endl;
  Rodrigues(matrix.rowRange(0,3).colRange(0,3),rod);
  cout<< " rod angle2:"<<rod.t()*57.3f<<endl;

  cv::Mat T = R.inv()*matrix;
  cout << " T1:"<<Matrix2Euler(T).t()*57.3f<<endl;
  T = R.inv()*matrix2;
  cout << " T2:"<<Matrix2Euler(T).t()*57.3f<<endl;
}








