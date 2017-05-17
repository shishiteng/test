#include <iostream>
#include <math.h>
#include <Eigen/Dense>

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::VectorXd VXD;
typedef Eigen::MatrixXd MXD;

using namespace std;

V3D getRotationFromTwoNormals(const V3D& a, const V3D& b, const V3D& a_perp)
{
  const V3D cross = a.cross(b);
  const double crossNorm = cross.norm();
  const double c = a.dot(b);
  const double angle = std::acos(c);
  
  cout<<"    cross:"<<cross.transpose()<<endl;
  cout<<"crossNorm:"<<crossNorm<<endl;
  cout<<"      dot:"<<c<<endl;
  cout<<"    angle:"<<angle<<endl;
  cout<<"cross/nor:"<<cross.transpose()/crossNorm<<endl;

  if(crossNorm<1e-6){
    if(c>0){
      return cross;
    } else {
      return a_perp*M_PI;
    }
  } else {
    return cross*(angle/crossNorm);
  }
}




int main(int argc, char **argv)
{
  if(argc != 4) {
    cout<<"para error,正确形式:"<<endl;;
    cout<<"  tanSpace [v1] [v2] [v3]"<<endl;
    cout<<"注意：vector表达单位为rad"<<endl;;
    return -1;
  } 

  V3D e_x(1,0,0), e_y(0,1,0), e_z(0,0,1);

  float a1 = atof(argv[1]);
  float a2 = atof(argv[2]);
  float a3 = atof(argv[3]);
  float aNorm = pow(a1*a1+a2*a2+a3*a3, 0.5);
  V3D vec(a1/aNorm,a2/aNorm,a3/aNorm);

  cout <<"        a:"<<e_z.transpose()<<endl;
  cout <<"        b:"<<vec.transpose()<<endl;
  V3D v = getRotationFromTwoNormals(e_z,vec,e_x);
  cout <<" rotation:"<<v.transpose()<<endl;
  
  return 0;
}
