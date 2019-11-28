#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


int main()
{
  double i2c[] = {
      0.00297605,  0.99999533, -0.00069284,  0.04322073,
     -0.99999237,  0.00297779,  0.00252793,  0.00140886,
      0.00252998,  0.00068531,  0.99999656, -0.02625511,
    0,            0,           0,          1.
  };

  cv::Mat I2C(4,4,CV_64FC1,i2c);

#if 0
  double c2l[] = {
   -0.068012, 0.00632638,   0.997664,  0.0489667,
   -0.997558,  0.0154698, -0.0681029,   0.042206,
   -0.0158645,   -0.99986,  0.0052588,  0.0731468,
    0,          0,          0,          1,
  };
#else
double c2l[] = {
    -0.0766806,   -0.22346,   0.971705, -0.0480779,
    -0.997003, 0.00634704, -0.0772165,   0.193515,
     0.0110871,  -0.974694,  -0.223273,   0.124106,
    0,          0,          0,          1
  };
#endif

  cv::Mat C2L(4,4,CV_64FC1,c2l);

  cv::Mat I2L = C2L * I2C;

  cout << "c2l:\n" << C2L << endl;
  cout << "l2c:\n" << C2L.inv() << endl;
  cout << "I2L:\n" << I2L << endl; 
  cout << "L2I:\n" << I2L.inv() << endl;
  
  cv::Mat Rvec(3,1,CV_32FC1);
  Rodrigues(I2L.rowRange(0,3).colRange(0,3),Rvec);
  cout<< "\n Rodrigues(I2L):"<<Rvec.t()*57.3f<<endl;

double a =1234523213.3213055; 
cout<<a<<endl;//输出"12345"
cout<<setiosflags(ios::fixed)<<a<<endl;
cout<<setiosflags(ios::fixed)<<setprecision(3)<<a<<endl;
cout<<setiosflags(ios::scientific)<<a<<endl;
cout<<setprecision(3)<<a<<endl;


double i2w[] = {
-0.17429149, -0.010090056, -0.98463196, 0,
-0.010200803, 0.99991232, -0.0084407991, 0,
0.98463076, 0.0085728774, -0.17437911, 0,
0,          0,          0,          1
};

double l2w[] = {
-1, 0, 0, 0,
0, -1, 0, 0,
0, 0, 1, 0,
0, 0, 0, 1
};

cv::Mat I2W(4,4,CV_64FC1,i2w);
cv::Mat L2W(4,4,CV_64FC1,l2w);
cout << "{i2l}:\n" << L2W.inv()*I2W << endl;

return 0;

}
