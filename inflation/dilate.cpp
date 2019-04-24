#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main()
{
  Mat img = imread("a.png");
  imshow("原始图", img);

  Mat gray;
  cvtColor(img, gray, COLOR_BGR2GRAY);
  //imshow("灰度图", gray);

  img = 255 - gray;
  //threshold(img, result, 170, 255, CV_THRESH_BINARY);
  //imshow("反色图", img);
  
  Mat out;
  //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
  Mat element = getStructuringElement(MORPH_ELLIPSE, Size(50, 50));
  //Mat element = getStructuringElement(MORPH_RECT, Size(50, 50)); 
  dilate(img, out, element);
  //imshow("膨胀操作", out);

  vector<vector<Point>>contours;
  vector<Vec4i>hierarchy;
  findContours(out, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
  //findContours(out, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  printf("contours_size:%d\n",contours.size());
  printf("hierarchy_size:%d\n",hierarchy.size());
  
  //绘制轮廓图
  Mat dst = Mat(gray.size(), CV_8UC1, 255);
  for (int i = 0; i < hierarchy.size(); i++) {
    Scalar color(0);
    drawContours( dst, contours, i, color);
  }
  imshow("轮廓图", dst);
  
  Mat inflation = dst - (255-gray);
  imshow("inflation", inflation);
  waitKey(0);
  
}
