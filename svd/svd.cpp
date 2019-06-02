//https://www.jianshu.com/p/13a81c5b4b9d

#include "opencv2/opencv.hpp"
using namespace cv;

//参数分别为输入图像，输出图像，压缩比例
void SVDRESTRUCT(const cv::Mat &inputImg, cv::Mat &outputImg, double  theratio)
{
  cv::Mat tempt;
  cv::Mat U, W, V;
  inputImg.convertTo(tempt, CV_32FC1);
  cv::SVD::compute(tempt, W, U, V);
  cv::Mat w = Mat::zeros(Size(W.rows, W.rows), CV_32FC1);
  int len = theratio * W.rows;
  for (int i = 0; i < len; ++i)
    w.ptr<float>(i)[i] = W.ptr<float>(i)[0];
  cv::Mat result = U * w*V;
  result.convertTo(outputImg, CV_8UC1);
}

int main(int argc, char *argv[])
{
  if (argc<3)
  {
    std::cout<<"Usage: ./svd [file name] [theratio]"<<std::endl;
    return -1;
  }

  
  cv::Mat scrX = imread(argv[1], 0);
  cv::Mat result;
  SVDRESTRUCT(scrX, result, atof(argv[2]));
  cv::imshow("src", scrX);
  cv::imshow("dst", result);
  waitKey(0);
}

