#include <stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"

#include <ctime>
#include <cstdlib>
#include <chrono>

using namespace cv;
using namespace std;

#define int8 char
#define int16 short
#define int32 int
#define uint8 unsigned char
#define uint16 unsigned short
#define uint32 unsigned int

#define MAX_FEATURES 256

typedef struct{
  uint16 x;
  uint16 y;
  int16 id;
}feature_t;

typedef struct{
  uint16 count;
  feature_t pt[MAX_FEATURES];
}features_t;



//declare
int gassianblur(uint8 *src,uint8* dst,uint16 width,uint16 height,uint16 kernel_size);
int pyrdown(uint8 *src,uint8* dst,uint16 width,uint16 height,uint16 kernel_size,uint16 num_levels);
int sobel(uint8 *src,uint8* dst,uint16 width,uint16 height,uint16 kernel_size);
int calc_klt(uint8 *prev_img,
	     uint8* next_img,
	     features_t *prev_features,
	     features_t *next_features,
	     uint16 width,
	     uint16 height,
	     uint16 win_size,
	     uint16 num_levels);



void test_gassianblur(Mat image)
{
  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::time_point<std::chrono::system_clock> start2, end2;
  
  uint8* src = (uint8*)image.data;
  uint16 width = image.cols;
  uint16 height = image.rows;
  uint16 kernel_size = 3;
  uint16 num_levels = 3;
  uint8 dst[640*480];
  memset(dst,0,sizeof(dst));

  start = std::chrono::system_clock::now();
  gassianblur(src,dst,width,height,kernel_size);
  //sobel(src,dst,width,height,kernel_size);
  Mat result(image.rows,image.cols,CV_8UC1,dst);
  end = std::chrono::system_clock::now();
  
  start2 = std::chrono::system_clock::now();
  Mat result2(image.rows, image.cols, CV_8UC1);  
  GaussianBlur(image, result2, Size(3, 3), 0, 0);
  //buildOpticalFlowPyramid(image, image_pyramid2,Size(21,21),num_levels, false, BORDER_REFLECT_101,BORDER_CONSTANT, false);
  end2 = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
  printf("c code cost:%f ms\n",elapsed_seconds.count() * 1000);
  printf("opencv cost:%f ms\n",elapsed_seconds2.count() * 1000);
  imshow("c_result", result);
  imshow("opencv_result", result2);
  imshow("diff(10x)", (result2-result)*10);
}

void test_pyrdown(Mat image)
{
  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::time_point<std::chrono::system_clock> start2, end2;
  
  uint8* src = (uint8*)image.data;
  uint16 width = image.cols;
  uint16 height = image.rows;
  uint16 kernel_size = 3;
  uint16 num_levels = 3;
  uint8 dst[640*480*2];
  memset(dst,0,sizeof(dst));

  int16 *dstDeriv = (int16*)malloc(640*480*2);
  memset(dstDeriv,0,640*480*2);
    
  start = std::chrono::system_clock::now(); //
  uint8 * pdst = dst;
  int16 * pdstDeriv = dstDeriv;
  vector<Mat> image_pyramid,image_derive;
  image_pyramid.push_back(image);
  pyrdown(src,pdst,width,height,kernel_size,num_levels);
  for(int i=1;i<num_levels;i++) {
    uint16 rows = image.rows>>i;
    uint16 cols = image.cols>>i;
    Mat result(rows,cols,CV_8UC1,pdst);
    image_pyramid.push_back(result);
    pdst += rows*cols;
  }
  for(int i=0;i<num_levels;i++) {
    uint16 rows = image.rows>>i;
    uint16 cols = image.cols>>i;
    Mat image_pyr;
    if(0==i)
      image_pyr= image;
    else
      image_pyr= image_pyramid[i];

    sobel(image_pyr.data,(uint8*)pdstDeriv,cols,rows,5);
    Mat m(rows,cols,CV_8UC2,(uint8*)pdstDeriv);
    image_derive.push_back(m);
    pdstDeriv += rows*cols;
#if 0	  
    std::vector<Mat> channels;
    split(m,channels);
    Mat resultX = channels[0]*8;
    Mat resultY = channels[1]*8;
#endif
  }
  end = std::chrono::system_clock::now();  //

  //opencv 用的卷积核是5x5，我用的是3x3的，所以结果略有差异
  start2 = std::chrono::system_clock::now(); //
  vector<Mat> image_pyramid2;
  buildOpticalFlowPyramid(image, image_pyramid2,Size(21,21),num_levels, true, BORDER_REFLECT_101,BORDER_CONSTANT, false);
  end2 = std::chrono::system_clock::now();//

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
  printf("c code cost:%f ms\n",elapsed_seconds.count() * 1000);
  printf("opencv cost:%f ms\n",elapsed_seconds2.count() * 1000);

  for(int i=0;i<num_levels*2;i+=2) {
    char str[64];
    // pyramid
    sprintf(str,"pyramid_c_%d",i);
    imshow(str,image_pyramid[i]);
    sprintf(str,"pyramid_opencv_%d",i);
    imshow(str,image_pyramid2[i*2]);

    sprintf(str,"pyramid_diff_%d-1x",i);
    imshow(str,(image_pyramid2[i*2]-image_pyramid[i]));
    //cout<< image_pyramid2[i] -image_pyramid[i]<<endl;

    // gradient
    sprintf(str,"gradient_c_%d",i);
    imshow(str,image_derive[i]);
    sprintf(str,"gradient_opencv_%d",i);
    imshow(str,image_pyramid2[i*2+1]);

    sprintf(str,"gradient_diff_%d-1x",i);
    imshow(str,(image_pyramid2[i*2+1]-image_derive[i]));
    //cout<< image_pyramid2[i] -image_pyramid[i]<<endl;
  }
}

void test_sobel(Mat image)
{
  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::time_point<std::chrono::system_clock> start2, end2;
  
  uint8* src = (uint8*)image.data;
  uint16 width = image.cols;
  uint16 height = image.rows;
  uint16 kernel_size = 3;
  uint16 num_levels = 4;
  uint8 *dst = (uint8*)malloc(640*480*2);
  memset(dst,0,640*480*2);

  //Mat image1;
  //GaussianBlur(image, image1, Size(3, 3), 0, 0);
  //src = (uint8*)image1.data;

  //opencv的结果更平滑，我这里还有点噪声
  start = std::chrono::system_clock::now();
  sobel(src,dst,width,height,kernel_size);
  Mat m(image.rows,image.cols,CV_8UC2,dst);
  std::vector<Mat> channels;
  split(m,channels);
  Mat resultX = channels[0];
  Mat resultY = channels[1];
  end = std::chrono::system_clock::now();
  
  start2 = std::chrono::system_clock::now();
  Mat resultX2(image.rows, image.cols, CV_8UC1);
  Mat resultY2(image.rows, image.cols, CV_8UC1);  
  Sobel(image,resultX2,CV_8UC1,1,0);
  Sobel(image,resultY2,CV_8UC1,0,1);
  end2 = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
  printf("c code cost:%f ms\n",elapsed_seconds.count() * 1000);
  printf("opencv cost:%f ms\n",elapsed_seconds2.count() * 1000);
  imshow("c_sobelX", resultX);
  imshow("c_sobelY", resultY);
  imshow("opencv_sobelX", resultX2);
  imshow("opencv_sobelY", resultY2);
  imshow("diffX-10x", (resultX2-resultX)*10);
  imshow("diffY-10x", (resultY2-resultY)*10);
  
  free(dst);
}

void test_klt(Mat image0,Mat image1)
{
  if(image0.empty() || image1.empty()) {
    fprintf(stderr,"input image invalid.\n");
    return;
  }

  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::time_point<std::chrono::system_clock> start2, end2;
  
  std::vector<cv::Point2f> prevPts,nextPts;
  std::vector<unsigned char> status;
  std::vector<float> err;

  // detect corners
  goodFeaturesToTrack(image0, prevPts, 150, 0.01, 20);

  //format input features
  features_t prev_features,next_features;
  prev_features.count = prevPts.size();
  for(int i=0;i<prevPts.size();i++) {
    prev_features.pt[i].x = (int)prevPts[i].x;
    prev_features.pt[i].y = (int)prevPts[i].y;
    prev_features.pt[i].id = i;
    //printf("__%d[%d,%d]\n",i,prev_features.pt[i].x,prev_features.pt[i].y);
  }

  // my test
  start = std::chrono::system_clock::now();
  calc_klt(image0.data,image1.data,
	  &prev_features,&next_features,
	  image0.cols,image0.rows,
	  21,4);
  end = std::chrono::system_clock::now();

  // opencv test
  start2 = std::chrono::system_clock::now();
  calcOpticalFlowPyrLK(image0, image1, prevPts, nextPts, status, err,
		       Size(21,21),
		       3,
		       TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01));
  end2 = std::chrono::system_clock::now();

  int track_cnt,track_cnt2 = 0;
  cv::Mat result(image0.rows,image0.cols,CV_8UC3);
  cv::cvtColor(image0,result,COLOR_GRAY2RGB);
  for (unsigned int j = 0; j < prevPts.size(); j++) {
    if(next_features.pt[j].id != -1) {
      cv::Point2f pt(next_features.pt[j].x,next_features.pt[j].y);
      cv::line(result, prevPts[j], pt, cv::Scalar(0, 0, 255), 2);
#if 0
      printf("my_opticalflow:     %d [%.0f,%.0f] to [%.1f,%.1f] v_[%.3f,%.3f]\n",j,
	     prevPts[j].x,
	     prevPts[j].y,
	     pt.x,
	     pt.y,
	     pt.x-prevPts[j].x,
	     pt.y-prevPts[j].y);
#endif
      track_cnt++;
    }
    
    //opencv
    if(status[j]) {
      cv::line(result, prevPts[j], nextPts[j], cv::Scalar(0, 255, 0), 1);
      cv::circle(result, prevPts[j], 2,cv::Scalar(255 , 0, 0), 2);
#if 0
      printf("opencv_opticalflow: %d [%.0f,%.0f] to [%.1f,%.1f] v_[%.3f,%.3f]\n",j,
	     prevPts[j].x,
	     prevPts[j].y,
	     nextPts[j].x,
	     nextPts[j].y,
	     nextPts[j].x-prevPts[j].x,
	     nextPts[j].y-prevPts[j].y);
#endif
      track_cnt2++;
    }
  }

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
  printf("c code cost:%f ms (%d/%d)\n",elapsed_seconds.count()*1000,prevPts.size(),track_cnt);
  printf("opencv cost:%f ms (%d/%d)\n",elapsed_seconds2.count()*1000,prevPts.size(),track_cnt2);
  imshow("opencv-result",result);
}

int main(int argc, char** argv)
{
  Mat image,image1;
  switch(argc) {
  case 2: 
    image = imread(argv[1],IMREAD_GRAYSCALE);
    break;
  case 3:
    image = imread(argv[1],IMREAD_GRAYSCALE);
    image1 = imread(argv[2],IMREAD_GRAYSCALE);
    break;
  default:
    image = imread("image/lena.jpg",IMREAD_GRAYSCALE);
    break;
  } 
  
  if(image.empty()) {
    fprintf(stderr,"open image failed.\n");
    return -1;
  }

  //test_gassianblur(image);
  //test_pyrdown(image);
  //test_sobel(image);
  test_klt(image,image1);

  waitKey(0);
  return 0;
}
