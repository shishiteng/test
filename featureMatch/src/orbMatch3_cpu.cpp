#include <iostream>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <stdio.h>

using namespace std;
using namespace cv;

int main(int argc,char** argv)
{
  int64 st, et;
  if(argc != 3){
    printf("please input:orbMatch image1 image2\n");
    return -1;
  }

  Mat img_1 = imread(argv[1],IMREAD_GRAYSCALE);
  Mat img_2 = imread(argv[2],IMREAD_GRAYSCALE);
  if (!img_1.data || !img_2.data){
      cout << "error reading images " << endl;
      return -1;
  }

  vector<KeyPoint> keyPoints_1, keyPoints_2;
  Mat descriptors_1, descriptors_2;

  st = cvGetTickCount();
  //static Ptr<ORB> create (int nfeatures,float scaleFactor,int nlevels,int edgeThreshold,int firstLevel,int WTA_K,int scoreType,int patchSize,int fastThreshold)
  Ptr<ORB> orb = ORB::create(2000, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);

#if 0
  orb->detect(img_1, keyPoints_1, noArray());
  orb->detect(img_2, keyPoints_2, noArray());
  et = cvGetTickCount();
  printf("orb detect cost: %fms\n", (et-st)/(double)cvGetTickFrequency()/1000.);
  cout <<"-- orb1 features:"<<keyPoints_1.size()<<endl;
  cout <<"-- orb2 features:"<<keyPoints_2.size()<<endl;

  st = cvGetTickCount();
  orb->compute(img_1, keyPoints_1, descriptors_1);
  orb->compute(img_2, keyPoints_2, descriptors_2);
  et = cvGetTickCount();
  printf("compute descriptors cost: %fms\n", (et-st)/(double)cvGetTickFrequency()/1000.);
#else
  orb->detectAndCompute(img_1, noArray(), keyPoints_1, descriptors_1);
  orb->detectAndCompute(img_2, noArray(), keyPoints_2, descriptors_2);
  et = cvGetTickCount();
  printf("orb detect and compute cost: %fms\n", (et-st)/(double)cvGetTickFrequency()/1000.);
  cout <<"-- orb1 features:"<<keyPoints_1.size()<<endl;
  cout <<"-- orb2 features:"<<keyPoints_2.size()<<endl;
#endif

  st = cvGetTickCount();
  //static Ptr<BFMatcher> create (int normType=NORM_L2, bool crossCheck=false);
  //static Ptr<BFMatcher> matcher = BFMatcher::create();
  BFMatcher matcher;
  vector<DMatch> matches;
  matcher.match(descriptors_1, descriptors_2, matches);

  double max_dist = 0; double min_dist = 100;
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ ){ 
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
  //-- Draw only "good" matches (i.e. whose distance is less than 0.6*max_dist )
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;
  for( int i = 0; i < descriptors_1.rows; i++ ) { 
    if( matches[i].distance < 0.75*max_dist )  { 
      good_matches.push_back( matches[i]); 
    }
  }

  et = cvGetTickCount();
  printf("match  time: %f\n", (et-st)/(double)cvGetTickFrequency()/1000.);
  printf("matchs num: %d\n", matches.size());
  printf("good matchs num: %d\n", good_matches.size());

  if(good_matches.size() < 4){
    printf("not enough matches,exit.\n");
    return -1;
  }

  Mat img_matches;
  drawMatches(img_1, keyPoints_1, img_2, keyPoints_2,
	      good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
	      vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  imshow( "Match", img_matches);
  cvWaitKey();

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ ) {
    //-- Get the keypoints from the good matches
    obj.push_back( keyPoints_1[ good_matches[i].queryIdx ].pt );
    scene.push_back( keyPoints_2[ good_matches[i].trainIdx ].pt );
  }

  st = cvGetTickCount();
  Mat H = findHomography( obj, scene, CV_RANSAC );
  et = cvGetTickCount();
  printf("ransac  time: %f\n", (et-st)/(double)cvGetTickFrequency()/1000.);

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(5);
  obj_corners[0] = cvPoint(0,0); 
  obj_corners[1] = cvPoint( img_1.cols, 0 );
  obj_corners[2] = cvPoint( img_1.cols, img_1.rows );
  obj_corners[3] = cvPoint( 0, img_1.rows );
  obj_corners[4] = cvPoint( img_1.cols/2, img_1.rows/2 );
  std::vector<Point2f> scene_corners(5);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_1.cols, 0), scene_corners[1] + Point2f( img_1.cols, 0), Scalar(255,  0, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_1.cols, 0), scene_corners[2] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_1.cols, 0), scene_corners[3] + Point2f( img_1.cols, 0), Scalar( 0, 0, 255), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_1.cols, 0), scene_corners[0] + Point2f( img_1.cols, 0), Scalar( 255, 255, 0), 4 );
  line( img_matches, scene_corners[4] + Point2f( img_1.cols, 0), scene_corners[0] + Point2f( img_1.cols, 0), Scalar( 255, 255, 0), 4 );

  //-- Show detected matches
  imshow( "Good Matches & Object detection", img_matches );
  waitKey(0);

  //投影.误差测量
  vector<Point2f> reproj;
  reproj.resize(obj.size());
  perspectiveTransform(obj, reproj, H);

  Mat diff;
  diff = Mat(reproj) - Mat(scene);

  int inlier = 0;
  double err_sum = 0;
  for(int i = 0; i < diff.rows; i++){
    float* ptr = diff.ptr<float>(i);
    float err = ptr[0]*ptr[0] + ptr[1]*ptr[1];
    if(err < 25.f){
      inlier++;
      err_sum += sqrt(err);
    }
  }
  printf("inlier num: %d\n", inlier);
  printf("ratio %f\n", inlier / (float)(diff.rows));
  printf("mean reprojection error: %f\n", err_sum / inlier);

  return 0;
}
