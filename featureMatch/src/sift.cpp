#include "opencv2/opencv.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>//SIFT

//#include <opencv2/legacy/legacy.hpp>//BFMatch暴力匹配
#include <vector>
#include<iostream>
using namespace std;
using namespace cv;

int main(int argc,char **argv)
{
    Mat input1 = imread(argv[1], 1);
    Mat input2 = imread(argv[2], 1);
    Mat imgGray1, imgGray2;
    //转换灰度图
    cvtColor(input1, imgGray1, CV_BGR2GRAY);
    cvtColor(input2, imgGray2, CV_BGR2GRAY);

    Ptr<xfeatures2d::SIFT> sift = xfeatures2d::SIFT::create(1000,1);
    vector<KeyPoint> keypoint1, keypoint2;
    Mat descriptor1, descriptor2;
    sift->detectAndCompute(imgGray1, noArray(), keypoint1, descriptor1);
    sift->detectAndCompute(imgGray2, noArray(), keypoint2, descriptor2);
    

    //BruteForceMatcher<L2<float>> matcher;
    BFMatcher matcher;
    vector<DMatch> matches;
    matcher.match(descriptor1, descriptor2, matches);
    //特征点排序
    sort(matches.begin(), matches.end());
    //获取排名前10个的匹配度高的匹配点集
    vector<KeyPoint> goodImagePoints1, goodImagePoints2;

    vector<DMatch> matchesVoted;

    for (int i = 0; i<30; i++)
    {
        DMatch dmatch;
        dmatch.queryIdx = i;
        dmatch.trainIdx = i;

        matchesVoted.push_back(dmatch);
        goodImagePoints1.push_back(keypoint1[matches[i].queryIdx]);
        goodImagePoints2.push_back(keypoint2[matches[i].trainIdx]);
    }

    Mat img_matches;
    std::vector< DMatch > emptyVec;
    drawMatches(input1, goodImagePoints1, input2, goodImagePoints2, matchesVoted, img_matches, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    imshow("SIFT_Match_Image", img_matches);
    waitKey();
    return 0;
}
