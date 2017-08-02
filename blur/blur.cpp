#include <stdlib.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    if (argc<3) {
        cout<<"Usage: ./blur [file name] [kernel size]"<<endl;
        return -1;
    }

    Mat I = imread(argv[1]);//, CV_LOAD_IMAGE_GRAYSCALE);
    imshow("orig", I);

    Mat dst;
    int i = atoi(argv[2]);

    /* Homogeneous blur,均值滤波 */
    blur(I, dst, Size(i, i), Point(-1,-1));
    imshow("homogeneous", dst);

    /* Guassian blur，高斯滤波 */
    GaussianBlur(I, dst, Size(i, i), 0, 0);
    imshow("gassian", dst);


    /* Median blur，中值滤波 */
    medianBlur(I, dst, i);
    imshow("media", dst);
    
    /* Bilatrial blur，双边滤波 */
    bilateralFilter(I, dst, i, i*2, i/2);
    imshow("bilatrial", dst);

    waitKey(0);

    return 0;
}
