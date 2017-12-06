#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    IplImage*img = cvLoadImage(argv[1]);
    IplImage*t_img = cvLoadImage(argv[2]);

    IplImage*src_gray1 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);    
    cvCvtColor(img, src_gray1, CV_RGB2GRAY);

    IplImage*t_gray1 = cvCreateImage(cvGetSize(t_img), IPL_DEPTH_8U, 1);
    cvCvtColor(t_img, t_gray1, CV_RGB2GRAY);

    unsigned int src_width = img->width;
    unsigned int src_height = img->height;
    unsigned int t_width = t_img->width;
    unsigned int t_height = t_img->height;
    int maxwidth = 0;
    int maxheight = 0;

    unsigned long InterRelateValue = 0, S_energy = 0, T_energy = 0;
    double matchvalue = 0, maxvalue = 0;

    IplImage*s_img = cvCreateImage(cvGetSize(t_img), IPL_DEPTH_8U, 1);
    for (int i = 0; i < (src_height-t_height); i++)
    {
        for (int j = 0; j < (src_width - t_width); j++)
        {
            cvSetImageROI(src_gray1, cvRect(j, i, t_width, t_height));
            cvCopy(src_gray1, s_img, 0);
            cvResetImageROI(src_gray1);
            for (int m = 0; m < t_height; m++)
            {
                unsigned char*s_ptr = (unsigned char*)s_img->imageData + m*s_img->widthStep;
                unsigned char*t_ptr = (unsigned char*)t_gray1->imageData + m*t_gray1->widthStep;
                for (int n = 0; n < t_width; n++)
                {
                    unsigned char s_value = s_ptr[n];
                    unsigned char t_value = t_ptr[n];
                    InterRelateValue = InterRelateValue + s_value*t_value;
                    S_energy = S_energy + s_value*s_value;
                    T_energy = T_energy + t_value*t_value;
                }
            }
            matchvalue = ((double)InterRelateValue) / (sqrt((double)S_energy)*sqrt((double)T_energy));
            if (matchvalue > maxvalue)
            {
                maxvalue = matchvalue;
                maxwidth = j;
                maxheight = i;

            }
            InterRelateValue =0;
            S_energy = 0;
            T_energy = 0;
        }
    }
    IplImage*Match_image = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
    cvSet2D(Match_image, maxheight, maxwidth, cvScalar(255, 0, 0, 0));
    cvRectangle(img, cvPoint(maxwidth, maxheight), cvPoint(maxwidth + t_width, maxheight + t_height), cvScalar(0, 0, 255, 0), 1, 8, 0);

    cvNamedWindow("img");
    cvShowImage("img", img);
    cvNamedWindow("t_img");
    cvShowImage("t_img", t_img);
    cvNamedWindow("Match_image");
    cvShowImage("Match_image", Match_image);
    waitKey();

}
