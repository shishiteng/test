#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

double shrDeltaT()
{
  double DeltaT;

  static struct timeval _NewTime;  // new wall clock time (struct representation in seconds and microseconds)
  static struct timeval _OldTime0; // old wall clock time 0(struct representation in seconds and microseconds)
  gettimeofday(&_NewTime, NULL);

  DeltaT =  ((double)_NewTime.tv_sec + 1.0e-6 * (double)_NewTime.tv_usec) - ((double)_OldTime0.tv_sec + 1.0e-6 * (double)_OldTime0.tv_usec);

  // Reset old time 0 to new
  _OldTime0.tv_sec = _NewTime.tv_sec;
  _OldTime0.tv_usec = _NewTime.tv_usec;

  return DeltaT;
}


void reference_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)
{
  int i;
  for (i=0; i<n; i++)
    {
      int b = *src++; // load red
      int g = *src++; // load green
      int r = *src++; // load blue 

      // build weighted average:
      int y = (r*77)+(g*151)+(b*28);

      // undo the scale by 256 and write to memory:
      *dest++ = (y>>8);
    }
}

#if 0
void neon_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)
{
  int i;
  uint8x8_t rfac = vdup_n_u8 (77);
  uint8x8_t gfac = vdup_n_u8 (151);
  uint8x8_t bfac = vdup_n_u8 (28);
  n/=8;

  for (i=0; i<n; i++) {
    uint16x8_t  temp;
    uint8x8x3_t bgr  = vld3_u8 (src);
    uint8x8_t result;

    temp = vmull_u8 (bgr.val[0],      bfac);
    temp = vmlal_u8 (temp,bgr.val[1], gfac);
    temp = vmlal_u8 (temp,bgr.val[2], rfac);

    result = vshrn_n_u16 (temp, 8);
    vst1_u8 (dest, result);
    src  += 8*3;
    dest += 8;
  }
}
#endif

int main(int argc,char *argv[])
{
  double delta;
  if(argc != 2){
    printf("para error:\n    neon_convert [src]\n");
    return -1;
  }
  
  Mat src = imread(argv[1]);
  if(src.empty()){
    printf("open image:%s failed.\n",argv[1]);
    return -1;
  }

  Mat dest;

  shrDeltaT();
  cvtColor(src, dest, CV_BGR2GRAY);
  delta = shrDeltaT();
  printf("c time: %fms\n", delta*1000);

  imshow("gray",dest);
  waitKey(0);

  return 0;

}
