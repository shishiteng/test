
#ifndef FAST_CUDA_H_
#define FAST_CUDA_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include "corner.h"

__device__
extern int position(int m,int n,int width);

__global__
extern void fast(uchar* image, int width, int height,Corner* d_corner,int gridsize_x, int gridsize_y, const int threshold);

__global__
extern void nms(uchar* image, Corner* d_corner,int width, int height);



#endif /* FAST_CUDA_H_ */
