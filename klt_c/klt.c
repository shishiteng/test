#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "opencv2/opencv.hpp"//

/****************************headers********************************/

#define int8 char
#define int16 short
#define int32 int
#define int64 long int
#define uint8 unsigned char
#define uint16 unsigned short
#define uint32 unsigned int

#define MAX_LEVELS        4
#define MAX_FEATURES      256
#define MAX_ITERATE_COUNT 10
#define MAX_BUFFER_SIZE   409600       //

#define ROUND(x) ((int)(x+0.5))
#define FLOOR(x) ((int)(x))
#define DESCALE(x,b) ((x)>>(b))


typedef struct{
  uint16 width;
  uint16 height;
  uint8 *data;
}frame_t;

typedef struct{
  uint16 x;
  uint16 y;
  int16 id;
}feature_t;

typedef struct{
  frame_t pyramid[MAX_LEVELS];
  frame_t derive[MAX_LEVELS];
  uint16 num_levels;
}pyramid_t;

typedef struct{
  uint16 count;
  feature_t pt[MAX_FEATURES];
}features_t;

/****************************source********************************/
uint8* g_pyr_buffer0;
uint8* g_pyr_buffer1;
uint8* g_deriv_buffer;
uint8* g_buf = NULL;

int gassianblur(uint8 *src,uint8* dst,uint16 width,uint16 height,uint16 kernel_size)
{
  uint16 kernel[9] = {
    1, 2, 1,
    2, 4, 2,
    1, 2, 1
  };
    
  int i,j,k=0;
  uint8 data[9] = {0};
  //边界上的数据不做处理
  for(i=1;i<height-1;i++) {
    for(j=1;j<width-1;j++) {
      uint16 sum = 0;
      data[0] = src[(i-1)*width + j - 1];
      data[1] = src[(i-1)*width + j + 0];
      data[2] = src[(i-1)*width + j + 1];
      data[3] = src[(i+0)*width + j - 1];
      data[4] = src[(i+0)*width + j + 0];
      data[5] = src[(i+0)*width + j + 1];
      data[6] = src[(i+1)*width + j - 1];
      data[7] = src[(i+1)*width + j + 0];
      data[8] = src[(i+1)*width + j + 1];

      for(k=0;k<9;k++) {
	sum += kernel[k] * data[k];
      }

      //dst[i * width + j] = (uint8)(sum/16);
      dst[i * width + j] = (uint8)(sum>>4);
    }
  }
}


//问题：
// 1.只支持3x3
// 2.整型运算，精度稍有牺牲
int pyrdown(uint8 *src,uint8* dst,uint16 width,uint16 height,uint16 kernel_size,uint16 num_levels)
{
  uint16 kernel[9] = {
    1, 2, 1,
    2, 4, 2,
    1, 2, 1
  };


  //高斯模糊+下采样
  //边界上的数据不做处理

  //level: 0,1,2... num_levels

  uint8 *d = dst;
    
  uint32 i,j,k =0;
  uint8 data[9] = {0};
  for(uint16 l=0;l<num_levels;l++) {
    uint16 w = width>>l;
    uint16 h = height>>l;
    uint32 m = 0;

    //高斯卷积,同时偶数行采样
    for(i=1;i<h;i+=2) {
      for(j=1;j<w;j+=2) {
	uint16 sum = 0;
	data[0] = src[(i-1)*w + j - 1];
	data[1] = src[(i-1)*w + j + 0];
	data[2] = src[(i-1)*w + j + 1];
	data[3] = src[(i+0)*w + j - 1];
	data[4] = src[(i+0)*w + j + 0];
	data[5] = src[(i+0)*w + j + 1];
	data[6] = src[(i+1)*w + j - 1];
	data[7] = src[(i+1)*w + j + 0];
	data[8] = src[(i+1)*w + j + 1];

	for(k=0;k<9;k++) {
	  sum += kernel[k] * data[k];
	}

	//Todo:这里截位产生的误差，会不会对后续算法精度造成影响？
	dst[m++] = (uint8)(sum>>4);
      }
    }

#if 0
    cv::Mat image(h>>1,w>>1,CV_8UC1,dst);
    cv::imshow("1",image);
    cv::waitKey(0);
#endif
    
    src = dst;
    dst += m;
  }


}

int pyrdown_5x5(uint8 *src,uint8* dst,uint16 width,uint16 height,uint16 kernel_size,uint16 num_levels)
{
  uint16 kernel[25] = {
    1, 2, 1,
    2, 4, 2,
    1, 2, 1
  };


  //高斯模糊+下采样
  //边界上的数据不做处理

  //level: 0,1,2... num_levels

  uint8 *d = dst;
    
  uint32 i,j,k =0;
  uint8 data[9] = {0};
  for(uint16 l=0;l<num_levels;l++) {
    uint16 w = width>>l;
    uint16 h = height>>l;
    uint32 m = 0;

    //高斯卷积,同时偶数行采样
    for(i=1;i<h;i+=2) {
      for(j=1;j<w;j+=2) {
	uint16 sum = 0;
	data[0] = src[(i-1)*w + j - 1];
	data[1] = src[(i-1)*w + j + 0];
	data[2] = src[(i-1)*w + j + 1];
	data[3] = src[(i+0)*w + j - 1];
	data[4] = src[(i+0)*w + j + 0];
	data[5] = src[(i+0)*w + j + 1];
	data[6] = src[(i+1)*w + j - 1];
	data[7] = src[(i+1)*w + j + 0];
	data[8] = src[(i+1)*w + j + 1];

	for(k=0;k<9;k++) {
	  sum += kernel[k] * data[k];
	}

	//Todo:这里截位产生的误差，会不会对后续算法精度造成影响？
	dst[m++] = (uint8)(sum>>4);
      }
    }

#if 0
    cv::Mat image(h>>1,w>>1,CV_8UC1,dst);
    cv::imshow("1",image);
    cv::waitKey(0);
#endif
    
    src = dst;
    dst += m;
  }


}


//问题：
// 1.边缘未作处理
// 2.暂时是整型运算
// 3.梯度求出后有正负，这里把负数变成了0,应该是有问题的吧？
// 4.opencv做了去噪声，这个算法没做，是否要加上？如果要加的话应该拿高斯模糊后的图
int sobel(uint8 *src,uint8* dst,uint16 width,uint16 height,uint16 kernel_size)
{
  int16 kernel_x[9] = {
    -1, 0, 1,
    -2, 0, 2,
    -1, 0, 1
  };

  int16 kernel_y[9] = {
    -1, -2, -1,
     0,  0,  0,
     1,  2,  1
  };

  int16* pdst = (int16*)dst;
  memset(pdst,0,sizeof(int16)*width*height);
  
  int i,j,k=0;
  uint8 data[9] = {0};
  //边界上的数据暂时不做处理
  for(i=1;i<height-1;i++) {
    for(j=1;j<width-1;j++) {
      int16 sum_x = 0,sum_y = 0;
      data[0] = src[(i-1)*width + j - 1];
      data[1] = src[(i-1)*width + j + 0];
      data[2] = src[(i-1)*width + j + 1];
      data[3] = src[(i+0)*width + j - 1];
      data[4] = src[(i+0)*width + j + 0];
      data[5] = src[(i+0)*width + j + 1];
      data[6] = src[(i+1)*width + j - 1];
      data[7] = src[(i+1)*width + j + 0];
      data[8] = src[(i+1)*width + j + 1];

      for(k=0;k<9;k++) {
	sum_x += kernel_x[k] * data[k];
	sum_y += kernel_y[k] * data[k];
	//printf("[%d,%d]_____%d      (%d x %d)\n",j,i,sum_x, kernel_x[k], data[k]);
      }

#if 0
      //这里把负数取0了
      uint8 * p = (uint8*)&pdst[i * width + j];
      sum_x = sum_x < 0 ? 0: sum_x;
      sum_y = sum_y < 0 ? 0: sum_y;
      //printf("[%d,%d] %d %d\n",j,i,sum_x,sum_y);
      p[0] = sum_x>>3;
      p[1] = sum_y>>3;
#else
      //可以表示正负
      //双通道表示x、y方向梯度
      int8 * p = (int8*)&pdst[i * width + j];
      //p[0] = sum_x>>3;
      //p[1] = sum_y>>3;
      p[0] = sum_x>>3;
      p[1] = sum_y>>3;
      //printf("[%d,%d] %d %d\n",j,i,p[0],p[1]);
#endif
    }
  }
}

int sobel_5x5(uint8 *src,uint8* dst,uint16 width,uint16 height,uint16 kernel_size)
{
  int16 kernel_x[25] = {
    -1, 0, 1,
    -2, 0, 2,
    -1, 0, 1
  };

  int16 kernel_y[9] = {
    -1, -2, -1,
     0,  0,  0,
     1,  2,  1
  };

  int16* pdst = (int16*)dst;
  memset(pdst,0,sizeof(int16)*width*height);
  
  int i,j,k=0;
  uint8 data[9] = {0};
  //边界上的数据暂时不做处理
  for(i=1;i<height-1;i++) {
    for(j=1;j<width-1;j++) {
      int16 sum_x = 0,sum_y = 0;
      data[0] = src[(i-1)*width + j - 1];
      data[1] = src[(i-1)*width + j + 0];
      data[2] = src[(i-1)*width + j + 1];
      data[3] = src[(i+0)*width + j - 1];
      data[4] = src[(i+0)*width + j + 0];
      data[5] = src[(i+0)*width + j + 1];
      data[6] = src[(i+1)*width + j - 1];
      data[7] = src[(i+1)*width + j + 0];
      data[8] = src[(i+1)*width + j + 1];

      for(k=0;k<9;k++) {
	sum_x += kernel_x[k] * data[k];
	sum_y += kernel_y[k] * data[k];
	//printf("[%d,%d]_____%d      (%d x %d)\n",j,i,sum_x, kernel_x[k], data[k]);
      }

#if 0
      //这里把负数取0了
      uint8 * p = (uint8*)&pdst[i * width + j];
      sum_x = sum_x < 0 ? 0: sum_x;
      sum_y = sum_y < 0 ? 0: sum_y;
      //printf("[%d,%d] %d %d\n",j,i,sum_x,sum_y);
      p[0] = sum_x>>3;
      p[1] = sum_y>>3;
#else
      //可以表示正负
      //双通道表示x、y方向梯度
      int8 * p = (int8*)&pdst[i * width + j];
      p[0] = sum_x>>3;
      p[1] = sum_y>>3;
      //printf("[%d,%d] %d %d\n",j,i,p[0],p[1]);
#endif
    }
  }
}

int klt_init(pyramid_t *pyr0,pyramid_t *pyr1,uint16 width,uint16 height,uint16 num_levels)
{
  int s = width*height;
  g_pyr_buffer0 = (uint8*)malloc(s*sizeof(uint8)); //这里要特别注意，原图是不放在这个buffer里的
  g_pyr_buffer1 = (uint8*)malloc(s*sizeof(uint8));
  g_deriv_buffer = (uint8*)malloc(s*sizeof(int16)*2);
  
  uint8 *p_pyr0 = g_pyr_buffer0;
  uint8 *p_pyr1 = g_pyr_buffer1;
  uint8 *p_deriv = g_deriv_buffer;

  //printf("klt_init:g_pyr_buf0 0x%x \n",g_pyr_buffer0);
  //printf("klt_init:g_pyr_buf1 0x%x \n",g_pyr_buffer1);
  //printf("klt_init:g_dev_buf0 0x%x \n",g_deriv_buffer0);
  //printf("klt_init:g_dev_buf1 0x%x \n",g_deriv_buffer1);
  
  for(int i=0;i<num_levels;i++) {
    uint16 w = width>>i;
    uint16 h = height>>i;

    pyr0->pyramid[i].width = w;
    pyr0->pyramid[i].height = h;
    if(0 == i) {
      pyr0->pyramid[i].data = NULL;
    } else {
      pyr0->pyramid[i].data = p_pyr0;
      p_pyr0 += w*h;      
    }

    pyr1->pyramid[i].width = w;
    pyr1->pyramid[i].height = h;
    if(0 == i) {
      pyr1->pyramid[i].data = NULL;
    } else {
      pyr1->pyramid[i].data = p_pyr1;
      p_pyr1 += w*h;
    }

    pyr0->derive[i].width = w;
    pyr0->derive[i].height = h;
    pyr0->derive[i].data = p_deriv;
    p_deriv += w*h*sizeof(int16);

    //printf("klt_init:pyr0_level%d 0x%x \n",i,pyr0->pyramid[i].data);
    //printf("klt_init:pyr1_level%d 0x%x \n",i,pyr1->pyramid[i].data);
    //printf("klt_init:dev0_level%d 0x%x \n",i,pyr0->derive[i].data);
  }

  pyr0->num_levels = num_levels;
  pyr1->num_levels = num_levels;

  return 0;
}


int klt_buildpyramids(pyramid_t *pyr,uint8 *data)
{
  int width = pyr->pyramid[0].width;
  int height = pyr->pyramid[0].height;
  int num_levels = pyr->num_levels;
  //memcpy((void*)pyr->pyramid[0].data, (void*)data, width*height);
  pyr->pyramid[0].data = data;

  uint8 * p_pyr_buf = pyr->pyramid[1].data;
  uint16 kernel_size = 3;

  // pyrdown
  //fprintf(stderr,"pyrdown:0x%x\n",p_pyr_buf);
  pyrdown(data,p_pyr_buf,width,height,kernel_size,num_levels);

#if 0
  for(int i=0;i<pyr->num_levels;i++) {
    int w = pyr->pyramid[i].width;
    int h = pyr->pyramid[i].height;
    uint8 *p = pyr->pyramid[i].data;
    cv::Mat image(h,w,CV_8UC1,p);
    cv::imshow("pyramid",image);
    cv::waitKey(0);
  }
#endif
    
  pyr->num_levels = num_levels;
    
  return 0;
}

int klt_sobel(pyramid_t *pyr)
{
  // sobel
  for(int i=0;i<pyr->num_levels;i++) {
    uint16 w = pyr->pyramid[i].width;
    uint16 h = pyr->pyramid[i].height;
    sobel(pyr->pyramid[i].data,pyr->derive[i].data,w,h,3);
  }

#if 0
  //uint8 d[640*480];
  uint8* d = (uint8*)malloc(MAX_BUFFER_SIZE*2);
  memset(d,0,MAX_BUFFER_SIZE*2);
  for(int i=0;i<pyr->num_levels;i++) {
    int w = pyr->derive[i].width;
    int h = pyr->derive[i].height;
    int16 *p16 = (int16*)pyr->derive[i].data;
    //fprintf(stderr,"level_%d 0x%x\n",i,p16);
    for(int j=0;j<w;j++) {
      for(int k=0;k<h;k++) {
	int8 *p = (int8*)&p16[k*w+j];
	int8 dx = p[0];
	int8 dy = p[1];
	dx = dx > 0 ? dx : 0;
	dy = dy > 0 ? dy : 0;
	d[k*w+j] = (uint8)dy;
	//printf("[%d,%d] %d %d | %d %d\n",j,k,p[0],p[1],dx,dy);
      }
    }
    cv::Mat image(h,w,CV_8UC1,d);
    cv::imshow("deriv",image);
    cv::waitKey(0);
  }
  free(d);
#endif
    
  return 0;
}

int klt_check(pyramid_t *prev_pyr,pyramid_t *next_pyr,features_t *prev_pts,features_t *next_pts)
{
  // 验证输入
  //uint8* p = prev_pyr->pyramid[2].data;
  for(int i=0;i<prev_pyr->num_levels;i++) {
    int w = prev_pyr->pyramid[i].width;
    int h = prev_pyr->pyramid[i].height;
    uint8 *p = prev_pyr->pyramid[i].data;
    fprintf(stderr,"level_%d:%dx%d 0x%x\n",i,w,h,p);
    cv::Mat image(h,w,CV_8UC1,p);
    cv::imshow("0_pyr",image);
    cv::waitKey(0);
  }

  for(int i=0;i<next_pyr->num_levels;i++) {
    int w = next_pyr->pyramid[i].width;
    int h = next_pyr->pyramid[i].height;
    uint8 *p = next_pyr->pyramid[i].data;
    fprintf(stderr,"level_%d:%dx%d 0x%x\n",i,w,h,p);
    cv::Mat image(h,w,CV_8UC1,p);
    cv::imshow("1_pyr",image);
    cv::waitKey(0);
  }

  for(int i=0;i<prev_pyr->num_levels;i++) {
    int w = prev_pyr->derive[i].width;
    int h = prev_pyr->derive[i].height;

    uint8 d[MAX_BUFFER_SIZE*2];
    uint8 d1[MAX_BUFFER_SIZE*2];
    int16 *p16 = (int16*)prev_pyr->derive[i].data;
    for(int j=0;j<w;j++) {
      for(int k=0;k<h;k++) {
	int8 *p = (int8*)&p16[k*w+j];
	int8 dx = p[0];
	int8 dy = p[1];
	dx = dx > 0 ? dx : 0;
	dy = dy > 0 ? dy : 0;
	d[k*w+j] = dx;
	d1[k*w+j] =dy;
      }
    }
    cv::Mat imageX(h,w,CV_8UC1,d);
    cv::Mat imageY(h,w,CV_8UC1,d1);
    cv::imshow("0_deriv_x",imageX);
    cv::imshow("0_deriv_y",imageY);
    cv::waitKey(0);
  }

  uint8 *p = prev_pyr->pyramid[0].data;
  cv::Mat image(prev_pyr->pyramid[0].height,prev_pyr->pyramid[0].width,CV_8UC1,p);
  cv::Mat result;
  cv::cvtColor(image,result,cv::COLOR_GRAY2RGB);
  for(int i=0;i<prev_pts->count;i++) {
    cv::Point2f pt;
    pt.x = prev_pts->pt[i].x;
    pt.y = prev_pts->pt[i].y;
    cv::circle(result, pt, 2,cv::Scalar(255 , 0, 0), 2);
  }
  cv::imshow("features",result);
  cv::waitKey(0);
    
  return 0;
}

int klt_run(pyramid_t *prev_pyr,pyramid_t *next_pyr,features_t *prev_pts,features_t *next_pts)
{

  float FLT_SCALE=0.001f;
  int32 W_BITS=14; 
  int32 cn2 = 2;
  uint16 win_size = 21;
  uint16 half_win = win_size>>1;

  for(int32 m=0;m<prev_pts->count;m++) {

    // 目标点的初始值是它在前一帧最高层的位置
    float x = (float)prev_pts->pt[m].x/(1<<(prev_pyr->num_levels-1));
    float y = (float)prev_pts->pt[m].y/(1<<(prev_pyr->num_levels-1));

    // 多层追踪
    for(int32 l=(prev_pyr->num_levels-1);l>=0;l--) {
      // 点在A图像l层中的位置，不应该被改变
      const float xa = (float)prev_pts->pt[m].x/(1<<l);
      const float ya = (float)prev_pts->pt[m].y/(1<<l);

      win_size = 21;
      half_win = win_size>>1;
      
      // 1.同层追踪
      // 数据准备
      uint8 *image0 = prev_pyr->pyramid[l].data;
      uint8 *image1 = next_pyr->pyramid[l].data;
      int16 *image0_deriv = (int16*)prev_pyr->derive[l].data;
      uint16 width = prev_pyr->pyramid[l].width;
      uint16 height = prev_pyr->pyramid[l].height;
      int dstep = width*2; //梯度的step
      int step = width;    //灰度的step

      // 获取坐标的小数部分(A中的)，用于双线性插值
      float a = xa - int(xa);
      float b = ya - int(ya);

      //printf("level:%d\n",l);
      //printf("  level_%d point0: [%.1f, %.1f]\n",l,xa,ya);
      //printf("  level_%d point1: [%.1f, %.1f]\n",l,x,y);
      //printf("  float of pt:%.3f, %.3f\n",a,b);
      //printf("  size: %dx%d\n",width,height);
      //printf("  image0: 0x%x\n",image0);
      //printf("  image1: 0x%x\n",image1);
      //printf("  derive0: 0x%x\n",image0_deriv);

      // 为了在循环内部使用整型运算，双线性插值系数转为整型
      int iw00 = ROUND((1.f - a)*(1.f - b)*(1 << W_BITS));
      int iw01 = ROUND(a*(1.f - b)*(1 << W_BITS));
      int iw10 = ROUND((1.f - a)*b*(1 << W_BITS));
      int iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;

      float iA11=0,iA12=0,iA22=0; //整型的梯度
      float A11,A12,A22;        //浮点型梯度

      int Iptr[21*21]={0};      //用于保存窗口中的灰度
      int dIptr[21*21*2]={0};   //用于保存窗口中的梯度

      //fprintf(stderr," -----1\n",a,b);

      //如果越界，窗口缩小
      if(xa<half_win || ya<half_win || xa>=(width-1-half_win) || ya>=(height-1-half_win)) {
	win_size = win_size/2 + 1;
	half_win = win_size>>1;
	if(xa<half_win || ya<half_win || xa>=(width-1-half_win) || ya>=(height-1-half_win)){
	  win_size = win_size/2 + 1;;
	  half_win = win_size>>1;
	}
      }
      
      // 1.1 计算A的梯度矩阵
      for(int16 i=0;i<win_size;i++) {
	for(int16 j=0;j<win_size;j++) {
	  //i:x方向 j:y方向
	  int16 x_ = (int16)(xa - half_win + i);
	  int16 y_ = (int16)(ya - half_win + j);
	  int ival,ixval,iyval;
	  
	  //越界的，填充0
	  if(x_<0 || y_<0 || x_>=(width-1) || y_>=(height-1)) {
	    //printf("out of bound[%.0f,%.0f] %d,%d\n",xa,ya,x_,y_);
	    break;
	  } else {
	    //提供亚像素的精度
	    uint8* src = (uint8*)&image0[y_*width+x_];
	    int8* dsrc = (int8*)&image0_deriv[y_*width+x_];
	
	    ival = DESCALE(src[0]*iw00 +
			       src[1]*iw01 +
			       src[step]*iw10 +
			       src[step+1]*iw11,
			       W_BITS);
	    ixval = DESCALE(dsrc[0]*iw00 +
				dsrc[cn2]*iw01 +
				dsrc[dstep]*iw10 +
				dsrc[dstep+cn2]*iw11,
				W_BITS);
	    iyval = DESCALE(dsrc[1]*iw00 +
				dsrc[cn2+1]*iw01 +
				dsrc[dstep+1]*iw10 +
				dsrc[dstep+cn2+1]*iw11,
				W_BITS);
	  }
	  Iptr[j*win_size+i] = ival;
	  dIptr[j*win_size*2+i*2]   = ixval;
	  dIptr[j*win_size*2+i*2+1] = iyval;
	
	  iA11 += ixval*ixval;
	  iA12 += ixval*iyval;
	  iA22 += iyval*iyval;

	  //printf(" pt(%d,%d)__derive[%d,%d]\n",x_,y_,dsrc[0],dsrc[1]);
	  //printf(" pt(%d,%d)__derive[%d,%d]\n",x_,y_,dsrc[0],dsrc[1]);
	}
      }

      //fprintf(stderr," -----2\n",a,b);
    
      A11 = iA11*FLT_SCALE;
      A12 = iA12*FLT_SCALE;
      A22 = iA22*FLT_SCALE;
      float D = A11*A22 - A12*A12;
      D = 1./D;
      
      //printf("  G:%f, %f, %f\n",iA11,iA22,iA12);
      //printf("  G_scale:%.3f, %.3f, %.3f\n",A11,A22,A12);
      //printf("  D:%f\n",D);

      
      // 1.2 迭代计算与B的光流矢量
      // 点在图像B的l层中的位置
      float xb = x; 
      float yb = y;
      bool flag = false; //是否追踪到
      for(int32 iter=0; iter<MAX_ITERATE_COUNT; iter++) {
	//判断是否越界

	//双线性插值
	a = xb - int(xb);
	b = yb - int(yb);
	iw00 = ROUND((1.f - a)*(1.f - b)*(1 << W_BITS));
	iw01 = ROUND(a*(1.f - b)*(1 << W_BITS));
	iw10 = ROUND((1.f - a)*b*(1 << W_BITS));
	iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;
      
	int ib1=0,ib2=0;
	float b1, b2;

	//计算A和B的參差向量b
	for(int i=0;i<win_size;i++) {
	  for(int j=0;j<win_size;j++) {
	    int16 x_ = (int16)(xb - half_win + i);
	    int16 y_ = (int16)(yb - half_win + j);
	  
	    uint8* src = (uint8*)&image1[y_*width+x_];
	    int jval = DESCALE(src[0]*iw00 +
			       src[1]*iw01 +
			       src[step]*iw10 +
			       src[step+1]*iw11,
			       W_BITS);
	    int ival = Iptr[j*win_size+i];
	    int ixval = dIptr[j*win_size*2+i*2];
	    int iyval = dIptr[j*win_size*2+i*2+1];
	    int diff = jval - ival;
	    ib1 += diff*ixval;
	    ib2 += diff*iyval;
	    //printf("pt(%d,%d)_win[%d,%d]___diff:%d (%d,%d)\n",x_,y_,i,j,diff,jval,ival);
	  }
	}

	b1 = ib1*FLT_SCALE;
	b2 = ib2*FLT_SCALE;

	//printf("  __iter%d\n",iter);
	//printf("    b:%d, %d\n",ib1,ib2);
	//printf("    b_scale:%.3f, %.3f\n",b1,b2);

	float delta_x = (float)((A12*b2 - A22*b1) * D);
	float delta_y = (float)((A12*b1 - A11*b2) * D);
	//printf("    delta=%f, %f\n",delta_x,delta_y);

	// 迭代中止条件之一
	if((fabs(delta_x)+fabs(delta_y))< 0.1) {
	  //printf("  iter count:%d\n",iter);
	  flag = true;
	  break;
	}
      
	// 移动点坐标
	xb += delta_x;
	yb += delta_y;
	//printf("    new point[%.3f,%.3f] --->[%.1f,%.1f]\n",xb,yb,xb*(1<<l),yb*(1<<l));
      }//loop:iterator

      //跟踪成功，结果传递到下一层;
      //跟踪失败，置状态

      if(!flag) {
	next_pts->pt[m].id = -1;
	break;
      }

      float scale = (0==l ? 1.f : 2.f);
      x = xb * scale;
      y = yb * scale;
      next_pts->pt[m].id = 1;
      next_pts->pt[m].x = ROUND(x);
      next_pts->pt[m].y = ROUND(y);
      
      //printf("  point in next level: [%.1f, %.1f]\n",x,y);
    }//loop:levels
    
      
  }//loop:features

  return 0;
}


int calc_klt(uint8* prev_img,
	     uint8* next_img,
	     features_t* prev_features,
	     features_t* next_features,
	     uint16 width,
	     uint16 height,
	     uint16 win_size,
	     uint16 num_levels)
{
  //fprintf(stderr,"1\n");
  
  // 1.初始化金字塔
  pyramid_t prev_pyr, next_pyr;
  klt_init(&prev_pyr,&next_pyr,width,height,num_levels);
   
  // 2.建立高斯金字塔,其实每次可以之算一帧，另一帧缓存
  klt_buildpyramids(&prev_pyr,prev_img);
  klt_buildpyramids(&next_pyr,next_img);

  // 3.计算sobel
  klt_sobel(&prev_pyr);

  // 测试计算结果
  //klt_check(&prev_pyr, &next_pyr, prev_features, next_features);
  
  // 4.klt
  klt_run(&prev_pyr, &next_pyr, prev_features, next_features);
  
#if 0
  // 4.print output
  for(int i=0;i<next_features->count;i++) {
    printf("%d___%d,%d\n",i,next_features->pt[i].x,next_features->pt[i].y);
  }
#endif


  free(g_pyr_buffer0);
  free(g_pyr_buffer1);
  free(g_deriv_buffer);
    
  return 0;
}


