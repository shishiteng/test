#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;

#if (defined __GNUC__) && ( defined(__i386__) || defined(__x86_64__) )
  #define CUSTOM_ASM_COMMENT(X) __asm__("#" X)
#elif (defined __GNUC__) && (defined __arm__)
  #define CUSTOM_ASM_COMMENT(X) __asm__("@" X)
#else
  #define CUSTOM_ASM_COMMENT(X)
#endif

double shrDeltaT()
{
  double DeltaT;

  static struct timeval _NewTime;
  static struct timeval _OldTime0;
  gettimeofday(&_NewTime, NULL);

  DeltaT =  ((double)_NewTime.tv_sec + 1.0e-6 * (double)_NewTime.tv_usec) - ((double)_OldTime0.tv_sec + 1.0e-6 * (double)_OldTime0.tv_usec);

  // Reset old time 0 to new
  _OldTime0.tv_sec = _NewTime.tv_sec;
  _OldTime0.tv_usec = _NewTime.tv_usec;

  return DeltaT;
}


void printFloat32x4(float32x4_t data)
{
  float a = vgetq_lane_f32(data,0);
  float b = vgetq_lane_f32(data,1);
  float c = vgetq_lane_f32(data,2);
  float d = vgetq_lane_f32(data,3);
  printf(" %3f,%3f,%3f,%3f\n",a,b,c,d);
}


/* C=A*B
 * A:m x n
 * B:n x p
 * C:m x p
*/
void multiply_reference(float *C,float *A,float *B,int m,int n,int p)
{
  for(int i=0;i<m;i++)
    //C[m][p] = sum(A[row:m]]*b[col:n])
    for(int k=0;k<p;k++) {
      float sum = 0;
      //cout<<""<<i<<","<<k<<endl;
      for(int j=0;j<n;j++) {
	//A[i][j]*B[j][k]
	sum += A[i*n+j] * B[j*p+k];
	//cout<<"  "<<i<<","<<j<<"  "<<j<<","<<k<<" A:"<<i*n+j<<" B:"<<j*p+k<<"    sum:"<<sum<<endl;
      }
      
      C[i*p+k] = sum;
    }
}


// 4x4 
void altneonmult(const float *matrixA, const float *matrixB, float *matrixR)
{
  float32x4_t a0, a1, a2, a3, b, r;

  a0 = vld1q_f32(matrixA);/* col 0 of matrixA */
  a1 = vld1q_f32(matrixA +4); /* col 1 of matrixA */
  a2 = vld1q_f32(matrixA +8); /* col 2 of matrixA */
  a3 = vld1q_f32(matrixA +12); /* col 3 of matrixA */

  b = vld1q_f32(matrixB);
  /* load col 0 of matrixB */
  r = vmulq_lane_f32(a0, vget_low_f32(b), 0);
  r = vmlaq_lane_f32(r, a1, vget_low_f32(b), 1);
  r = vmlaq_lane_f32(r, a2, vget_high_f32(b), 0);
  r = vmlaq_lane_f32(r, a3, vget_high_f32(b), 1);
  vst1q_f32(matrixR, r);

  /* store col 0 of result */
  b = vld1q_f32(matrixB + 4); /* load col 1 of matrixB */
  r = vmulq_lane_f32(a0, vget_low_f32(b), 0);
  r = vmlaq_lane_f32(r, a1, vget_low_f32(b), 1);
  r = vmlaq_lane_f32(r, a2, vget_high_f32(b), 0);
  r = vmlaq_lane_f32(r, a3, vget_high_f32(b), 1);
  vst1q_f32(matrixR + 4, r);

  /* store col 1 of result */
  b = vld1q_f32(matrixB + 8); /* load col 2 of matrixB */
  r = vmulq_lane_f32(a0, vget_low_f32(b), 0);
  r = vmlaq_lane_f32(r, a1, vget_low_f32(b), 1);
  r = vmlaq_lane_f32(r, a2, vget_high_f32(b), 0);
  r = vmlaq_lane_f32(r, a3, vget_high_f32(b), 1);
  vst1q_f32(matrixR + 8, r);
  /* store col 2 of result */

  b = vld1q_f32(matrixB + 12); /* load col 3 of matrixB */
  r = vmulq_lane_f32(a0, vget_low_f32(b), 0);
  r = vmlaq_lane_f32(r, a1, vget_low_f32(b), 1);
  r = vmlaq_lane_f32(r, a2, vget_high_f32(b), 0);
  r = vmlaq_lane_f32(r, a3, vget_high_f32(b), 1);
  vst1q_f32(matrixR + 12, r);
  /* store col 3 of result */
}


//4x4
void neonmult(const float *matrixA, const float *matrixB, float *matrixR)
{
  float32x4_t a0, a1, a2, a3, b0, b1, b2, b3, r0, r1, r2, r3;

  //
  a0 = vld1q_f32(matrixA);
  a1 = vld1q_f32(matrixA + 4);
  a2 = vld1q_f32(matrixA + 8);
  a3 = vld1q_f32(matrixA + 12);
  
  b0 = vld1q_f32(matrixB);
  b1 = vld1q_f32(matrixB + 4);
  b2 = vld1q_f32(matrixB + 8);
  b3 = vld1q_f32(matrixB + 12);

  r0 = vmulq_lane_f32(a0, vget_low_f32(b0), 0);
  r0 = vmlaq_lane_f32(r0, a1, vget_low_f32(b0), 1);
  r0 = vmlaq_lane_f32(r0, a2, vget_high_f32(b0), 0);
  r0 = vmlaq_lane_f32(r0, a3, vget_high_f32(b0), 1);

  r1 = vmulq_lane_f32(a0, vget_low_f32(b1), 0);
  r1 = vmlaq_lane_f32(r1, a1, vget_low_f32(b1), 1);
  r1 = vmlaq_lane_f32(r1, a2, vget_high_f32(b1), 0);
  r1 = vmlaq_lane_f32(r1, a3, vget_high_f32(b1), 1);

  r2 = vmulq_lane_f32(a0, vget_low_f32(b2), 0);
  r2 = vmlaq_lane_f32(r2, a1, vget_low_f32(b2), 1);
  r2 = vmlaq_lane_f32(r2, a2, vget_high_f32(b2), 0);
  r2 = vmlaq_lane_f32(r2, a3, vget_high_f32(b2), 1);

  r3 = vmulq_lane_f32(a0, vget_low_f32(b3), 0);
  r3 = vmlaq_lane_f32(r3, a1, vget_low_f32(b3), 1);
  r3 = vmlaq_lane_f32(r3, a2, vget_high_f32(b3), 0);
  r3 = vmlaq_lane_f32(r3, a3, vget_high_f32(b3), 1);

  vst1q_f32(matrixR, r0);
  vst1q_f32(matrixR + 4, r1);
  vst1q_f32(matrixR + 8, r2);
  vst1q_f32(matrixR + 12, r3);
}

void FillArray(float* pfData, int iSize,int seed)
{
  srand(seed);
  int i;
  for (i = 0; i < iSize; ++i) {
    int x = -10,y=10;
    int k=x+rand()%(y-x+1);
    float d = (float)k/10.f;
    pfData[i] = d;
  }
}

#define M 4
#define N 4
#define P 4

int main(int argc ,char **argv)
{
  double delta = 0;
  int i = 1;
  if(argc == 2)
    i = atoi(argv[1]);
  int n = i;

  float* A = (float*) malloc(sizeof(float)*M*N);
  float* B = (float*) malloc(sizeof(float)*N*P);
  float* C = (float*) malloc(sizeof(float)*M*P);

  srand(time(NULL));//2006);
  FillArray(A, M*N, 1);
  FillArray(B, N*P, 10);
  FillArray(C, M*P, -1);

  Mat matA(M,N,CV_32FC1,A);
  Mat matB(N,P,CV_32FC1,B);
  cout<<"A:"<<endl<<matA<<endl;
  cout<<"B:"<<endl<<matB<<endl;
  cout<<"At x Bt:"<<endl<<matA.t()*matB.t()<<endl;

  shrDeltaT();
  while(i-- > 0)
    multiply_reference(C,A,B,M,N,P);
  delta = shrDeltaT();

  cout<<"c time:"<<delta*1000<<endl;
  cout<<"average:"<<delta*1000/n<<"ms\n\n";

  Mat matC(M,P,CV_32FC1,C);
  //cout<<"matC:"<<endl<<matC<<endl<<endl;;

  i = n;
  shrDeltaT();
  while(i-- > 0)
    neonmult(A,B,C);
  delta = shrDeltaT();

  cout<<"neonmult time:"<<delta*1000<<endl;
  cout<<"average:"<<delta*1000/n<<"ms\n";
  Mat matNeon(M,P,CV_32FC1,C);
  cout<<"matNeonmul:"<<endl<<matNeon<<endl;
 
  i = n;
  shrDeltaT();
  while(i-- > 0)
    altneonmult(A,B,C);
  delta = shrDeltaT();

  cout<<"altneonmult time:"<<delta*1000<<endl;
  cout<<"average:"<<delta*1000/n<<"ms\n\n";
  Mat matNeonalt(M,P,CV_32FC1,C);
  cout<<"matAltneonmul:"<<endl<<matNeonalt<<endl;


  //cout<<"======="<<endl<<matA*matB<<endl;

  return 0;
}
