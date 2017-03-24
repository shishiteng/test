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
  for(unsigned int i=0;i<m;i++)
    //C[m][p] = sum(A[row:m]]*b[col:n])
    for(unsigned int k=0;k<p;k++) {
      float sum = 0;
      //cout<<""<<i<<","<<k<<endl;
      for(unsigned int j=0;j<n;j++) {
	//A[i][j]*B[j][k]
	float a = A[i*n+j];
	float b = B[j*p+k];
	sum += a*b;
	//cout<<"  "<<i<<","<<j<<"  "<<j<<","<<k<<" A:"<<i*n+j<<" B:"<<j*p+k<<"    sum:"<<sum<<endl;
      }
      
      C[i*p+k] = sum;
    }
}

/* C=A*B
 * A:m x n
 * B:n x p
 * C:m x p
 * C:0.08ms
 * openCV:0.05ms
 * eigen:0.051ms 
 * this fuction:0.024ms
*/
void multiply_neon(float *C,float *A,float *B,int m,int n,int p)
{
  for(int i=0;i<m;i++)
    for(int k=0;k<p;k++) {
      //volatile float sum = 0;
      float sum = 0;
      float32x4_t fSum = {0,0,0,0};
      //memset(&fSum,0,sizeof(fSum));
      //cout<<""<<i<<","<<k<<endl;
      for(int j=0;j<n;j+=4) {
	//A[i][j]*B[j][k]
	float *srcA = A + i*n+j;
	float *srcB = B + j*p+k;
	float32_t *ptr = srcB;

	//load 4 row elements of A
	float32x4_t fA;
	float32x4_t fB;
	float32x4_t fDot;
	CUSTOM_ASM_COMMENT("load A");
	//load 4 elements from srcA
	fA = vld1q_f32(srcA);

	//load 4 col elements of B
	//1. fB:{srcB[0],srcB[1],srcB[2],srcB[3]}
	//2. fB:{ ptr[0],srcB[1],srcB[2],srcB[3]}
	fB = vld1q_f32(srcB);
	//printFloat32x4(fB);
	CUSTOM_ASM_COMMENT("load B");
	fB = vld1q_lane_f32(ptr,fB,0);
	fB = vld1q_lane_f32(ptr+p,fB,1);
	fB = vld1q_lane_f32(ptr+p*2,fB,2);
	fB = vld1q_lane_f32(ptr+p*3,fB,3);

	//cout<<" fA:";
	//printFloat32x4(fA);
	//cout<<" fB:";
	//printFloat32x4(fB);

	//vector dot
	CUSTOM_ASM_COMMENT("fDot");
	fDot = vmulq_f32(fA,fB);
	
	//vector sum
	CUSTOM_ASM_COMMENT("sum...");

	//cout<<" fDot:";
	//printFloat32x4(fDot);
	//cout<<" fSum:";
	//printFloat32x4(fSum);

	fSum = vaddq_f32(fSum,fDot);
	//cout<<" fSum2:";
	//printFloat32x4(fSum);
	CUSTOM_ASM_COMMENT("sum over.");
      }

      CUSTOM_ASM_COMMENT("======");
      sum = vgetq_lane_f32(fSum,0)
	  +vgetq_lane_f32(fSum,1)
	  +vgetq_lane_f32(fSum,2)
	  +vgetq_lane_f32(fSum,3);;
      C[i*p+k] = sum;
      CUSTOM_ASM_COMMENT("------");
    }
}


void FillEigenMatrix(Eigen::MatrixXf *m, float *data, int rows, int cols)
{
  for(int i=0;i<rows;i++) {
    for(int j=0;j<cols;j++) {
      (*m)(i,j) = data[i*cols+j];
    }
  }
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

#define M 2
#define N 96
#define P 96

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
  //cout<<"A:"<<endl<<matA<<endl;
  //cout<<"B:"<<endl<<matB<<endl;

  Eigen::MatrixXf eigenA = Eigen::MatrixXf::Random(M,N);
  Eigen::MatrixXf eigenB = Eigen::MatrixXf::Random(N,P);
  FillEigenMatrix(&eigenA,A,M,N);
  FillEigenMatrix(&eigenB,B,N,P);

  //C
  shrDeltaT();
  while(i-- > 0)
    multiply_reference(C,A,B,M,N,P);
  delta = shrDeltaT();

  cout<<"c time:"<<delta*1000<<endl;
  cout<<"average:"<<delta*1000/n<<"ms\n\n";
  Mat matC(M,P,CV_32FC1,C);
  //cout<<"matC:"<<endl<<matC<<endl<<endl;;


  Mat matOpenCV;
  i = n;
  shrDeltaT();
  while(i-- > 0)
    matOpenCV = matA*matB;
  delta = shrDeltaT();
  cout<<"opencv time:"<<delta*1000<<endl;
  cout<<"average:"<<delta*1000/n<<"ms\n\n";
  //cout<<"matOpenCV:"<<endl<<matOpenCV<<endl<<endl;;

  Eigen::MatrixXf matEigen;
  i = n;
  shrDeltaT();
  while(i-- > 0)
    matEigen = eigenA*eigenB;
  delta = shrDeltaT();
  cout<<"eigen time:"<<delta*1000<<endl;
  cout<<"average:"<<delta*1000/n<<"ms\n\n";
  //cout<<"matEigen:"<<endl<<matEigen<<endl<<endl;;

  i = n;
  shrDeltaT();
  while(i-- > 0)
    multiply_neon(C,A,B,M,N,P);
  delta = shrDeltaT();
  cout<<"neon time:"<<delta*1000<<endl;
  cout<<"average:"<<delta*1000/n<<"ms\n\n";
  Mat matNeon(M,P,CV_32FC1,C);
  //cout<<"matNeon:"<<endl<<matNeon<<endl;


  //cout<<"======="<<endl<<matA*matB<<endl;

  return 0;
}
