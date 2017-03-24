#include<stdio.h>
#include<math.h>
#include <sys/time.h>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

#define N 4

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

//二阶矩阵相乘
void Matrix_Multiply(float A[][N], float B[][N], float C[][N]) { 
  for(int i = 0; i < 2; i++) {  
    for(int j = 0; j < 2; j++) {  
      C[i][j] = 0;		
      for(int t = 0; t < 2; t++) {  
	C[i][j] = C[i][j] + A[i][t] * B[t][j];		  
      }	
    }		  
  }  
}  

//矩阵加法：
void add(int n, float A[][N], float B[][N], float R[][N])
{ 
  int i, j;
  for(i = 0; i < n; i++)
    for(j = 0; j < n; j++)
      R[i][j] = A[i][j] + B[i][j];
}

//矩阵减法：
void sub(int n, float A[][N], float B[][N], float R[][N])
{ 
  int i,j;
  for(i = 0; i < n; i++)
    for(j = 0; j < n; j++)
      R[i][j] = A[i][j] - B[i][j];
}
void strassen(int n, float A[][N], float B[][N], float C[][N])
{
  int i, j;
  float A11[N][N], A12[N][N], A21[N][N], A22[N][N];
  float B11[N][N], B12[N][N], B21[N][N], B22[N][N];
  float C11[N][N], C12[N][N], C21[N][N], C22[N][N];
  float AA[N][N], BB[N][N];
  float M1[N][N], M2[N][N], M3[N][N], M4[N][N], M5[N][N], M6[N][N], M7[N][N];
  if(n == 2) {
    Matrix_Multiply(A, B, C);
  } else {
    for(i = 0; i < n / 2; i++) {
      for(j = 0; j < n / 2; j++) {
	A11[i][j] = A[i][j];
	A12[i][j] = A[i][j + n / 2];
	A21[i][j] = A[i + n / 2][j];
	A22[i][j] = A[i + n / 2][j + n / 2];

	B11[i][j] = B[i][j];
	B12[i][j] = B[i][j + n / 2];
	B21[i][j] = B[i + n /2][j];
	B22[i][j] = B[i + n /2][j + n / 2];
      }
    }

    sub(n / 2, B12, B22, BB);
    strassen(n / 2, A11, BB, M1);

    add(n / 2, A11, A12, AA);
    strassen(n / 2, AA, B22, M2);

    add(n / 2, A21, A22, AA);
    strassen(n / 2, AA, B11, M3);

    sub(n / 2, B21, B11, BB);
    strassen(n / 2, A22, BB, M4);

    add(n / 2, A11, A22, AA);
    add(n / 2, B11, B22, BB);
    strassen(n / 2, AA, BB, M5);

    sub(n / 2, A12, A22, AA);
    add(n / 2, B21, B22, BB);
    strassen(n / 2, AA, BB, M6);

    sub(n / 2, A11, A21, AA);
    add(n / 2, B11, B12, BB);
    strassen(n / 2, AA, BB, M7);

    //C11 = M5 + M4 - M2 + M6
    add(n / 2, M5, M4, AA);
    sub(n / 2, M6, M2, BB);
    add(n / 2, AA, BB, C11);

    //C12 = M1 + M2
    add(n / 2, M1, M2, C12);

    //C21 = M3 + M4
    add(n / 2, M3, M4, C21);

    //C22 = M5 + M1 - M3 - M7
    sub(n / 2, M5, M3, AA);
    sub(n / 2, M1, M7, BB);
    add(n / 2, AA, BB, C22);

    for(i = 0; i < n / 2; i++) {  
      for(j = 0; j < n / 2; j++) {  
	C[i][j] = C11[i][j];  
	C[i][j + n / 2] = C12[i][j];  
	C[i + n / 2][j] = C21[i][j];  
	C[i + n / 2][j + n / 2] = C22[i][j];		  
      }		  
    } 
  }
}



unsigned long long nanosec()
{
  struct timespec time_start={0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);
  return (unsigned long long)(time_start.tv_sec * 1000000000 + time_start.tv_nsec);
}



void fillArray(float *pData, int iSize)
{
  for(int i=0;i<iSize;i++) {
      //产生-1000到1000之间的随机数
      int x = -100000,y=100000;
      int k = x+rand()%(y-x+1);
      float d = (float)k/100000.f;
      pData[i] = d;
  }
}

int main(int argc ,char **argv)
{
  double delta = 0;
  int i = 1;
  if(argc == 2)
    i = atoi(argv[1]);
  int n = i;
  int j = i;

  float A[N][N], B[N][N], C[N][N];
  srand(time(NULL));//2006);
  fillArray(A[0],N*N);
  fillArray(B[0],N*N);
  fillArray(C[0],N*N);

  shrDeltaT();
  while(i-- > 0)
    strassen(N, A, B, C);
  delta = shrDeltaT();

  printf("strassen time: %fms\n", delta*1000);
  printf("  average: %fms\n", delta*1000/n);


  return 0;
}
