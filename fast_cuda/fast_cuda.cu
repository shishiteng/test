#include <iostream>
#include "fast_cuda.h"
#include <opencv2/opencv.hpp>
#include "corner.h"
#include <stdio.h>

__device__
int position(int m,int n,int width)
{
	int idx=m+n*width;
	return idx;
}
__global__
void fast(uchar* image, int width, int height,Corner* d_corner,int gridsize_x, int gridsize_y, const int threshold)
{
	__shared__ uchar patch[22][22];
	uint sp=0,sn=0;
	int m=blockDim.x*blockIdx.x+threadIdx.x;
	int n=blockDim.y*blockIdx.y+threadIdx.y;
	uint idx = m+n*width;
	uint idx_block=threadIdx.y*blockDim.x+threadIdx.x;               //offset for pixel in patch
	
	(d_corner+idx)->set=0;
	(d_corner+idx)->value=0;
	int patch_top_x=blockDim.x*blockIdx.x-3;
	int patch_top_y=blockDim.y*blockIdx.y-3;
	int idx_block_256=idx_block+256;

    //load into patch
	patch[idx_block%22][idx_block/22]=image[position(patch_top_x+idx_block%22,patch_top_y+idx_block/22,width)];
	if(idx_block_256<484)
		patch[(idx_block+256)%22][(idx_block+256)/22]=image[position(patch_top_x+idx_block_256%22,patch_top_y+idx_block_256/22,width)];
	__syncthreads();

	//detect
	int x=3+threadIdx.x;
    int y=3+threadIdx.y;
	if(m>2&&m<(width-3)&&n>2&&n<(height-3))            //detect the points away from the edges
	{
		uchar center_value = patch[x][y];
		sp |=(patch[x][y-3]>(center_value + threshold))<< 0;
		sp |=(patch[x+1][y-3]>(center_value + threshold))<< 1;
		sp |=(patch[x+2][y-2]>(center_value + threshold))<< 2;
		sp |=(patch[x+3][y-1]>(center_value + threshold))<< 3;
		sp |=(patch[x+3][y]>(center_value + threshold))<< 4;
		sp |=(patch[x+3][y+1]>(center_value + threshold))<< 5;
		sp |=(patch[x+2][y+2]>(center_value + threshold))<< 6;
		sp |=(patch[x+1][y+3]>(center_value + threshold))<< 7;
		sp |=(patch[x][y+3]>(center_value + threshold))<< 8;
		sp |=(patch[x-1][y+3]>(center_value + threshold))<< 9;
		sp |=(patch[x-2][y+2]>(center_value + threshold))<< 10;
		sp |=(patch[x-3][y+1]>(center_value + threshold))<< 11;
		sp |=(patch[x-3][y]>(center_value + threshold))<< 12;
		sp |=(patch[x-3][y-1]>(center_value + threshold))<< 13;
		sp |=(patch[x-2][y-2]>(center_value + threshold))<< 14;
		sp |=(patch[x-1][y-3]>(center_value + threshold))<< 15;

		sp+=sp<<16;
		uint sp1=sp&(sp<<1);
		uint sp2=sp1&(sp1<<2);
		uint sp3=sp2&(sp2<<4);
		uint sp4=sp3&(sp<<8);
		if(sp4!=0)
		{
			int value=abs(center_value-patch[x-1][y-1])+abs(center_value-patch[x][y-1])+abs(center_value-patch[x+1][y-1])+
					abs(center_value-patch[x-1][y])+abs(center_value-patch[x+1][y])+abs(center_value-patch[x+1][y-1])+
					abs(center_value-patch[x+1][y])+abs(center_value-patch[x+1][y+1]);
			d_corner[idx].value=value;
			d_corner[idx].set=1;
		}
		else
		{
			sn |=(patch[x][y-3]<(center_value - threshold))<< 0;
			sn |=(patch[x+1][y-3]<(center_value - threshold))<< 1;
			sn |=(patch[x+2][y-2]<(center_value - threshold))<< 2;
			sn |=(patch[x+3][y-1]<(center_value - threshold))<< 3;
			sn |=(patch[x+3][y]<(center_value - threshold))<< 4;
			sn |=(patch[x+3][y+1]<(center_value - threshold))<< 5;
			sn |=(patch[x+2][y+2]<(center_value - threshold))<< 6;
			sn |=(patch[x+1][y+3]<(center_value - threshold))<< 7;
			sn |=(patch[x][y+3]>(center_value - threshold))<< 8;
			sn |=(patch[x-1][y+3]<(center_value - threshold))<< 9;
			sn |=(patch[x-2][y+2]<(center_value - threshold))<< 10;
			sn |=(patch[x-3][y+1]<(center_value - threshold))<< 11;
			sn |=(patch[x-3][y]<(center_value - threshold))<< 12;
			sn |=(patch[x-3][y-1]<(center_value - threshold))<< 13;
			sn |=(patch[x-2][y-2]<(center_value - threshold))<< 14;
			sn |=(patch[x-1][y-3]<(center_value - threshold))<< 15;
			sn+=sn<<16;
			uint sn1=sn&(sn<<1);
			uint sn2=sn1&(sn1<<2);
			uint sn3=sn2&(sn2<<4);
			uint sn4=sn3&(sn<<8);
			if(sn4!=0)
			{
				int value=abs(center_value-patch[x-1][y-1])+abs(center_value-patch[x][y-1])+abs(center_value-patch[x+1][y-1])+
						abs(center_value-patch[x-1][y])+abs(center_value-patch[x+1][y])+abs(center_value-patch[x+1][y-1])+
						abs(center_value-patch[x+1][y])+abs(center_value-patch[x+1][y+1]);
				d_corner[idx].value=value;
				d_corner[idx].set=1;
				printf("");
			}
		}
	}

}
__global__
void nms(uchar* image, Corner* d_corner,int width, int height)
{
	int m=blockDim.x*blockIdx.x+threadIdx.x;
	int n=blockDim.y*blockIdx.y+threadIdx.y;
	int idx=n*width+m;
	if(d_corner[idx].set==1)
	{
		int corner_value=d_corner[idx].value;
		if(d_corner[position(m-1,n-1,width)].value> corner_value)
		{d_corner[idx].set=0;return;}
		if(d_corner[position(m,n-1,width)].value> corner_value)
		{d_corner[idx].set=0;return;}
		if(d_corner[position(m+1,n-1,width)].value> corner_value)
		{d_corner[idx].set=0;return;}
		if(d_corner[position(m-1,n,width)].value> corner_value)
		{d_corner[idx].set=0;return;}
		if(d_corner[position(m+1,n,width)].value> corner_value)
		{d_corner[idx].set=0;return;}
		if(d_corner[position(m+1,n-1,width)].value> corner_value)
		{d_corner[idx].set=0;return;}
		if(d_corner[position(m+1,n,width)].value> corner_value)
		{d_corner[idx].set=0;return;}
		if(d_corner[position(m+1,n+1,width)].value> corner_value)
		{d_corner[idx].set=0;return;}

	}
}

