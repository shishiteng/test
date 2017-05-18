
#include <iostream>
#include <opencv2/opencv.hpp>
#include <helper_cuda.h>
#include <timer.h>
#include <string>
#include "corner.h"
#include "fast_cuda.h"

int main( void )
{
	using namespace std;
	using namespace cv;
	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);

	cudaEvent_t kernel_start;
	cudaEventCreate(&kernel_start);

    const int threshold=20;
    string filename="zazhi.jpg";
	Mat image;
	image = cv::imread(filename,0);   // Read the file
	if(! image.data )                              // Check for invalid input
	{
		cout <<  "Could not open or find the image" << std::endl ;
		return -1;
	}
	cudaEventRecord(start);

	uchar* d_data;                // create a pointer
	size_t imSize=image.cols*image.rows;
	Corner* h_corner=new Corner[imSize];
	Corner* d_corner;
	checkCudaErrors(cudaMalloc((void**) &d_corner,sizeof(Corner)*imSize));
	checkCudaErrors(cudaMalloc((void**) &d_data, sizeof(uchar)*imSize)); // create memory on the gpu and pass a pointer to the host
	checkCudaErrors(cudaMemcpy(d_data, image.data, sizeof(uchar)*imSize, cudaMemcpyHostToDevice));// copy from the image data to the gpu memory you reserved
	dim3 blocksize(16,16);
	dim3 gridsize((image.cols-1)/blocksize.x+1, (image.rows-1)/blocksize.y+1, 1);
	//cudaEventRecord(start);
	cudaEventRecord(kernel_start);
	fast<<<gridsize,blocksize>>>(d_data, image.cols, image.rows,d_corner,gridsize.x,gridsize.y,threshold); // processed data on the gpu
	//checkCudaErrors(cudaDeviceSynchronize());
	//cudaEventRecord(stop);	
	//cudaEventSynchronize(stop);
	nms<<<gridsize,blocksize>>>(d_data,d_corner,image.cols,image.rows);
	checkCudaErrors(cudaMemcpy(h_corner,d_corner,sizeof(Corner)*imSize,cudaMemcpyDeviceToHost));

	cudaEventRecord(stop);	
	cudaEventSynchronize(stop);

	float elptime;
	cudaEventElapsedTime(&elptime,start,stop);

	float elptime2;
	cudaEventElapsedTime(&elptime2,kernel_start,stop);

	//show the corner in the image
	Mat image_color = imread(filename,1);
    int point=0;
	for(int i=0;i<imSize;i++)
	{
		if(h_corner[i].set!=0)
		{
			int x=i%image.cols;
			int y=i/image.cols;
			circle(image_color,Point(x,y),1,Scalar(0,255,0),-1,8,0);
			point++;
		}
	}
	cout<<"points:"<<point<<endl;
	cout<<"Elapsed time:"<<elptime<<"ms"<<endl;
	cout<<"Elapsed kernel time:"<<elptime2<<"ms"<<endl;

	//printf("%x\n",0x7|((10>1)<<3));
	//cout<<"the size of: "<<sizeof(corner)<<endl;
	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
	imshow( "Display window", image_color );                   // Show our image inside it.
	waitKey(0);                                          // Wait for a keystroke in the window
	delete[] h_corner;
	cudaFree(d_corner);
	cudaFree(d_data);
	cudaEventDestroy(start);
	cudaEventDestroy(stop);
	return 0;
}
