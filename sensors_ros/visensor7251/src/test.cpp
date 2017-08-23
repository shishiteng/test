#include <iostream>
#include <sstream>
#include <fstream>

#include "device.h"
#include <opencv2/opencv.hpp>


using namespace std;
using namespace sn;
using namespace cv;


#define CFG_SYS_REG_NUM 32
#define L_SENSOR_CFG_BASE_ADDR      0x10000
#define R_SENSOR_CFG_BASE_ADDR      0x20000

#define L_REMAP_CFG_BASE_ADDR       0x30000
#define R_REMAP_CFG_BASE_ADDR       0x40000



#define L_PRE_REMAP_BASE_ADDR       0x00000000
#define R_PRE_REMAP_BASE_ADDR       0x00400000
#define L_POST_REMAP_BASE_ADDR      0x00800000
#define R_POST_REMAP_BASE_ADDR      0x00C00000
#define L_REMAP_PARAMETER_BASE_ADDR 0x01400000
#define R_REMAP_PARAMETER_BASE_ADDR 0x01C00000
#define DISPARITY_IMG_BASE_ADDR     0x04000000
#define SGM_LRPD_BASE_ADDR          0x05000000
#define TEST_0_BASE_ADDR            0x07000000
#define TEST_1_BASE_ADDR            0x07400000

double fc = 0.0;
double tc = 0.0;
int mulVal = 16;

int main(int argc, char** argv)
{
  //open device
  device *cam = new device();
  resolution rn = cam->getImageSize();
  int width = rn.width;
  int height = rn.height;

  cv::Size imageSize = cv::Size(width, height);
  if (!cam->init()) {
    return 0;
  }

  //3.device settings

  //
  //sensor reg list
  string regAddr = "/home/vip/catkin_ws/src/sensors_ros/fisheye7251/config/ov7251_registers_addrs_640x480.coe";
  string regData = "/home/vip/catkin_ws/src/sensors_ros/fisheye7251/config/ov7251_registers_datas_640x480.coe";
  if (!cam->setup(regAddr, regData)) {
    return 0;
  }  

#if 0
  //zengyi                                                                                     
  cam->sendCmd(0x350b,0x40,L_SENSOR_CFG_BASE_ADDR);
  cam->sendCmd(0x350b,0x40,R_SENSOR_CFG_BASE_ADDR);

  //baoguang                                                                                   
  double expTime = 5000.0f / 19.3333333;
  uint reg3501 = (uint)(expTime / 16);
  uint reg3502 = (uint)((expTime - reg3501) * 16);

  cam->sendCmd(0x3501, reg3501, 0x10000, 0x00);
  cam->sendCmd(0x3502, reg3502, 0x10000, 0x00);
  cam->sendCmd(0x3501, reg3501, 0x20000, 0x00);
  cam->sendCmd(0x3502, reg3502, 0x20000, 0x00);
#endif

  // imu
  cam->sendCmd(0x1FD, 0x1);
  cam->sendCmd(0x102, 200000);
 
  //
  uint sys_reg_cfg[CFG_SYS_REG_NUM][2] = 
    {
      {0x000,L_PRE_REMAP_BASE_ADDR },
      {0x004,R_PRE_REMAP_BASE_ADDR },
      {0x008,L_POST_REMAP_BASE_ADDR},
      {0x00C,R_POST_REMAP_BASE_ADDR},
      {0x010,L_REMAP_PARAMETER_BASE_ADDR},
      {0x014,R_REMAP_PARAMETER_BASE_ADDR},
      {0x018,DISPARITY_IMG_BASE_ADDR}, 
      {0x01C,SGM_LRPD_BASE_ADDR}, 
      {0x020,TEST_0_BASE_ADDR}, 
      {0x024,TEST_1_BASE_ADDR}, 

        {0x100,   5000000}, //30hz
      //{0x100,   1400000}, //35hz
      //{0x100,   150000000}, /* Freme_rate, unit=10ns*/
        //{0x100, 5000000}, //20fps
        //{0x100,   100000000}, //20fps
      //{0x100, 2000000}, //12fps
      //{0x100, 1400000}, //17.6fps
      {0x104,  200000}, /* Exposuer time ,unit=10ns */
      {0x108,  0x199a}, /* Filter err threshold */
      {0x10C,    8192}, /* Curve fit fact */
      {0x110,      63}, /* btcost sobel threshold */
      {0x114,       1}, /* Dlx_sub_drx_check_threshold */
      {0x118,      24}, /* sgm_p1 */
      {0x11C,      96}, /* sgm_p2 */
      {0x120,       0}, /* test_en */
      {0x124,    0x01}, /* image_display_en */  //0x11
      {0x128,     200}, /* display threshold row num*/
      {0x12C,      30}, /* uniqueness_check_factor */
      {0x1FC,       1}  /* sensor dout enable */
    };

  for(int i=0; i<CFG_SYS_REG_NUM; i++) {
    cam->sendCmd(sys_reg_cfg[i][0],sys_reg_cfg[i][1],0x0);
    //cv::waitKey(5);
  }

  //cam->sendCmd(0x100,0x00,0,1);

  //reset
  //cout << "Reset device...\n";
  //cyusb_reset_device(handle);

  cam->start();

  Mat remapLeft = Mat(Size(width, height), CV_16UC1);
  Mat remapRight = remapLeft.clone();

  Mat left8u = Mat(Size(width, height), CV_8UC1);
  Mat right8u = left8u.clone();

  imuData imudata;
  
  //4.capture image
  while (1) {
    //get frame
    if(cam->readIMU(&imudata)){
      printf("imu  :  %lu\n",imudata.imu_timestamp);
    }
    continue;

    #if 0
    if (!cam->QueryFrame(remapLeft, remapRight)) {
      //cout<<"no image\n";
      continue;
    }
    #endif
    
    convertScaleAbs(remapLeft, left8u, 255 / 1023.0);
    convertScaleAbs(remapRight, right8u, 255 / 1023.0);

    if(!left8u.empty() && !right8u.empty()) {
	imshow("left",left8u);
	imshow("right",right8u);
	waitKey(1);
    }
  }

  cam->close();

  return 0;
}
