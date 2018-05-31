/*
 * device.cpp
 *
 *  Created on: Apr 28, 2017
 *      Author: vip2
 */

#include "device.h"
#include "snTool.h"
#include <fstream>

using namespace std;
using namespace cv;
using namespace snTool;

#include <unistd.h>
#include <queue>
#include <mutex>
#include <condition_variable>

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

// CFG begin
#define CFG_REG_SPACE_ROW_LEN        4096
#define CFG_SYS_REG_SPACE_SIZE                CFG_REG_SPACE_ROW_LEN
#define SYS_CTRL_REG_BASE_ADDR     (0x0<<16)
#define L_SENSOR_REG_BASE_ADDR     (0x1<<16)
#define R_SENSOR_REG_BASE_ADDR     (0x2<<16)
#define IMU_REG_BASE_ADDR          (0x3<<16)


namespace sn
{
// 宏定义分辨率
#define VGA_WIDTH 640
#define VGA_HEIGHT 480
// 接收发送的addr
#define BULK_OUT 0x01
#define BULK_IN 0x81
//
#define ReadDataBytes 524288
//#define ReadDataBytes (16*1024)
// 指令字节数
#define CMDBYTES 16
#define HEADBYTES CMDBYTES

#define IMU_BYTES 48
static bool bStreaming = false;


mutex m_buf;
mutex i_buf;
queue<imuData> imu_buf;
condition_variable con_imu;
condition_variable con_img;

  
device::device(resolution_mode rm /*= resolution_mode::VGA*/,
               imgMode mode /*= imgMode::SLAR*/)
{
    // TODO Auto-generated constructor stub
    m_bulkin_addr = BULK_IN;
    m_bulkout_addr = BULK_OUT;

    m_leftOffset = 0x10000;
    m_rightOffset = 0x20000;

    if (VGA == rm)
    {
        m_width = VGA_WIDTH;
        m_height = VGA_HEIGHT;
    }
    else
    {
        // 默认 VGA
        m_width = VGA_WIDTH;
        m_height = VGA_HEIGHT;
    }

    m_mode = mode;

    if (SLAR == m_mode)
    {
        cv::Size imageSize = cv::Size(m_width, m_height);
        m_leftImage = Mat(imageSize, CV_16UC1);
        m_leftImage_cache = m_leftImage.clone();
        m_rightImage = m_leftImage.clone();
        m_rightImage_cache = m_leftImage.clone();
        m_dispImage = m_leftImage.clone();
        m_dispImage_cache = m_leftImage.clone();
        m_remap_leftImage = m_leftImage.clone();
        m_remap_leftImage_cache = m_leftImage.clone();
        m_remap_rightImage = m_leftImage.clone();
        m_remap_rightImage_cache = m_leftImage.clone();
        // 指针赋值
        m_leftImage_ptr = m_leftImage.data;
        m_leftImage_cache_ptr = m_leftImage_cache.data;
        m_rightImage_ptr = m_rightImage.data;
        m_rightImage_cache_ptr = m_rightImage_cache.data;
        m_dispImage_ptr = m_dispImage.data;
        m_dispImage_cache_ptr = m_dispImage_cache.data;
        m_remap_leftImage_ptr = m_remap_leftImage.data;
        m_remap_leftImage_cache_ptr = m_remap_leftImage_cache.data;
        m_remap_rightImage_ptr = m_remap_rightImage.data;
        m_remap_rightImage_cache_ptr = m_remap_rightImage_cache.data;
        // 每行字节数
        m_single_rowBytes = m_width;
        m_sbs_rowBytes = m_single_rowBytes * 2;
        m_disp_rowBytes = m_single_rowBytes;
    }
    else if (SBS == m_mode)
    {
        Size sbsSize = Size(m_width * 2, m_height);
        m_sbsImage = Mat(sbsSize, CV_16UC1);
        m_sbsImage_cache = m_sbsImage.clone();

        m_sbsImage_ptr = m_sbsImage.data;
        m_sbsImage_cache_ptr = m_sbsImage_cache.data;

        m_sbs_rowBytes = m_width * 4;
        m_disp_rowBytes = m_width * 2;
    }

    //初始化spin
    pthread_spin_init(&m_imglock, 0);
    pthread_spin_init(&m_displock, 0);
    pthread_spin_init(&m_remaplock, 0);
    pthread_spin_init(&m_imulock, 0);

    m_existNewImg = false;
    m_existNewRemap = false;
    m_existNewDisp = false;
    m_existNewImu = false;

    m_remainCount = 0;
    m_cacheInBuf = new uchar[ReadDataBytes * 2];
}

device::~device()
{
    // TODO Auto-generated destructor stub
    if (SBS == m_mode)
    {
        m_sbsImage.release();
        m_sbsImage_cache.release();
    }
    else if (SLAR == m_mode)
    {
        m_leftImage.release();
        m_leftImage_cache.release();
        m_rightImage.release();
        m_rightImage_cache.release();
        m_remap_leftImage.release();
        m_remap_leftImage_cache.release();
        m_remap_rightImage.release();
        m_remap_rightImage_cache.release();
    }

    delete[] m_cacheInBuf;
}

void sn::device::close()
{

  // fpga停止发送数据    
  sendCmd(0x0, 0x0, 0xFC, 0, 0);
  
  // 等待接收线程结束
  pthread_join(m_collect_thread, NULL);

  // 接完usb3.0缓存数据
  cout<<"clear usb buffer..."<<endl;
  int len = 16*1024;
  uchar buffer[len];
  int timeout = 200;
  while(1) {
    int transferred = -1;
    int usbErr = cyusb_bulk_transfer(m_device, m_bulkin_addr, buffer, len, &transferred, timeout);
    if(0 != usbErr)
      break;
    else
      cout<<"received: "<<transferred<<endl;
  }

  // 关闭cyusb
  cyusb_close();
}

bool device::init()
{
    // 检测设备个数
    int devNum = -1;
    devNum = cyusb_open();
    if (devNum < 0)
    {
        cerr << "Error opening library" << endl;
        return false;
    }
    else if (devNum == 0)
    {
        cerr << "未找到设备..." << endl;
        return false;
    }

    // 查找对应的vid和pid设备
    const int VID = 0x04B4;
    const int PID = 0x00F1;
    cyusb_handle *handle = NULL;
    for (int i = 0; i < devNum; ++i)
    {
        handle = cyusb_gethandle(i);
        if (cyusb_getvendor(handle) == VID && cyusb_getproduct(handle) == PID)
        {
            cout << "开启设备..." << endl;
            break;
        }

        if (i == devNum)
        {
            cerr << "未找到指定设备..." << endl;
            cyusb_close();
            return false;
        }
    }

    m_device = handle;

    // 与设备连接
    int usbErr = -1;
    usbErr = cyusb_kernel_driver_active(m_device, 0);
    if (usbErr != 0)
    {
        cerr << "kernel driver active. Exitting\n"
             << endl;
        cyusb_close();
        return false;
    }
    usbErr = cyusb_claim_interface(m_device, 0);
    if (usbErr != 0)
    {
        cerr << "Error in claiming interface\n"
             << endl;
        cyusb_close();
        return false;
    }
    else
        cout << "Successfully claimed interface\n"
             << endl;

    //reset
    //cout << "Reset device...\n";
    //cyusb_reset_device(handle);


    return true;
}

bool sn::device::clearData()
{

    return false;
}

bool sn::device::setup(std::string regAddr, std::string regData)
{
    //sendCmd(0x4, 0x0, 0x0, 0x2);

    ifstream regAddrFile(regAddr.c_str());
    ifstream regDataFile(regData.c_str());

    if (regAddrFile && regDataFile)
    {
        std::string lineAddrStr;
        std::string lineDataStr;

        while (getline(regAddrFile, lineAddrStr) && getline(regDataFile, lineDataStr))
        {
            long addrval = snTool::hex2int(lineAddrStr);
            long dataval = snTool::hex2int(lineDataStr);

            if (addrval+1)
            {
                sendCmd(addrval, dataval, m_leftOffset);
                sendCmd(addrval, dataval, m_rightOffset);
            }
        }
    }
    else
    {
        cerr << "文件不存在！" << endl;
        return false;
    }

    regAddrFile.close();
    regDataFile.close();

    //sendCmd(0x4, 0x1, 0x0, 0x2);

    return true;
}

bool sn::device::start()
{
    // 开启线程
    bStreaming = true;
    pthread_create(&m_collect_thread, NULL, XferLoop, this);

    return true;
}

int device::getImageData(cv::Mat &leftImg, cv::Mat &rightImg, uint64 &imgTimeStamp)
{
  if (m_existNewImg) {
    swapPtr((void **)&leftImg.data, (void **)&m_leftImage_ptr);
    swapPtr((void **)&rightImg.data, (void **)&m_rightImage_ptr);
    imgTimeStamp = m_imgTimeStamp;
    m_existNewImg = false;
    return 0;
  }
  return -1;
}
  //修改成阻塞读取的方式
bool sn::device::QueryFrame(cv::Mat &leftImg, cv::Mat &rightImg, uint64 &imgTimeStamp)
{
  std::unique_lock<std::mutex> lk(i_buf);
  con_img.wait(lk, [&]
	       {
		 return ( 0 == getImageData(leftImg,rightImg,imgTimeStamp));
	       });
  lk.unlock();

  return true;
#if 0
  if (m_existNewImg) {
    pthread_spin_lock(&m_imglock);
    swapPtr((void **)&leftImg.data, (void **)&m_leftImage_ptr);
    swapPtr((void **)&rightImg.data, (void **)&m_rightImage_ptr);
    imgTimeStamp = m_imgTimeStamp;
    m_existNewImg = false;
    pthread_spin_unlock(&m_imglock);
    return true;
  }

  return false;
#endif
}

bool device::QueryFrame(cv::Mat &leftImg, cv::Mat &rightImg, cv::Mat &dispImg)
{
    if (m_existNewImg)
    {
        pthread_spin_lock(&m_imglock);
        swapPtr((void **)&leftImg.data, (void **)&m_leftImage_ptr);
        swapPtr((void **)&rightImg.data, (void **)&m_rightImage_ptr);
        m_existNewImg = false;
        pthread_spin_unlock(&m_imglock);
    }

    if (m_existNewDisp)
    {
        pthread_spin_lock(&m_displock);
        swapPtr((void **)&dispImg.data, (void **)&m_dispImage_ptr);
        m_existNewDisp = false;
        pthread_spin_unlock(&m_displock);
    }

    return true;
}

int device::getImuData(imuData *imudata)
{
  if(!imu_buf.empty()) {
    imuData data = imu_buf.front();
    memcpy(imudata,&data,48);
    imu_buf.pop();
    return 0;
  }
  
  return -1;
}
  
//阻塞读取
bool device::readIMU(imuData *imudata){
  std::unique_lock<std::mutex> lk(m_buf);
  con_imu.wait(lk, [&]
	   {
	     return ( 0 == getImuData(imudata));
	   });
  lk.unlock();
  
  return true;
  
#if 0
  imuData data;
  if(!imu_buf.empty()) {
    data = imu_buf.front();
    memcpy(imudata,&data,48);
    //m_buf.lock();
    pthread_spin_lock(&m_imulock);//
    imu_buf.pop();
    pthread_spin_unlock(&m_imulock);
    //m_buf.unlock();
    return true;
  }

  return false;
#endif
}

bool device::remapFrame(cv::Mat &leftImg, cv::Mat &rightImg)
{
    if (m_existNewRemap)
    {
        pthread_spin_lock(&m_remaplock);
        swapPtr((void **)&leftImg.data, (void **)&m_remap_leftImage_ptr);
        swapPtr((void **)&rightImg.data, (void **)&m_remap_rightImage_ptr);
        m_existNewRemap = false;
        pthread_spin_unlock(&m_remaplock);
        return true;
    }

    return false;
}

bool device::disFrame(cv::Mat &dispImg)
{
    if (m_existNewDisp)
    {
        pthread_spin_lock(&m_displock);
        swapPtr((void **)&dispImg.data, (void **)&m_dispImage_ptr);
        m_existNewDisp = false;
        pthread_spin_unlock(&m_displock);
        return true;
    }

    return false;
}

resolution device::getImageSize()
{
    return resolution(m_width, m_height);
}

bool device::sendCmd(uint Base_addr, uint Data, uint offset /*= 0x0*/, uint cmd /*= 0x0*/, uint dir /*= 0x0*/, uint id /*= 0x1234*/)
{
    // 偏移量
    Base_addr += offset;

    long sendbytes = CMDBYTES;
    uchar *buf = new uchar[sendbytes];

    memset(buf, 0, CMDBYTES);
    unsigned int *data = (unsigned int *)buf;
    data[0] = Data << 16;
    data[1] = (Base_addr << 16) | (Data >> 16);
    data[2] = Base_addr >> 16;
    data[3] = (id << 16) | (dir << 4) | cmd;

    int transferred = 0;
    int timeout = 200;
    int usbErr = -1;
    usbErr = cyusb_bulk_transfer(m_device, m_bulkout_addr, buf, sendbytes, &transferred, timeout);
    // 释放
    delete[] buf;
    if (usbErr == 0)
    {
      //cerr<<"."<<endl;
        return true;
    }
    else
    {
        cyusb_error(usbErr);
        cyusb_close();
        return false;
    }
}

bool device::uploadRemap(std::string filename, bool left, int bits)
{
    uchar *remap = NULL;
    int bytes = (bits / 8) * m_width * m_height;
    snTool::readBinaryFile<uchar>(filename, &remap, bytes);

    int rowBytes = (bits / 8) * m_width;
    for (unsigned int i = 0; i < m_height; ++i)
    {
        if (left)
        {
            uint cmd = 3;
            if (!sendData(remap + i * rowBytes, rowBytes, i, cmd))
            {
                return false;
            }
        }
        else
        {
            uint cmd = 4;
            if (!sendData(remap + i * rowBytes, rowBytes, i, cmd))
            {
                return false;
            }
        }
    }

    return true;
}

int device::receiveData(uchar *buf, int &transferred)
{
    int timeout = 1000;
    return cyusb_bulk_transfer(m_device, m_bulkin_addr, buf, ReadDataBytes, &transferred, timeout);
}

 void *XferLoop(void *args)
{
    device *cam = (device *)args;

    int len = ReadDataBytes;
    uchar buffer[len];

    while (bStreaming) {
      //cout<<"=="<<endl;
      int usbErr = -1;
      int transferred = 0;
      

      usbErr = cam->receiveData(buffer, transferred);
      //cout<<"receive data..."<<transferred<<endl;

      if (usbErr == 0) {
	//cout<<"parse data..."<<endl;
	cam->parseData(buffer, transferred);
      } else {
        //cout<<"receive data timeout."<<endl;
        break;
      }
    }
}

void device::parseData(const void *lpData, const long dwSize)
{
    int iBytes = 0;
    iBytes = dwSize + m_remainCount;
    memcpy(m_cacheInBuf + m_remainCount, lpData, dwSize);

    int cur = 0;
    bool end = false;
    while (!end && cur + HEADBYTES <= iBytes)
    {
        unsigned int *headData = (unsigned int *)(m_cacheInBuf + cur);
        unsigned int id = headData[3] >> 16;
        unsigned int cmd = headData[3] & 0x000f;
	//cout<<"id:"<<id<<"   cmd:"<<cmd<<endl;
        if (id == 4660)
        {
	  //cout<<"4660"<<"  "<<cmd<<endl;
            switch (cmd)
            {
            case 1:
            {
                // 解析指令
                unsigned int Data = headData[1] << 16 | headData[0] >> 16;
                unsigned int Base_addr = headData[2] << 16 | headData[1] >> 16;
                //FILE *fp;
                //fp = fopen("./reg.txt", "a");
                //fprintf(fp, "œÓÊÕ·µ»Ø <==> BAddr:%x\n              Data :%x\n", Base_addr, Data);
                //fclose(fp);

                cout << "Base_addr:" << Base_addr << endl;
                cout << "Data:" << Data << endl;

                cur += HEADBYTES;
                break;
            }
            case 4:
            {
                // 原图src
                if (cur + HEADBYTES + m_sbs_rowBytes > iBytes)
                {
                    end = true;
		    //cout<<"end"<<endl;
                    break;
                }

                unsigned int row = headData[2] & 0xffff;
                unsigned int frameNum = (headData[3] >> 8) & 0xff;
                //cout << "row:" << row << endl;
                writeOneRowImg(headData, row, frameNum);
                cur += HEADBYTES + m_sbs_rowBytes;
                break;
            }
            case 6:
            {
                // remap
                if (cur + HEADBYTES + m_sbs_rowBytes > iBytes)
                {
                    end = true;
                    break;
                }

                unsigned int row = headData[2] & 0xffff;
                //cout << "row:" << row << endl;
                writeOneRowRemapImg(headData, row);
                cur += HEADBYTES + m_sbs_rowBytes;
                break;
            }
            case 7:
            {
                // disp
                if (cur + HEADBYTES + m_disp_rowBytes > iBytes)
                {
                    end = true;
                    break;
                }

                unsigned int row = headData[2] & 0xffff;
                //cout << "row:" << row << endl;
                writeOneRowDisp(headData, row);
                cur += HEADBYTES + m_disp_rowBytes;
                break;
            }
            case 8:
            {
                break;
            }
            case 10:
            {
                break;
            }

            case 14:
            {
	      //IMU data
	      unsigned int bytesSize = ((headData[2] >> 16) & 0x0fff)*16;
	      if(cur+ HEADBYTES + bytesSize > iBytes) {
		end = true;
		break;
	      }
                    
	      imuData imudata;
	      memcpy(&imudata,headData + 28,IMU_BYTES);
	      m_buf.lock();
	      //pthread_spin_lock(&m_imulock);
	      imu_buf.push(imudata);
	      //pthread_spin_unlock(&m_imulock);//mutex换成自旋锁
	      m_buf.unlock();
	      con_imu.notify_one();
	      cur += HEADBYTES + bytesSize;

	      break;
            }
		case 15:
		{
		    //空包
		    unsigned int bytesSize = ((headData[2] >> 16) & 0x0fff)*16;
                    if(cur+ HEADBYTES + bytesSize > iBytes)
                    {
                    end = true;
                    break;
                    }

                    cur += HEADBYTES + bytesSize;
                    break;

		}
            default:
            {
                cur += HEADBYTES;
                break;
            }
            }
        }
        else
        {
            // id不符合直接丢弃整包
	  cout<<"throw"<<endl;
            cur = iBytes;
            m_remainCount = 0;
        }
    }

    if (cur < iBytes)
    {
        m_remainCount = iBytes - cur;
        memcpy(m_cacheInBuf, m_cacheInBuf + cur, m_remainCount);
    }
    else if (cur == iBytes)
    {
        m_remainCount = 0;
    }
}

bool device::writeOneRowImg(const void *data, unsigned int row, unsigned int frameNum)
{
    if (row >= 0 && row < m_height)
    {
        memcpy(m_leftImage_cache_ptr + row * m_single_rowBytes, (unsigned char *)data + HEADBYTES, m_single_rowBytes);
        memcpy(m_rightImage_cache_ptr + row * m_single_rowBytes, (unsigned char *)data + HEADBYTES + m_single_rowBytes, m_single_rowBytes);

        if (row >= m_height - 1)
        {
            
	    m_imgTimeStamp = ((uint64*)data)[0] >> 16;

            //pthread_spin_lock(&m_imglock);
	    i_buf.lock();
            swapPtr((void **)&m_leftImage_cache_ptr, (void **)&m_leftImage_ptr);
            swapPtr((void **)&m_rightImage_cache_ptr, (void **)&m_rightImage_ptr);
            m_existNewImg = true;
	    i_buf.unlock();
	    con_img.notify_one();
            //pthread_spin_unlock(&m_imglock);
        }
        return true;
    }
    return false;
}

bool device::writeOneRowRemapImg(const void *data, uint row)
{
    if (row >= 0 && row < m_height)
    {
        // 内存拷贝
        memcpy(m_remap_leftImage_cache_ptr + row * m_single_rowBytes, (unsigned char *)data + HEADBYTES, m_single_rowBytes);
        memcpy(m_remap_rightImage_cache_ptr + row * m_single_rowBytes, (unsigned char *)data + HEADBYTES + m_single_rowBytes, m_single_rowBytes);

        if (row >= m_height - 1)
        {
            pthread_spin_lock(&m_remaplock);
            swapPtr((void **)&m_remap_leftImage_cache_ptr, (void **)&m_remap_leftImage_ptr);
            swapPtr((void **)&m_remap_rightImage_cache_ptr, (void **)&m_remap_rightImage_ptr);
            m_existNewRemap = true;
            pthread_spin_unlock(&m_remaplock);
        }
        return true;
    }

    return false;
}

bool device::writeOneRowDisp(const void *data, uint row)
{
    if (row >= 0 && row < m_height)
    {

        memcpy(m_dispImage_cache_ptr + row * m_disp_rowBytes, (unsigned char *)data + HEADBYTES, m_disp_rowBytes);

        if (row >= m_height - 1)
        {
            pthread_spin_lock(&m_displock);
            swapPtr((void **)&m_dispImage_cache_ptr, (void **)&m_dispImage_ptr);
            m_existNewDisp = true;
            pthread_spin_unlock(&m_displock);
        }

        return true;
    }

    return false;
}

inline void device::swapPtr(void **p1, void **p2)
{
    void *temp = *p1;
    *p1 = *p2;
    *p2 = temp;
}

bool device::sendData(void *data, int bytes, uint rowNum, uint cmd,
                      uint id /*=0x1234*/)
{
    long totalBytes = HEADBYTES + bytes;
    uchar *sendData = (uchar *)malloc(totalBytes);
    memset(sendData, 0, totalBytes);

    uint *dataInt = (uint *)sendData;
    uint type_len = 0xf;
    uint total_size = bytes / 16;
    dataInt[3] = (id << 16) | (type_len << 4) | cmd;
    dataInt[2] = (total_size << 16) | rowNum;

    memcpy(sendData + HEADBYTES, data, bytes);

    int transferred = 0;
    int timeout = 200;
    int usbErr = -1;
    usbErr = cyusb_bulk_transfer(m_device, m_bulkout_addr, sendData, totalBytes, &transferred, timeout);
    free(sendData);
    if (usbErr == 0)
    {
        return true;
    }
    else
    {
        cyusb_error(usbErr);
        cyusb_close();
        return false;
    }
}

/****************加入硬件同步，接口扩展，start**************/
  
// 配置设备：imu帧率、图像帧率、最大曝光时间、最小曝光时间、当前曝光时间
bool device::config(int img_freq,int imu_freq,int max_exposure_time,int min_exposure_time,int cur_exposure_time)
{

  uint sys_reg_cfg[CFG_SYS_REG_SPACE_SIZE / 4][2] =
    {
      //{ 0x00,        100000000/img_freq}, /* Freme_rate, Note: The units must be 10ns, don't modify. */
      { 0x00,        img_freq},
      { 0x04,                            0 }, /* reserved */
      { 0x08,                            0 }, /* reserved */

      { 0x0C,            min_exposure_time }, /* [31:0] min exposuer time,unit=1us */
      { 0x10,            max_exposure_time }, /* [31:0] max exposuer time,unit=1us */
      { 0x14,            cur_exposure_time }, /* [31:0] current exposure time ,unit=1us */
      { 0x18,                   0x00ff503C }, /* [31:16] max light,         [15:12] expect exposure light offest, [11:0] expect exposure light avg. ,unit=1us */
      { 0x1C,                            0 }, /* reserved */
      { 0x20,                            0 }, /* reserved */

      { 0x24,                0x5 << 16 | 0x7 }, /* [23:16] gain frame gap,    [3: 0] sensor auto-gain tap numberr */
#if 0
      { 0x28,                         0x10 }, /* [31:00] auto-gain tap 00 */
      { 0x2C,                         0x20 }, /* [31:00] auto-gain tap 01 */
      { 0x30,                         0x30 }, /* [31:00] auto-gain tap 02 */
      { 0x34,                         0x40 }, /* [31:00] auto-gain tap 03 */
      { 0x38,                         0x50 }, /* [31:00] auto-gain tap 04 */
      { 0x3C,                         0x60 }, /* [31:00] auto-gain tap 05 */
      { 0x40,                         0x70 }, /* [31:00] auto-gain tap 06 */
      { 0x44,                         0x80 }, /* [31:00] auto-gain tap 07 */
      { 0x48,                         0x90 }, /* [31:00] auto-gain tap 08 */
      { 0x4C,                         0xa0 }, /* [31:00] auto-gain tap 09 */
#endif
      { 0x50,                         0xb0 }, /* [31:00] auto-gain tap 10 */
      { 0x54,                         0xc0 }, /* [31:00] auto-gain tap 11 */
      { 0x58,                         0xd0 }, /* [31:00] auto-gain tap 12 */
      { 0x5C,                         0xe0 }, /* [31:00] auto-gain tap 13 */
      { 0x60,                         0xf0 }, /* [31:00] auto-gain tap 14 */
      { 0x64,                         0xff }, /* [31:00] auto-gain tap 15 */
      { 0x68,                            0 }, // reserved
      { 0x6C,                            0 }, // reserved

      { 0x70,                           63 }, /* [7 : 0] btcost sobel threshold */
      { 0x74,                   0x00600018 }, /* [31:16] sgm_p2            ,[15 :0] sgm_p1 */
      { 0x78,                         8192 }, /* [15: 0] Curve fit fact */
      { 0x7C,                       0x011e }, /* [15: 8] Dlx_sub_drx_check_threshold, [7:0] uniqueness_check_factor */
      { 0x80,            0 }, /* [15: 0] invalid data map value */
      { 0x84,                         0x44 }, /* [7 : 4] grid box_y number.  [3:0] grid box_x number. */
      { 0x88,                            0 }, /* reserved */
      { 0x8C,                            0 }, /* reserved */

      { 0x90,             0 }, /* */
      { 0x94,                            0 }, /* reserved */
      { 0x98,             0 }, /* */
      { 0x9C,                            0 }, /* reserved */

      { 0xA0,            0 }, /* [15:0] sensor_cfg_regNum */
      { 0xA4,        0 }, /* [15:0] disp2depth register number */
      { 0xA8,                            0 }, /* reserved */
      { 0xAC,                          200 }, /* [15: 0] display threshold row num */

      { 0xB0,               imu_freq }, /* [15:0] disp2depth register number */
      { 0xB4,        0 }, /* reserved */
      { 0xB8,        0 }, /* [15: 0] display threshold row num */

      { 0xFC,                       0xC7E1 }  /* [7 : 0] system enable */
      //{ 0xFC,                       0xC701 }  //关闭自动曝光、自动增益
    };
  /* system enable
  // [10] imu enable, default= 1
  // [9] sensor enable, default= 1
  // [8] disp2depth enable, default= 1
  // [7] auto-exposure enable, default= 1
  // [6] auto-gain enable, default= 1
  // [5] left/right check enable, default= 1
  // [4] post_img_test enable, default= 0
  // [3] pre_img_test enable, default= 0
  // [2] disparity img display enable, default= 1
  // [1] post_img display enable, default= 0
  // [0] pre_img display enable, default= 0
  */
  for (int i = 0; i <= (0xBC-40) / 4; i++) {
    sendCmd(SYS_CTRL_REG_BASE_ADDR, sys_reg_cfg[i][1], sys_reg_cfg[i][0], 0, 0);
  }


  /************imu config*************/

  /* imu低通滤波,目前只支持(0，1)这一档
   *flag    0    1    2    3     4     5     6 
   *acc(ms) 1.94 5.80 7.80 11.80 19.80 35.70 66.96
   *gyr(ms) 0.97 2.9  3.9  5.9   9.9   17.85 33.48 
   */
  //复位reset
  sendCmd(IMU_REG_BASE_ADDR,0x80,0x6B,0,0);
  //等待15ms
  usleep(15*1000);
  //时钟选择
  sendCmd(IMU_REG_BASE_ADDR,0x01,0x6B,0,0);
  //中断设置：磁力bypass
  sendCmd(IMU_REG_BASE_ADDR,0x02,0x37,0,0);
  //中断设置：raw data ready
  sendCmd(IMU_REG_BASE_ADDR,0x01,0x38,0,0);
  //角速度低通滤波
  //sendCmd(IMU_REG_BASE_ADDR,0x04,0x1A,0,0); //9.9ms
  sendCmd(IMU_REG_BASE_ADDR,0x01,0x1A,0,0);   //2.9ms
  //加速度低通滤波
  sendCmd(IMU_REG_BASE_ADDR,0x00,0x1D,0,0);   //1.94ms
  //加速度量程设置：+-8g
  sendCmd(IMU_REG_BASE_ADDR,0x10,0x1c,0,0);
  //角速度量程设置：+2000dps
  sendCmd(IMU_REG_BASE_ADDR,0x18,0x1b,0,0); //+-2000dps
  //sendCmd(IMU_REG_BASE_ADDR,0x08,0x1b,0,0); //+-500dps
  //sendCmd(IMU_REG_BASE_ADDR,0x00,0x1b,0,0); //+-250dps
  //中断设置：raw data
  sendCmd(IMU_REG_BASE_ADDR,0x01,0x38,0,0);
  //输出频率设置：500hz
  //sendCmd(IMU_REG_BASE_ADDR,0x01,0x19,0,0);
  sendCmd(IMU_REG_BASE_ADDR,(1000/imu_freq)-1,0x19,0,0);  //分频系数

  /*****************imu config end***********************/

  return true;
}

  
// 设置曝光模式,1-自动曝光，0-固定曝光
bool device::setExposureMode(int flag)
{
  if(flag)
    sendCmd(SYS_CTRL_REG_BASE_ADDR, 0xC7E1, 0xFC, 0, 0);
  else
    sendCmd(SYS_CTRL_REG_BASE_ADDR, 0xC701, 0xFC, 0, 0);
  return true;
}

// 设置曝光时间
// n-曝光时间(ms)
bool device::setExposureTime(double n)
{
  //曝光时间:exp,单位us
  double exp = n*1000;//单位:us
  double expTime = exp / 19.3333333;
  uint reg3501 = (uint)(expTime / 16);
  uint reg3502 = (uint)((expTime - reg3501) * 16);

  sendCmd(L_SENSOR_REG_BASE_ADDR,reg3501,0x3501,0,0);
  sendCmd(L_SENSOR_REG_BASE_ADDR,reg3502,0x3502,0,0);
  sendCmd(R_SENSOR_REG_BASE_ADDR,reg3501,0x3501,0,0);
  sendCmd(R_SENSOR_REG_BASE_ADDR,reg3502,0x3502,0,0);
  
  return true;
}

// 设置增益(倍数)
bool device::setGain(int n)
{
  //增益
  sendCmd(L_SENSOR_REG_BASE_ADDR,n<<4,0x350B,0,0);
  sendCmd(R_SENSOR_REG_BASE_ADDR,n<<4,0x350B,0,0);
  //sendCmd(L_SENSOR_REG_BASE_ADDR,0x40,0x350B,0,0);
  //sendCmd(R_SENSOR_REG_BASE_ADDR,0x40,0x350B,0,0);
  return true;
}

// 配置imu低通滤波
bool device::setDlfpFlag(int acc_dlfp_flag, int gyr_dlfp_flag)
{
  sendCmd(IMU_REG_BASE_ADDR,gyr_dlfp_flag,0x1A,0,0); //gyro
  sendCmd(IMU_REG_BASE_ADDR,acc_dlfp_flag,0x1D,0,0); //accel
  return true;
}

// 设置imu初始偏置,imu原始数据带有偏置，配置芯片减去偏置量
bool device::setImuOffset(int *acc, int *gyr)
{
  //gyro
  gyr[0] *= 2;  gyr[1] *= 2;  gyr[2] *= 2;//。。。
  
  sendCmd(IMU_REG_BASE_ADDR,gyr[0]>>8,0x13,0,0); //gyr:x high
  sendCmd(IMU_REG_BASE_ADDR,gyr[0]&0x00ff,0x14,0,0); //gyr:x low
  sendCmd(IMU_REG_BASE_ADDR,gyr[1]>>8,0x15,0,0); //gyr:y high
  sendCmd(IMU_REG_BASE_ADDR,gyr[1]&0x00ff,0x16,0,0); //gyr:y low
  sendCmd(IMU_REG_BASE_ADDR,gyr[2]>>8,0x17,0,0); //gyr:z high
  sendCmd(IMU_REG_BASE_ADDR,gyr[2]&0x00ff,0x18,0,0); //gyr:z low

#if 0
  
#define acc_high_bit(x) (x/4 & 0x7F80 >> 7)
#define acc_low_bit(x)  (x/4 & 0x007F << 1)
  
  sendCmd(IMU_REG_BASE_ADDR,acc_high_bit(acc[0]),119,0,0); //acc:x high[7:14]
  sendCmd(IMU_REG_BASE_ADDR,acc_low_bit( acc[0]),120,0,0); //acc:x low[0:6]
  sendCmd(IMU_REG_BASE_ADDR,acc_high_bit(acc[1]),122,0,0); 
  sendCmd(IMU_REG_BASE_ADDR,acc_low_bit( acc[1]),123,0,0); 
  sendCmd(IMU_REG_BASE_ADDR,acc_high_bit(acc[2]),125,0,0); 
  sendCmd(IMU_REG_BASE_ADDR,acc_low_bit( acc[2]),126,0,0); 

  acc[2] /= 4;
  int high = acc[2] & 0x7F80;
  int low = acc[2] & 0x007F;
  //sendCmd(IMU_REG_BASE_ADDR,high>>7,125,0,0); //acc:z high
  //sendCmd(IMU_REG_BASE_ADDR,low<<1,126,0,0); //acc:z low
  printf("-------%x, %x, %x\n",acc[2], high>>7, low<<1);
#endif

  return true;
}

/****************加入硬件同步，接口扩展，end**************/

} /* namespace sn */
