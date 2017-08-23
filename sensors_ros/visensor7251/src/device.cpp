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
// 指令字节数
#define CMDBYTES 16
#define HEADBYTES CMDBYTES

#define IMU_BYTES 48
static bool bStreaming = false;


mutex m_buf;
queue<imuData> imu_buf;
condition_variable con;

  
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
        m_single_rowBytes = m_width * 2;
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

    m_remainCount = 0;
    m_cacheInBuf = new uchar[ReadDataBytes * 2];
	
    for(int i=0; i < 50; ++i){
    pthread_spin_init(&m_imulock[i], 0);
    m_imuData_ptr[i] = new imuData;
    m_exitNewImu[i] = false;
    }
    m_imuData_cache_ptr = new imuData;

    m_imuIndex = 0;
    m_imuSaveIndex = 0;
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
    sendCmd(0x4, 0x0, 0x0, 0x2);

    int waitNum = 100;
    while (bStreaming && waitNum-- > 0)
    {
        waitKey(30);
    }
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

bool sn::device::QueryFrame(cv::Mat &leftImg, cv::Mat &rightImg, uint64 &imgTimeStamp)
{
    if (m_existNewImg)
    {
        pthread_spin_lock(&m_imglock);
        swapPtr((void **)&leftImg.data, (void **)&m_leftImage_ptr);
        swapPtr((void **)&rightImg.data, (void **)&m_rightImage_ptr);
        imgTimeStamp = m_imgTimeStamp;
        m_existNewImg = false;
        pthread_spin_unlock(&m_imglock);
        return true;
    }

    return false;
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

bool device::readIMU(imuData *imudata){
#if 0
  if(m_exitNewImu[m_imuIndex]){
    pthread_spin_lock(&m_imulock[m_imuIndex]);
    memcpy(imudata,m_imuData_ptr[m_imuIndex],IMU_BYTES);
    m_exitNewImu[m_imuIndex] = false;
    int lastindex = m_imuIndex;
    m_imuIndex = (m_imuIndex+1 > 9) ? 0 : (m_imuIndex+1);
    //cout<<"=========="<<m_imuIndex<<endl;
    //usleep(50);
    pthread_spin_unlock(&m_imulock[lastindex]);

        
    //---
    return true;
  }   

  return false;
  
#else
  imuData data;
  if(!imu_buf.empty()) {
    data = imu_buf.front();
    memcpy(imudata,&data,48);
    m_buf.lock();
    imu_buf.pop();
    m_buf.unlock();
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

bool device::sendCmd(uint Base_addr, uint Data, uint offset /*= 0x0*/, uint cmd /*= 0x0*/, uint id /*= 0x1234*/)
{
    // 偏移量
    Base_addr += offset;

    long sendbytes = CMDBYTES;
    uchar *buf = new uchar[sendbytes];

    memset(buf, 0, CMDBYTES);
    unsigned int *data = (unsigned int *)buf;
    uint type_len = 0xf;
    data[0] = Data << 16;
    data[1] = (Base_addr << 16) | (Data >> 16);
    data[2] = Base_addr >> 16;
    data[3] = (id << 16) | cmd | type_len << 4;

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
    for (int i = 0; i < m_height; ++i)
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

static void *XferLoop(void *args)
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
                if (cur + HEADBYTES + m_sbs_rowBytes + IMU_BYTES > iBytes)
                {
                    end = true;
		    //cout<<"end"<<endl;
                    break;
                }

                // 内存拷贝imu数据
                memcpy(m_imuData_cache_ptr,(uchar*)headData + HEADBYTES + m_sbs_rowBytes,IMU_BYTES);

                unsigned int row = headData[2] & 0xffff;
                unsigned int frameNum = (headData[3] >> 8) & 0xff;
                //cout << "row:" << row << endl;
                writeOneRowImg(headData, row, frameNum);

#if 0
                if(m_imuData_cache_ptr->imu_status == 0x80){
//printf("%I64d %I64d\n",m_imuData_cache_ptr->imu_timestamp,m_imuData_cache_ptr->accel_z);
                      pthread_spin_lock(&m_imulock[m_imuSaveIndex]);
		      /*if (200000 != (m_imuData_cache_ptr->imu_timestamp - m_imuData_ptr->imu_timestamp))
			cout<<" ============="<<endl;*/
                      memcpy(m_imuData_ptr[m_imuSaveIndex],m_imuData_cache_ptr,IMU_BYTES);
                      m_exitNewImu[m_imuSaveIndex] = true;
		      int lastindex = m_imuSaveIndex;
		      m_imuSaveIndex = (m_imuSaveIndex+1 > 9) ? 0: (m_imuSaveIndex+1);
		      //cerr<<"++++++++"<<m_imuSaveIndex<<endl;
		      pthread_spin_unlock(&m_imulock[lastindex]);
		      //usleep(500);
                      //m_imuSaveIndex = (m_imuSaveIndex+1 > 9) ? 0: (m_imuSaveIndex+1);
		      //cout<<m_imuSaveIndex

                }
#else
		if(m_imuData_cache_ptr->imu_status == 0x80){
		  imuData imudata;
		  memcpy(&imudata,m_imuData_cache_ptr,IMU_BYTES);
		  m_buf.lock();
		  imu_buf.push(imudata);
		  m_buf.unlock();
		}
#endif

                cur += HEADBYTES + m_sbs_rowBytes + IMU_BYTES;
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
		case 15:
		{
		    //空包
		    unsigned int bytesSize = ((headData[2] >> 16) & 0x0fff)*16;
                    if(cur+ HEADBYTES + bytesSize + IMU_BYTES > iBytes)
                    {
                    end = true;
                    break;
                    }

                    cur += HEADBYTES + bytesSize + IMU_BYTES;
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
	if(row == 0){
		m_imgTimeStamp_cache = m_imuData_cache_ptr->img_timestamp;
                m_img_frame = frameNum;
	}
	
        memcpy(m_leftImage_cache_ptr + row * m_single_rowBytes, (unsigned char *)data + HEADBYTES, m_single_rowBytes);
        memcpy(m_rightImage_cache_ptr + row * m_single_rowBytes, (unsigned char *)data + HEADBYTES + m_single_rowBytes, m_single_rowBytes);

        if (row >= m_height - 1)
        {
            if(frameNum == m_img_frame){
		m_imgTimeStamp = m_imgTimeStamp_cache;
	    }else{
		m_imgTimeStamp = 0;
		}

            pthread_spin_lock(&m_imglock);
            swapPtr((void **)&m_leftImage_cache_ptr, (void **)&m_leftImage_ptr);
            swapPtr((void **)&m_rightImage_cache_ptr, (void **)&m_rightImage_ptr);
            m_existNewImg = true;
            pthread_spin_unlock(&m_imglock);
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

} /* namespace sn */
