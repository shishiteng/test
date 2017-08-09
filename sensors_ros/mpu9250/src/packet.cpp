#include <string.h>
#include <stdio.h>

#include "packet.h"


#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CH_OK
#define CH_OK   (0)
#endif

#ifndef CH_ERR
#define CH_ERR  (1)
#endif

//#define uint16_t unsigned short 

uint8_t ItemsLenTbl[13][2] = 
{
    {kItemID,               1},
    {kItemIPAdress,         4},
    {kItemAccRaw,           3*2},
    {kItemAccRawFiltered ,  3*2},
    {kItemGyoRaw ,          3*2},
    {kItemGyoRawFiltered ,  3*2},
    {kItemMagRaw ,          3*2},
    {kItemMagRawFiltered ,  3*2},
    {kItemAtdE ,            3*2},
    {kItemAtdQ ,            4*4},
    {kItemTemp ,            1*4},
    {kItemPressure ,        1*4},
    {kItemEnd ,             0x00},
};

static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currectCrc = crc;
}


uint32_t Packet_BeginEncode(Packet_t *pkt)
{
    pkt->ofs = 6;
    memset(&pkt->buf[0], 0, sizeof(pkt->buf));
    pkt->buf[0] = 0x5A; /* header */
    pkt->buf[1] = 0xA5; /* data packet */
    return CH_OK;
}

uint32_t Packet_AddData(Packet_t *pkt, uint8_t *buf, uint8_t len)
{
    /* add item content into buffer */
    memcpy((pkt->buf + pkt->ofs), buf, len);
    pkt->ofs += len;

    return CH_OK;
}

uint32_t Packet_GetEncodedPacket(Packet_t *pkt)
{

    pkt->payload_len = pkt->ofs -6;
    pkt->len = pkt->ofs;
    
    pkt->buf[2] = (pkt->payload_len & 0x00FF)>>0;
    pkt->buf[3] = (pkt->payload_len & 0xFF00)>>8;
    
    /* crc */
    uint16_t crc;
    crc = 0;
    crc16_update(&crc, &pkt->buf[0], 4);
    crc16_update(&crc, &pkt->buf[6], pkt->payload_len);
    pkt->buf[4] = (crc & 0x00FF)>>0;
    pkt->buf[5] = (crc & 0xFF00)>>8;

    return CH_OK;
}




enum status
{
    kStatus_Idle,
    kStatus_Cmd,
    kStatus_LenLow,
    kStatus_LenHigh,
    kStatus_CRCLow,
    kStatus_CRCHigh,
    kStatus_Data,
};

/* function pointer */
static OnDataReceivedEvent EventHandler;
Packet_t p;
static Packet_t *RxPkt = &p;


 /**
 * @brief  初始化姿态解码模块
 * @note   完成初始化一个引脚配置
 * @param  pkt 接收包指针
 * @param  接收成功回调函数
 * @code

 *      void OnDataReceived(Packet_t *pkt)
 *      {
 *          pkt->buf 为数据 pkt->payload_len 为接收到的字节长度 
 *      }
 *
 *      Packet_t pkt;
 *      Packet_DecodeInit(&pkt, OnDataReceived);
 * @endcode
 * @retval None
 */
void Packet_DecodeInit(Packet_t *pkt, OnDataReceivedEvent Func)
{
    EventHandler = Func;
    RxPkt = pkt;
}

 /**
 * @brief  接收IMU数据
 * @note   在串口接收中断中调用此函数
 * @param  c 串口数据
 * @retval CH_OK
 */
Packet_t * Packet_Decode(uint8_t c)
{
    static uint16_t CRCReceived = 0;
    static uint16_t CRCCalculated = 0;
    static uint8_t status = kStatus_Idle;
    static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};
                    
    switch(status)
    {
        case kStatus_Idle:
            if(c == 0x5A)
                status = kStatus_Cmd;
            break;
        case kStatus_Cmd:
            if(c == 0xA5)
                status = kStatus_LenLow;
            break;
        case kStatus_LenLow:
            RxPkt->payload_len = c;
            crc_header[2] = c;
            status = kStatus_LenHigh;
            break;
        case kStatus_LenHigh:
            RxPkt->payload_len |= (c<<8);
            crc_header[3] = c;
            status = kStatus_CRCLow;
            break;
        case kStatus_CRCLow:
            CRCReceived = c;
            status = kStatus_CRCHigh;
            break;
        case kStatus_CRCHigh:
            CRCReceived |= (c<<8);
            RxPkt->ofs = 0;
            CRCCalculated = 0;
            status = kStatus_Data;
            break;
        case kStatus_Data:
            RxPkt->buf[RxPkt->ofs++] = c;
            if(RxPkt->ofs >= RxPkt->payload_len)
            {
                /* calculate CRC */
                crc16_update(&CRCCalculated, crc_header, 4);
                crc16_update(&CRCCalculated, RxPkt->buf, RxPkt->ofs);
                
                /* CRC match */
                if(CRCCalculated == CRCReceived)
                {
                    if(EventHandler != NULL)
                    {
                        EventHandler(RxPkt);
                    }
                }

                status = kStatus_Idle;
				return RxPkt;
            }
            break;
        default:
            break;
    }
    return NULL;
}

