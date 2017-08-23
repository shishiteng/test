#ifndef __PACKET_H__
#define __PACKET_H__

#include <stdint.h>
//#include <stdbool.h>

#define MAX_PACKET_LEN          (128)

typedef enum 
{
    kItemID =                   0x90,   /* user programed ID    size: 1 */
    kItemIPAdress =             0x92,   /* ip address           size: 4 */
    kItemAccRaw =               0xA0,   /* raw acc              size: 3x2 */
    kItemAccRawFiltered =       0xA1,   
    kItemAccDynamic =           0xA2,   
    kItemGyoRaw =               0xB0,   /* raw gyro             size: 3x2 */
    kItemGyoRawFiltered =       0xB1,   
    kItemMagRaw =               0xC0,   /* raw mag              size: 3x2 */
    kItemMagRawFiltered =       0xC1,   
    kItemAtdE =                 0xD0,   /* eular angle          size:3x2 */
    kItemAtdQ =                 0xD1,   /* att q,               size:4x4 */
    kItemTemp =                 0xE0,   
    kItemPressure =             0xF0,   /* pressure             size:1x4 */
    kItemEnd =                  0xFF,   
}ItemID_t;


typedef struct
{
    uint32_t ofs;
    uint8_t buf[MAX_PACKET_LEN];
    uint16_t payload_len;
    uint16_t len;
}Packet_t;


/* packet Tx API */
uint32_t Packet_AddData(Packet_t *pkt, uint8_t *buf, uint8_t len);
uint32_t Packet_BeginEncode(Packet_t *pkt);
uint32_t Packet_GetEncodedPacket(Packet_t *pkt);

/* packet Rx API */
typedef void (*OnDataReceivedEvent)(Packet_t *pkt);

void Packet_DecodeInit(Packet_t *pkt, OnDataReceivedEvent rx_handler);
Packet_t * Packet_Decode(uint8_t c);


#endif

