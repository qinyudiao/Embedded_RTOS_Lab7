 /**
 * @file
 * @author Jonathan Valvano and modified by Jeageun Jung
 *
 * @brief Use CAN0 to communicate on CAN bus
 * Conventions:
 *  - How to use :
 * When Initialize OS,  CAN0_Open() one time.
 * Every ID that want to use as an receive port, CAN0_SetRecv(receive idx) to make interrupt for specific index.
 * CAN0_GetMailwithIdx(data, index) in the point that we want to get data. It automatically block code until data arrive using blocking semaphore.
 * CAN0_SendDatawithIdx(data, index) when we have data to send.
 
 *  - Pin configurations:
 * CAN0Rx PE4 (8) I TTL CAN module 0 receive.
 * CAN0Tx PE5 (8) O TTL CAN module 0 transmit.
 * @copyright Copyright (c) 2019
 *
 */


#ifndef __CAN0_H__
#define __CAN0_H__
#define CAN_BITRATE             1000000

// reverse these IDs on the other microcontroller
#define RCV_ID 2
#define XMT_ID 4
// Returns true if receive data is available
//         false if no receive data ready
//int CAN0_CheckMail(void);


// int CAN0_GetMailNonBlock(uint8_t data[4]);

/**
 * @brief If receive data is ready, gets the data 
 * 
 * @param data   Pointer to get data
 * @param rxidx  Recieve ID which is previously opened
 */
void CAN0_GetMailwithIdx(uint8_t data[4], int rxidx);
//void CAN0_GetMail(uint8_t data[4]);

/**
 * @brief Initialize CAN0 port 
 */
void CAN0_Open(void);

/**
 * @brief Set handler work for specific recieve ID
 * 
 * @param idx  ID to use for recieving data
 */
int CAN0_SetRecv(int idx);

/**
 * @brief  Send 4 bytes of data to other microcontroller 
 * 
 * @param data   Pointer to send data
 * @param idx  ID to send the data
 */
void CAN0_SendDatawithIdx(uint8_t data[4],int idx);



#endif //  __CAN0_H__

