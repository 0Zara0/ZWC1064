#ifndef OMV_UART_H
#define OMV_UART_H

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

// ============================================================================
//  OpenMV 串口配置
// ============================================================================
//  串口配置: UART4, 115200bps,缓冲区140字节，	接收地图‘0’‘1’‘2’‘3’‘4’‘5’
//  串口配置: UART5, 115200bps,缓冲区1字节，		接收标号‘A-Z’‘a-z’
// ============================================================================

#define OMV_UART_MAP        UART_4
#define OMV_UART_BAUD       115200
#define OMV_UART_MAP_TX_PIN     UART4_TX_C16
#define OMV_UART_MAP_RX_PIN     UART4_RX_C17

#define OMV_MAP_WIDTH       14
#define OMV_MAP_HEIGHT      10
#define OMV_MAP_TOTAL       (OMV_MAP_WIDTH * OMV_MAP_HEIGHT)

#define OMV_UART_SIGN        UART_5
#define OMV_UART_BAUD       115200
#define OMV_UART_SIGN_TX_PIN     UART5_TX_C28
#define OMV_UART_SIGN_RX_PIN     UART5_RX_C29

#define OMV_TRIGGER_BYTE        0x01
#define OMV_RECV_TIMEOUT_MS     3000


void OMV_UART_MAP_Init(void);
void OMV_UART_SIGN_Init(void);
uint8_t OMV_UART_MAP_RECEIVE(uint8_t receive[]);
void OMV_UART_MAP_SEND(uint8_t senddata);
uint8_t OMV_UART_SIGN_RECEIVE(uint8_t receive[]);
void OMV_UART_SIGN_SEND(uint8_t senddata);

#endif
