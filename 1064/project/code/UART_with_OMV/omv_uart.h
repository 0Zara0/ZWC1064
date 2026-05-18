#ifndef OMV_UART_H
#define OMV_UART_H

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

// ============================================================================
//  OpenMV 串口配置（与 main.py 匹配）
// ============================================================================
//  【历史变更】
//  原配置: UART2, 19200bps, 16x12地图, 字符'-#*$.'
//  现配置: UART4, 115200bps, 14x10地图, 字符'0'-'4'
//
//  OpenMV main.py 对应参数:
//    UART_ID = 12 (UART1+2), UART_BAUD = 115200
//    ROI: 77x55, GRID_SIZE = 11 -> 7x5 每ROI
//    4个ROI拼接 -> 14x10 = 140 字节
//    输出字符: '0'(空), '1'(墙), '2'(玩家), '3'(箱子), '4'(目标)
// ============================================================================

#define OMV_UART            UART_2
#define OMV_UART_BAUD       115200
#define OMV_UART_TX_PIN     UART2_TX_B18
#define OMV_UART_RX_PIN     UART2_RX_B19

#define OMV_MAP_WIDTH       14
#define OMV_MAP_HEIGHT      10
#define OMV_MAP_TOTAL       (OMV_MAP_WIDTH * OMV_MAP_HEIGHT)

#define OMV_TRIGGER_BYTE        0x01
#define OMV_FRAME_END_LEN       2
#define OMV_RECV_TIMEOUT_MS     3000

// OpenMV 直接发送数字字符，无需转码
#define OMV_CHAR_EMPTY          '0'
#define OMV_CHAR_WALL           '1'
#define OMV_CHAR_HERO           '2'
#define OMV_CHAR_BOX            '3'
#define OMV_CHAR_DEST           '4'

void OMV_UART_Init(void);
uint8 OMV_TriggerAndReceive(uint8 *map_buffer, uint16 buffer_size);

// ============================================================================
//  底层调试函数声明（用于 Stage 8 问题定位）
// ============================================================================
uint8  OMV_Test_TriggerOnly(void);                                                      // 测试1：仅发送触发
uint16 OMV_Test_TriggerAndSave(uint8 *buffer, uint16 buffer_size, uint16 timeout_ms);   // 测试2：触发+保存数据
uint16 OMV_Test_TriggerAndDump(uint8 *buffer, uint16 buffer_size, uint16 timeout_ms);   // 测试3：触发+逐字节打印
uint16 OMV_Test_SnoopRx(uint8 *buffer, uint16 buffer_size, uint16 listen_ms);           // 测试4：静默监听 RX

// OMV_MapToAscii 已废弃，OpenMV 现在直接发送 '0'-'4'
// 保留声明以兼容旧代码，但内部直接 memcpy
uint8 OMV_MapToAscii(const uint8 *omv_map, uint8 *ascii_map, uint16 length);

uint8 OMV_MapIsValid(const uint8 *omv_map, uint16 length);

#endif
