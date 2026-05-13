#ifndef OMV_UART_H
#define OMV_UART_H

#include "zf_common_typedef.h"

#define OMV_UART            UART_2
#define OMV_UART_BAUD       19200
#define OMV_UART_TX_PIN     UART2_TX_B18
#define OMV_UART_RX_PIN     UART2_RX_B19

#define OMV_MAP_WIDTH   16
#define OMV_MAP_HEIGHT  12
#define OMV_MAP_TOTAL   (OMV_MAP_WIDTH * OMV_MAP_HEIGHT)

#define OMV_TRIGGER_BYTE        0x01
#define OMV_FRAME_END_LEN       2
#define OMV_RECV_TIMEOUT_MS     500

#define OMV_CHAR_EMPTY          '-'
#define OMV_CHAR_WALL           '#'
#define OMV_CHAR_HERO           '*'
#define OMV_CHAR_BOX            '$'
#define OMV_CHAR_DEST           '.'

void OMV_UART_Init(void);
uint8 OMV_TriggerAndReceive(uint8 *map_buffer, uint16 buffer_size);
uint8 OMV_MapToAscii(const uint8 *omv_map, uint8 *ascii_map, uint16 length);
uint8 OMV_MapIsValid(const uint8 *omv_map, uint16 length);

#endif
