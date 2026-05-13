#include "omv_uart.h"
#include "zf_common_headfile.h"

void OMV_UART_Init(void)
{
    uart_init(OMV_UART, OMV_UART_BAUD, OMV_UART_TX_PIN, OMV_UART_RX_PIN);
}

uint8 OMV_TriggerAndReceive(uint8 *map_buffer, uint16 buffer_size)
{
    uint16 recv_count;
    uint16 timeout;

    if (map_buffer == NULL || buffer_size < OMV_MAP_TOTAL) {
        return 0;
    }

    uart_write_byte(OMV_UART, OMV_TRIGGER_BYTE);

    recv_count = 0;
    timeout = 0;
    while (recv_count < buffer_size && timeout < OMV_RECV_TIMEOUT_MS) {
        uint8 ch;

        if (uart_query_byte(OMV_UART, &ch) != 1) {
            system_delay_ms(1);
            timeout++;
            continue;
        }

        if (ch == 0x0D) {
            uint8 dummy;
            uart_query_byte(OMV_UART, &dummy);
            break;
        }

        map_buffer[recv_count] = ch;
        recv_count++;

        timeout = 0;
    }

    return recv_count;
}

uint8 OMV_MapToAscii(const uint8 *omv_map, uint8 *ascii_map, uint16 length)
{
    uint16 i;

    if (omv_map == NULL || ascii_map == NULL) {
        return 0;
    }

    for (i = 0; i < length; i++) {
        switch (omv_map[i]) {
        case OMV_CHAR_EMPTY:  ascii_map[i] = '0'; break;
        case OMV_CHAR_WALL:   ascii_map[i] = '1'; break;
        case OMV_CHAR_HERO:   ascii_map[i] = '2'; break;
        case OMV_CHAR_BOX:    ascii_map[i] = '3'; break;
        case OMV_CHAR_DEST:   ascii_map[i] = '4'; break;
        default:              ascii_map[i] = '0'; break;
        }
    }

    return 1;
}

uint8 OMV_MapIsValid(const uint8 *omv_map, uint16 length)
{
    uint16 i;

    if (omv_map == NULL) {
        return 0;
    }

    for (i = 0; i < length; i++) {
        uint8 ch = omv_map[i];
        if (ch != OMV_CHAR_EMPTY &&
            ch != OMV_CHAR_WALL  &&
            ch != OMV_CHAR_HERO  &&
            ch != OMV_CHAR_BOX   &&
            ch != OMV_CHAR_DEST) {
            return 0;
        }
    }

    return 1;
}
