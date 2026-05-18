#include "omv_uart.h"
#include "zf_common_headfile.h"

void OMV_UART_Init(void)
{
    uart_init(OMV_UART, OMV_UART_BAUD, OMV_UART_TX_PIN, OMV_UART_RX_PIN);
}

// ============================================================================
//  底层调试函数：用于精确定位串口问题
// ============================================================================

// 测试1：发送触发字节并等待，不读取任何数据
// 用于确认 OpenMV 是否收到触发信号（通过观察 OpenMV 蓝灯）
uint8 OMV_Test_TriggerOnly(void)
{
    uart_write_byte(OMV_UART, OMV_TRIGGER_BYTE);
    return 1;
}

// 测试2：发送触发后，接收并保存所有数据到 buffer
// 返回实际接收到的字节数（数据保存在 buffer 中供后续分析）
uint16 OMV_Test_TriggerAndSave(uint8 *buffer, uint16 buffer_size, uint16 timeout_ms)
{
    uint16 recv_count = 0;
    uint16 timeout = 0;
    uint8 ch;

    // 清空 RX FIFO
    while (uart_query_byte(OMV_UART, &ch) == 1) {
        // 丢弃已有数据
    }

    // 发送触发字节
    uart_write_byte(OMV_UART, OMV_TRIGGER_BYTE);

    // 等待并接收数据
    while (recv_count < buffer_size && timeout < timeout_ms) {
        if (uart_query_byte(OMV_UART, &ch) != 1) {
            system_delay_ms(1);
            timeout++;
            continue;
        }

				if (ch == '\r' || ch == '\n') {
            continue;
        }
				
        buffer[recv_count] = ch;
        recv_count++;
        timeout = 0;  // 收到数据后重置超时
    }

    return recv_count;
}

// 测试3：发送触发后，逐字节读取并打印（带超时）
// 用于观察 OpenMV 实际发送了什么内容
uint16 OMV_Test_TriggerAndDump(uint8 *buffer, uint16 buffer_size, uint16 timeout_ms)
{
    uint16 recv_count = 0;
    uint16 timeout = 0;

    // 清空 RX FIFO（先读取并丢弃已有数据）
    uint8 ch;
    while (uart_query_byte(OMV_UART, &ch) == 1) {
        // 丢弃
    }

    // 发送触发字节
    uart_write_byte(OMV_UART, OMV_TRIGGER_BYTE);
    printf("  [TX] Trigger 0x01 sent\r\n");

    // 等待并逐字节接收
    while (recv_count < buffer_size && timeout < timeout_ms) {
        if (uart_query_byte(OMV_UART, &ch) != 1) {
            system_delay_ms(1);
            timeout++;
            continue;
        }

        buffer[recv_count] = ch;
        recv_count++;
        timeout = 0;  // 收到数据后重置超时
    }

    return recv_count;
}

// 测试4：不发送触发，仅监听 RX 一段时间
// 用于检测是否有噪声/干扰数据
uint16 OMV_Test_SnoopRx(uint8 *buffer, uint16 buffer_size, uint16 listen_ms)
{
    uint16 recv_count = 0;
    uint16 elapsed = 0;
    uint8 ch;

    while (elapsed < listen_ms && recv_count < buffer_size) {
        if (uart_query_byte(OMV_UART, &ch) == 1) {
            buffer[recv_count] = ch;
            recv_count++;
        }
        system_delay_ms(1);
        elapsed++;
    }

    return recv_count;
}

// 原始函数：发送触发并接收完整地图
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

// ============================================================================
//  OMV_MapToAscii - 已废弃，现为透传函数
// ============================================================================
//  【历史变更】
//  原功能: 将 OpenMV 发送的 '-#*$.' 字符转码为 '0'-'4'
//  现功能: OpenMV 已直接发送 '0'-'4'，本函数直接 memcpy
//
//  保留此函数以兼容旧代码调用点（如 main.c 中的 OMV_MapToAscii()）
//  新代码可直接使用接收缓冲区，无需调用此函数
// ============================================================================
uint8 OMV_MapToAscii(const uint8 *omv_map, uint8 *ascii_map, uint16 length)
{
    uint16 i;

    if (omv_map == NULL || ascii_map == NULL) {
        return 0;
    }

    // OpenMV 现在直接发送 '0'-'4'，无需转码，直接复制
    for (i = 0; i < length; i++) {
        ascii_map[i] = omv_map[i];
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
