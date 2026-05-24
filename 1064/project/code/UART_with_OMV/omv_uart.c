#include "omv_uart.h"
#include "zf_common_headfile.h"



/*2026 5 22 简介

		具体每个串口对应的引脚看.h文件，UART1是用来printf的;

	与openmv通信的相关函数，使用UART4,UART5,波特率固定115200，串口4设定140个字节数据，串口5设定1个字节数据，能收能发，使用轮询收发模式，
	在主函数里面使用阻塞模式收发，示例："while(!OMV_UART_MAP_RECEIVE(receive)){}"没接收够140个永远卡死，串口5缓冲区只有1个字节，
	接收到一个字节就结束阻塞模式收发，所以一次只能发一个字节，否则会接收不到或者产生混乱;
*/

/*地图串口初始化函数，对应UART4
	参数：void
	返回值：void
*/
void OMV_UART_MAP_Init(void)
{
    uart_init(OMV_UART_MAP, OMV_UART_BAUD, OMV_UART_MAP_TX_PIN, OMV_UART_MAP_RX_PIN);
		uart_rx_interrupt (OMV_UART_MAP,0);
	return;
}

/*标号串口初始化函数，对应UART5
	参数：void
	返回值：void
*/	
void OMV_UART_SIGN_Init(void)
{
    uart_init(OMV_UART_SIGN, OMV_UART_BAUD, OMV_UART_SIGN_TX_PIN, OMV_UART_SIGN_RX_PIN);
	  uart_rx_interrupt (OMV_UART_SIGN,0);
	return;
}

/*地图串口接收数据函数(UART4)
	参数：接收的数组
返回值：uint8_t 1表示成功，0表示未成功
备注：阻塞接收，只有接收完140个才会认为接收成功，否则认为接收不成功
*/

uint8_t OMV_UART_MAP_RECEIVE(uint8_t receive[])
{
	uint8_t dat;
	static uint8_t index=0;
		while(uart_query_byte(OMV_UART_MAP,&dat))
	{
		receive[index]=dat;
		index++;
		if(index>=OMV_MAP_TOTAL)
		{	
			index=0;
			return 1;
		}
		}	
	return 0;
}
/*地图串口发送字节函数(UART4)
	参数：发送的字节
	返回值：void
*/
void OMV_UART_MAP_SEND(uint8_t senddata)
{
		uart_write_byte (OMV_UART_MAP,senddata);

}

/*标记串口接收字符(UART5)
	参数：接收的变量
返回值：uint8_t 1表示成功，0表示未成功
备注：只接收一个字节
*/
uint8_t OMV_UART_SIGN_RECEIVE(uint8_t receive[])
{
	uint8_t dat;
	static uint8_t index=0;
		while(uart_query_byte(OMV_UART_SIGN,&dat))
	{
		receive[index]=dat;
		index++;
		if(index>=1)
		{	
			index=0;
			return 1;
		}
		}	
	return 0;
}

/*地图串口发送字节函数(UART5)
	参数：发送的字节
	返回值：void
*/
void OMV_UART_SIGN_SEND(uint8_t senddata)
{
		uart_write_byte (OMV_UART_SIGN,senddata);

}
