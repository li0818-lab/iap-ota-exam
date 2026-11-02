#include "stm32f4xx.h"
#include "./usart/bsp_debug_usart.h"
#include "my_proto.h"
#include "stdio.h"

//协议帧格式2字节命合，2字节总长度，N字节数据区，4字节保留，2字节crc16校验


int main(void)
{	
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  Debug_USART_Config();//没有这个打印不了
	
	/* 发送一个字符串 */
	Usart_SendString( DEBUG_USART,"这是一个串口中断接收回显实验\n");
//	printf("这是一个串口中断接收回显实验\n");

  while(1)
	{	
		
	}	
}




