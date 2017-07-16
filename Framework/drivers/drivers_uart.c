#include "drivers_uart.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartupper_low.h"

#include "peripheral_define.h"
#include "utilities_debug.h"
#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	//fw_printfln("HAL_UART_RxCpltCallback");
	if(UartHandle == &RC_UART){
		//rcUartRxCpltCallback();
		fw_Warning();
	}else if(UartHandle == &CTRL_UART){
		ctrlUartRxCpltCallback();
		//fw_Warning();
	}
}   
