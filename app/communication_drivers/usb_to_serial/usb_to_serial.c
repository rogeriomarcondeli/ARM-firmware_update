/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file usb_to_serial.c
 * @brief USB to Serial module.
 *
 * @author joao.rosa
 *
 * @date 23/01/2017
 *
 */

#include <stdint.h>

#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"

#include "communication_drivers/epi/sdram_mem.h"
#include "board_drivers/hardware_def.h"

#include "usb_to_serial.h"

void isr_usb_to_serial(void)
{
    unsigned long ulStatus;

    // Get the interrrupt status.
    ulStatus = UARTIntStatus(FT230_UART_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(FT230_UART_BASE, ulStatus);

    //if(0x00000010 == ulStatus)
    if(0x00000040 == ulStatus)
    {
    	while(UARTCharsAvail(FT230_UART_BASE))
		{
			UARTCharGet(FT230_UART_BASE);

		}

    	if(sdram_read_write())
    	{
    		UARTCharPutNonBlocking(FT230_UART_BASE, 'E');
			UARTCharPutNonBlocking(FT230_UART_BASE, 'R');
			UARTCharPutNonBlocking(FT230_UART_BASE, ' ');
    	}
    	else
    	{
    		UARTCharPutNonBlocking(FT230_UART_BASE, 'O');
			UARTCharPutNonBlocking(FT230_UART_BASE, 'K');
			UARTCharPutNonBlocking(FT230_UART_BASE, ' ');
    	}

    }

}

void init_usb_to_serial(void)
{

	// Usb serial configuration, operation mode 8-N-1
	UARTConfigSetExpClk(FT230_UART_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 1000000,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
						UART_CONFIG_PAR_NONE));

	UARTEnable(FT230_UART_BASE);

	UARTFIFOEnable(FT230_UART_BASE);

	//Habilita interrup��o pela UART0 (RS-485)
	IntRegister(FT230_INT, isr_usb_to_serial);
	UARTIntEnable(FT230_UART_BASE, UART_INT_RX | UART_INT_RT);

	//Seta n�veis de prioridade entre as interrup��es
	IntPrioritySet(FT230_INT, 3);

	IntEnable(FT230_INT);

}
