#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/uart.h"
#include "hc12.h"
#include "buffer.h"


//*****************************************************************************
//
//!
//!
//! \param
//! \param
//!
//! ???
//!
//! \return
//
//*****************************************************************************
void
UART2IntHandler()
{
    UARTIntClear(UART2_BASE, UART_INT_RX);
    int i;
    for (i = 0; i < DATA_LENGTH; i++)
    {
        char data = (char)(UART2_DR_R&0xFF);
        WriteByteToBuffer(data);
    }
}


//*****************************************************************************
//
//!
//!
//! \param
//! \param
//!
//! ???
//!
//! \return
//
//*****************************************************************************
void
InitHC12UART()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);

    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable();
    IntEnable(INT_UART2);
    UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX7_8, UART_FIFO_RX7_8);
    UARTIntEnable(UART2_BASE, UART_INT_RX);
}




