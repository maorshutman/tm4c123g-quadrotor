#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/uart.h"
#include "hc12.h"
#include "buffer.h"
#include "escpwm.h"



//*****************************************************************************
//
// Global instance structure for the HC12 driver.
//
//*****************************************************************************

int main()
{
    volatile uint32_t ui32Adjust;

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // DEBUGGING
    // timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);

    //
    // Initialize PWM
    //
    InitPWM();

    //
    // UART2 settings.
    //
    InitHC12UART();

    while (1)
    {
        if(buff[3] == 1 && buff[4] == 2 && buff[5] == 3 &&
           buff[6] == 4 && buff[7] == 5 && buff[8] == 6)
        {
            ui32Adjust -= 2;
            if (ui32Adjust < 500)
            {
                ui32Adjust = 500;
            }
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui32Adjust * ui32Load / 1000);
        }

        else if(buff[3] == 10 && buff[4] == 20 && buff[5] == 30 &&
                buff[6] == 40 && buff[7] == 50 && buff[8] == 60)
        {
            ui32Adjust += 2;
            if (ui32Adjust > 900)
            {
                ui32Adjust = 900;
            }
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui32Adjust * ui32Load / 1000);
        }

        // GPIO_PORTB_DATA_R &= ~0x01;

        SysCtlDelay(100000); // every 72 msec
    }

    return 0;
}

void Timer0IntHandler(void)
{
}
