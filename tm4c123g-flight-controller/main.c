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
// Main application entry point.
//
//*****************************************************************************
int main()
{
    volatile uint32_t ui32Adjust = 500;

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // DEBUGGING
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);

    //
    // Initialize PWM.
    //
    InitPWM();

    //
    // Initialize UART for radio receiver.
    //
    InitHC12UART();

    while (1)
    {
        if(buff[3] == 1 && buff[4] == 2 && buff[5] == 3 &&
           buff[6] == 4 && buff[7] == 5 && buff[8] == 6)
        {
            ui32Adjust -= 1;
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
            ui32Adjust += 1;
            if (ui32Adjust > 990)
            {
                ui32Adjust = 990;
            }
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui32Adjust * ui32Load / 1000);
        }

        SysCtlDelay(100000); // every 72 msec
    }

    return 0;
}
