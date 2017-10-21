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


#define PWM_FREQUENCY 490

volatile uint32_t ui32Adjust;
volatile uint32_t ui32Load;

//*****************************************************************************
//
// Global instance structure for the HC12 driver.
//
//*****************************************************************************

int main()
{
    volatile uint32_t ui32PWMClock;
    volatile uint32_t ui32Period;
    ui32Adjust = 450;

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);


    //
    // Interrupt timer setup.
    //
    /*
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = (SysCtlClockGet() / 50); // 250 Hz interrupt
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);
    */

    //
    // PWM setup
    //

    // PWM module 1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // PWM generator 0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // PWM generator 1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // switches
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // esc 1
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinConfigure(GPIO_PD0_M1PWM0);

    // esc 2
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    ROM_GPIOPinConfigure(GPIO_PD1_M1PWM1);

    // esc 3
    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    ROM_GPIOPinConfigure(GPIO_PE4_M1PWM2);

    // esc 4
    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
    ROM_GPIOPinConfigure(GPIO_PE5_M1PWM3);

    // SW1 and SW2
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // PWM frequency
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

    // PWM generator 0
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    // PWM generator 1
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);

    // esc 1
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);

    // esc 2
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui32Adjust * ui32Load / 1000);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);

    // esc 3
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32Adjust * ui32Load / 1000);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);

    // esc 4
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui32Adjust * ui32Load / 1000);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

    // enable both generators
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);

    // DEBUGGING
    // timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);

    //
    // UART2 settings.
    //
    InitHC12UART();

    while (1)
    {
        if(buff[3] == 1)
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

        else if(buff[3] == 10)
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

