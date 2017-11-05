#include "escpwm.h"

volatile uint32_t ui32Load;


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
InitPWM(void)
{
    volatile uint32_t ui32PWMClock;
    volatile uint32_t ui32Period;

    // PWM module 1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // PWM generator 0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // PWM generator 1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

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
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 500 * ui32Load / 1000);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);

    // esc 2
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 500 * ui32Load / 1000);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);

    // esc 3
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 500 * ui32Load / 1000);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);

    // esc 4
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 500 * ui32Load / 1000);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

    // enable both generators
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
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
SetMotorPulseWidth(uint8_t motorNumber, float dutyCycle)
{
    switch (motorNumber)
    {
    case 0:
        ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, dutyCycle * ui32Load);
        break;
    case 1:
        ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, dutyCycle * ui32Load);
        break;
    case 2:
        ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyCycle * ui32Load);
        break;
    case 3:
        ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyCycle * ui32Load);
        break;
    }
}










