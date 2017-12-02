#include "escpwm.h"

//*****************************************************************************
//
// Initializes the PWM modules.
//
//*****************************************************************************
void
InitPWM(tPWM * psPWM)
{
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
    psPWM->ui32PWMClock = SysCtlClockGet() / 64;
    psPWM->ui32Load = (psPWM->ui32PWMClock / PWM_FREQUENCY) - 1;

    // PWM generator 0
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, psPWM->ui32Load);

    // PWM generator 1
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, psPWM->ui32Load);

    float initialDutyCycle = 0.99;

    // esc 1
    psPWM->dutyCycles[0] = initialDutyCycle;
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (1 - initialDutyCycle) * psPWM->ui32Load);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);

    // esc 2
    psPWM->dutyCycles[1] = initialDutyCycle;
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, (1 - initialDutyCycle) * psPWM->ui32Load);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);

    // esc 3
    psPWM->dutyCycles[2] = initialDutyCycle;
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, (1 - initialDutyCycle) * psPWM->ui32Load);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);

    // esc 4
    psPWM->dutyCycles[3] = initialDutyCycle;
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, (1 - initialDutyCycle) * psPWM->ui32Load);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

    // enable both generators
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}


//*****************************************************************************
//
// Sets the duty cycle of the PWM that is fed into the ESCs.
//
//*****************************************************************************
void
SetMotorPulseWidth(uint8_t motorNumber, float dutyCycle, tPWM * psPWM)
{
    switch (motorNumber)
    {
    case 0:
        psPWM->dutyCycles[0] = dutyCycle;
        ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0,
                             dutyCycle * psPWM->ui32Load);
        break;
    case 1:
        psPWM->dutyCycles[1] = dutyCycle;
        ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
                             dutyCycle * psPWM->ui32Load);
        break;
    case 2:
        psPWM->dutyCycles[2] = dutyCycle;
        ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2,
                             dutyCycle * psPWM->ui32Load);
        break;
    case 3:
        psPWM->dutyCycles[3] = dutyCycle;
        ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,
                             dutyCycle * psPWM->ui32Load);
        break;
    }
}


//*****************************************************************************
//
// On startup, the ESCs throttle is calibrated.
//
//*****************************************************************************
void
CalibrateThrottle(tPWM * psPWM)
{
    SysCtlDelay(3 * SysCtlClockGet() / 3);  // 3 sec delay
    float dcycle = 0.5;
    SetMotorPulseWidth(0, 1.0 - dcycle, psPWM);
    SetMotorPulseWidth(1, 1.0 - dcycle, psPWM);
    SetMotorPulseWidth(2, 1.0 - dcycle, psPWM);
    SetMotorPulseWidth(3, 1.0 - dcycle, psPWM);
    SysCtlDelay(3 * SysCtlClockGet() / 3);  // 3 sec delay
}










