#ifndef _ESCPWMH_
#define _ESCPWMH_

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
#include "driverlib/uart.h"
#include "driverlib/pwm.h"

//*****************************************************************************
//
// ESCs state.
//
//*****************************************************************************
typedef struct
{
    //
    //
    //
    uint32_t ui32PWMClock;

    //
    //
    //
    uint32_t ui32Period;

    //
    //
    //
    uint32_t ui32Load;

    //
    //
    //
    float dutyCycles[4];
}
tPWM;

#define PWM_FREQUENCY 490

void InitPWM(tPWM * psPWM);
void SetMotorPulseWidth(uint8_t motorNumber, float dutyCycle, tPWM * psPWM);

#endif
