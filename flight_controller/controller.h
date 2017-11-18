
//*****************************************************************************
//
// controller.h
//
//*****************************************************************************

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "comp_dcm.h"
#include "escpwm.h"

//*****************************************************************************
//
// Controller state.
//
//*****************************************************************************
typedef struct
{
    //
    // The required duty cycle fed into the ESCs.
    //
    float fDutyCycle[4];

    //
    // The required omega^2 of all motors.
    //
    float fOmegaSq[4];

    //
    // Desired angular state.
    //
    float fDesState[3];

    //
    // Current battery voltage.
    //
    float fBatteryV;
}
tPDController;


//*****************************************************************************
//
// Prototypes.
//
//*****************************************************************************
extern void errorToInput(tPDController * psPD, tCompDCM * psDCM);
extern void updatePWM(tPDController * psPD, tPWM * psPWM);
extern float calcDutyCycle(float battV, float reqRPM);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // _CONTROLLER_H_
