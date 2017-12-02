
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
    // The desired total thrust in the z direction in terms of omega^2.
    //
    float fThrustZDir;

    //
    // The required omega^2 for all motors.
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
extern void InitPDController(tPDController * psPD);
extern void ErrorToInput(tPDController * psPD, tCompDCM * psDCM);
extern void PDContUpdatePWM(tPDController * psPD, tPWM * psPWM);
extern float CalcDutyCycle(float battV, float reqOmegaSq);
//extern void ReadDesiredState(tPDController * psPD);
extern void ReadDesiredState(tPDController * psPD, tPWM * psPWM);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // _CONTROLLER_H_
