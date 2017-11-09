
//*****************************************************************************
//
// controller.h
//
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

//*****************************************************************************
//
// Controller state.
//
//*****************************************************************************
typedef struct
{
    //
    //
    //
    float fDutyCycle[4];

    //
    //
    //
    float fOmegaSq[4];

    //
    // desired
    //
    float fDesState[3];
}
tPDController;

//*****************************************************************************
//
// Prototypes.
//
//*****************************************************************************
extern void errorToInput(tPDController * psPD, tCompDCM * psDCM);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // _CONTROLLER_H_
