
//*****************************************************************************
//
// controller.c - Complementary filter algorithm on a Direction Cosine Matrix for
//              fusing sensor data from an accelerometer, gyroscope, and
//              magnetometer.
//
//
//*****************************************************************************

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/debug.h"
#include "controller.h"

//*****************************************************************************
//
// If M_PI has not been defined by the system headers, define it here.
//
//*****************************************************************************
#ifndef M_PI
#define M_PI                    3.14159265358979323846
#endif

//*****************************************************************************
//
// Quadrotor parameters.
//
//*****************************************************************************

#define m                   0.7 // kg
#define g                   9.81 // m * s^-2
#define b                   5.0e-6  // N * Hz^-2, taken form Mellinger paper
#define BODY_MASS           0.66 // kg
#define MOTOR_MASS          0.047 // kg
#define ARM_LENGTH          0.25 // m
#define ARM_MASS            0.043 // kg
#define I_XX                0.00884 // kg * m^2
#define I_YY                0.00884 // kg * m^2
#define I_ZZ                0.0165 // kg * m^2
#define MAX_MOTOR_GAMMA     pow(2.0 * M_PI * 6360 / 60, 2) // max motor rpm
#define K                   (0.62 * 9.81) / pow(2 * M_PI * 6360 / 60, 2) // N * Hz^-2

//*****************************************************************************
//
// PD parameters.
//
//*****************************************************************************
#define KD                  3.0
#define KP                  4.0

//*****************************************************************************
//
// ???
//
//*****************************************************************************
void
errorToInput(tPDController * psPD, tCompDCM * psDCM)
{
    float totalThrust = m * g /
            (K * cosf(psDCM->fEuler[0]) * cosf(psDCM->fEuler[1]));

    //
    // PD error.
    //
    float eAlpha = -KP * (psPD->fDesState[2] - psDCM->fEuler[2]) +
            -KD * (0.0 - psDCM->pfGyro[2]);
    float eBeta = -KP * (psPD->fDesState[1] - psDCM->fEuler[1]) +
            -KD * (0.0 - psDCM->pfGyro[1]);
    float eGamma = -KP * (psPD->fDesState[0] - psDCM->fEuler[0]) +
            -KD * (0.0 - psDCM->pfGyro[0]);

    //
    // Torques up to constants.
    //
    float torqGamma = -I_XX * eGamma * 1.41421356237 / (ARM_LENGTH * K);
    float torqBeta = -I_YY * eBeta * 1.41421356237 / (ARM_LENGTH * K);
    float torqAlpha = -I_ZZ * eAlpha / b;

    //
    // Updates omega^2 for all motors.
    //
    float omegaSq[4];
    omegaSq[0] = (totalThrust + torqGamma - torqBeta - torqAlpha) / 4.0;
    omegaSq[1] = (totalThrust - torqGamma - torqBeta + torqAlpha) / 4.0;
    omegaSq[2] = (totalThrust - torqGamma + torqBeta - torqAlpha) / 4.0;
    omegaSq[3] = (totalThrust + torqGamma + torqBeta + torqAlpha) / 4.0;

    // limit motor angular velocity
    if (omegaSq[0] > MAX_MOTOR_GAMMA)    omegaSq[0] = MAX_MOTOR_GAMMA;
    if (omegaSq[1] > MAX_MOTOR_GAMMA)    omegaSq[1] = MAX_MOTOR_GAMMA;
    if (omegaSq[2] > MAX_MOTOR_GAMMA)    omegaSq[2] = MAX_MOTOR_GAMMA;
    if (omegaSq[3] > MAX_MOTOR_GAMMA)    omegaSq[3] = MAX_MOTOR_GAMMA;

    psPD->fOmegaSq[0] = omegaSq[0];
    psPD->fOmegaSq[1] = omegaSq[1];
    psPD->fOmegaSq[2] = omegaSq[2];
    psPD->fOmegaSq[3] = omegaSq[3];
}

//*****************************************************************************
//
// Update motor PWM duty cycles.
//
//*****************************************************************************
void
updatePWM(tPWM * psPWM)
{
//    SetMotorPulseWidth(0, , psPWM);
//    SetMotorPulseWidth(1, , psPWM);
//    SetMotorPulseWidth(2, , psPWM);
//    SetMotorPulseWidth(3, , psPWM);
}

