
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
#include "buffer.h"

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

#define g                   9.81 // m * s^-2
#define m                   0.66 // kg
#define b                   5.4e-6 // N * m * Hz^-2, taken form Mellinger paper
#define MOTOR_MASS          0.047 // kg
#define ARM_LENGTH          0.25 // m
#define ARM_MASS            0.043 // kg
#define I_XX                0.00884 // kg * m^2, 2*m_body*R^2/5 + 4*(l/2^0.5)^2*m_motor
#define I_YY                0.00884 // kg * m^2, 2*m_body*R^2/5 + 4*(l/2^0.5)^2*m_motor
#define I_ZZ                0.0165 // kg * m^2, 2*m_body*R^2/5 + 4*l^2*m_motor
#define MAX_MOTOR_OMEGA_SQ  pow(2.0 * M_PI * 6360 / 60.0, 2) * 0.7 // max motor omega^2
#define K                   (0.62 * 9.81) / pow(2 * M_PI * 6360 / 60, 2) // N * Hz^-2

//*****************************************************************************
//
// PD parameters.
//
//*****************************************************************************
#define KD                 40.0 // TODO
#define KP                 5.0 // TODO


//*****************************************************************************
//
// Initialize controller.
//
//*****************************************************************************
void
InitPDController(tPDController * psPD)
{
    //
    // Initial thrust.
    //
    psPD->fThrustZDir = 0.4; // in kg

    //
    // Desired angular state.
    //
    psPD->fDesState[0] = 0.0;
    psPD->fDesState[1] = 0.0;
    psPD->fDesState[2] = 0.0;
}


//*****************************************************************************
//
// Calculates the motor angular velocities from the PD errors.
//
//*****************************************************************************
void
ErrorToInput(tPDController * psPD, tCompDCM * psDCM)
{
    //
    // Total thrust in the z direction in the world frame.
    //
    float totalThrust = psPD->fThrustZDir * g /
            (K * cosf(psDCM->fEuler[0]) * cosf(psDCM->fEuler[1]));

    //
    // PD error. The desired angular velocity is set to 0.
    //
    float eAlpha = KP * (psPD->fDesState[2] - psDCM->fEuler[2]) +
            KD * (0.0 - psDCM->pfGyro[2]);
    float eBeta = KP * (psPD->fDesState[1] - psDCM->fEuler[1]) +
            KD * (0.0 - psDCM->pfGyro[1]);
    float eGamma = KP * (psPD->fDesState[0] - psDCM->fEuler[0]) +
            KD * (0.0 - psDCM->pfGyro[0]);

    //
    // Torques up to constants.
    //
    float torqGamma = I_XX * eGamma * 1.41421356237 / (ARM_LENGTH * K);
    float torqBeta = I_YY * eBeta * 1.41421356237 / (ARM_LENGTH * K);
    float torqAlpha = I_ZZ * eAlpha / b;

    //
    // Updates omega^2 for all motors.
    //
    float omegaSq[4];
    omegaSq[0] = (totalThrust + torqGamma - torqBeta - torqAlpha) / 4.0;
    omegaSq[1] = (totalThrust - torqGamma - torqBeta + torqAlpha) / 4.0;
    omegaSq[2] = (totalThrust - torqGamma + torqBeta - torqAlpha) / 4.0;
    omegaSq[3] = (totalThrust + torqGamma + torqBeta + torqAlpha) / 4.0;

    //
    // Limit motor angular velocity.
    //
    if (omegaSq[0] > MAX_MOTOR_OMEGA_SQ)
        omegaSq[0] = MAX_MOTOR_OMEGA_SQ;
    if (omegaSq[1] > MAX_MOTOR_OMEGA_SQ)
        omegaSq[1] = MAX_MOTOR_OMEGA_SQ;
    if (omegaSq[2] > MAX_MOTOR_OMEGA_SQ)
        omegaSq[2] = MAX_MOTOR_OMEGA_SQ;
    if (omegaSq[3] > MAX_MOTOR_OMEGA_SQ)
        omegaSq[3] = MAX_MOTOR_OMEGA_SQ;

    // DEBUGGING
    // set minimal thrust
    if (omegaSq[0] < (0.1 / K))
            omegaSq[0] = (0.1 / K);
    if (omegaSq[1] < (0.1 / K))
                omegaSq[1] = (0.1 / K);
    if (omegaSq[2] < (0.1 / K))
                omegaSq[2] = (0.1 / K);
    if (omegaSq[3] < (0.1 / K))
                omegaSq[3] = (0.1 / K);

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
PDContUpdatePWM(tPDController * psPD, tPWM * psPWM)
{
    float dutyCycle0 = CalcDutyCycle(psPD->fBatteryV, psPD->fOmegaSq[0]);
    float dutyCycle1 = CalcDutyCycle(psPD->fBatteryV, psPD->fOmegaSq[1]);
    float dutyCycle2 = CalcDutyCycle(psPD->fBatteryV, psPD->fOmegaSq[2]);
    float dutyCycle3 = CalcDutyCycle(psPD->fBatteryV, psPD->fOmegaSq[3]);

    //
    // Updates ESC signals.
    //
    SetMotorPulseWidth(0, 1 - dutyCycle0, psPWM);
    SetMotorPulseWidth(1, 1 - dutyCycle1, psPWM);
    SetMotorPulseWidth(2, 1 - dutyCycle2, psPWM);
    SetMotorPulseWidth(3, 1 - dutyCycle3, psPWM);
}


//*****************************************************************************
//
// Calculates a PWM duty cycle from a required RPM and battery voltage.
//
//*****************************************************************************
float
CalcDutyCycle(float battV, float reqOmegaSq)
{
    // TODO
    // add dependency on battery voltage
    // this is for 11.1 V
    float rpm = 60.0 * sqrtf(reqOmegaSq) / (2.0 * M_PI);
    return 4.82229155e-09 * rpm * rpm + 3.85170924e-05 * rpm + 4.92630283e-01;
}


//*****************************************************************************
//
// Reads desired state of quadrotor via radio.
//
//*****************************************************************************
void
ReadDesiredState(tPDController * psPD, tPWM * psPWM)
{
    //
    // Copies UART2 buffer inside a simple critical section.
    //
    IntMasterDisable();
    char bufferCopy[PACKET_LENGTH];
    int i;
    for (i = 0; i < PACKET_LENGTH; i++) {
        bufferCopy[i] = buff[i];
    }
    IntMasterEnable();

    //
    // Reads desired state from copied buffer into the PD controller's
    // struct.
    //

    //
    // Thrust
    //
    float delta = 0.001;
    if(bufferCopy[1] < 10)
    {
        psPD->fThrustZDir -= delta;
        if (psPD->fThrustZDir < 0.08)
        {
            psPD->fThrustZDir = 0.08;
        }
    }
    else if(bufferCopy[1] > 240)
    {
        psPD->fThrustZDir += delta;
        if (psPD->fThrustZDir > 1.0)
        {
            psPD->fThrustZDir = 1.0;
        }
    }

    delta = 0.01;
    //
    // Yaw (alpha)
    //
    if(bufferCopy[2] < 10)
    {
        psPD->fDesState[2] -= delta;
        if (psPD->fDesState[2] < -0.15)
        {
            psPD->fDesState[2] = -0.15;
        }
    }
    else if(bufferCopy[2] > 240)
    {
        psPD->fDesState[2] += delta;
        if (psPD->fDesState[2] > 0.15)
        {
            psPD->fDesState[2] = 0.15;
        }
    }

    //
    // Pitch (beta)
    //
    if(bufferCopy[3] < 10)
    {
        psPD->fDesState[1] -= delta;
        if (psPD->fDesState[1] < -0.15)
        {
            psPD->fDesState[1] = -0.15;
        }
    }
    else if(bufferCopy[3] > 240)
    {
        psPD->fDesState[1] += delta;
        if (psPD->fDesState[1] > 0.15)
        {
            psPD->fDesState[1] = 0.15;
        }
    }

    //
    // Roll (gamma)
    //
    if(bufferCopy[4] < 10)
    {
        psPD->fDesState[0] -= delta;
        if (psPD->fDesState[0] < -0.15)
        {
            psPD->fDesState[0] = -0.15;
        }
    }
    else if(bufferCopy[4] > 240)
    {
        psPD->fDesState[0] += delta;
        if (psPD->fDesState[0] > 0.15)
        {
            psPD->fDesState[0] = 0.15;
        }
    }
}

