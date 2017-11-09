
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
#define MAX_MOTOR_GAMMA     (2.0 * M_PI * 6360 / 60)^2 // max motor rpm
#define K                   (0.62 * 9.81) / ((2 * M_PI * 6360 / 60)^2) // N * Hz^-2

//*****************************************************************************
//
// PD parameters.
//
//*****************************************************************************
#define KD                  3.0
#define KP                  4.0

//*****************************************************************************
//
//! ???
//!
//
//*****************************************************************************
void
error_to_input(tPDController * psPD, tCompDCM * psDCM)
{
//    float total_thrust = m * g /
//            (4.0 * K * cosf(state.phi) * cosf(state.theta));
//
//    // PD errors
//    float e_phi = -KP * (state.phi_des - state.phi) +
//            -KD * (0.0 - state.phi_dot);
//    float e_theta = -KP * (state.theta_des - state.theta) +
//            -KD * (0.0 - state.theta_dot);
//    float e_psi = -KP * (state.psi_des - state.psi) +
//            -KD * (0.0 - state.psi_dot);
//
//    // gamma_i = omega_i^2
//    float gamma[4];
//    gamma[0] = total_thrust -
//            (2 * b * I_XX * e_phi + K * ARM_LENGTH * I_ZZ * e_psi) /
//            (4.0 * b * K * ARM_LENGTH);
//
//    gamma[1] = total_thrust + I_ZZ * e_psi / (4.0 * b) -
//            I_YY * e_theta / (2.0 * K * ARM_LENGTH);
//
//    gamma[2] = total_thrust -
//            (-2 * b * I_XX * e_phi + K * ARM_LENGTH * I_ZZ * e_psi) /
//                    (4.0 * b * K * ARM_LENGTH);
//
//    gamma[3] = total_thrust + I_ZZ * e_psi / (4.0 * b) +
//            I_YY * e_theta / (2.0 * K * ARM_LENGTH);
//
//    // limit motor angular velocity
//    if (gamma[0] > MAX_MOTOR_GAMMA)    gamma[0] = MAX_MOTOR_GAMMA;
//    if (gamma[1] > MAX_MOTOR_GAMMA)    gamma[1] = MAX_MOTOR_GAMMA;
//    if (gamma[2] > MAX_MOTOR_GAMMA)    gamma[2] = MAX_MOTOR_GAMMA;
//    if (gamma[3] > MAX_MOTOR_GAMMA)    gamma[3] = MAX_MOTOR_GAMMA;
//
//    pdCont->fGamma[0] = gamma[0];
//    pdCont->fGamma[1] = gamma[1];
//    pdCont->fGamma[2] = gamma[2];
//    pdCont->fGamma[3] = gamma[3];
}
