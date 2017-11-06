//*****************************************************************************
//
// comp_dcm.c - Complementary filter algorithm on a Direction Cosine Matrix for
//              fusing sensor data from an accelerometer, gyroscope, and
//              magnetometer.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/debug.h"
#include "comp_dcm.h"
#include "sensorlib/vector.h"

//*****************************************************************************
//
//! \addtogroup comp_dcm_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If M_PI has not been defined by the system headers, define it here.
//
//*****************************************************************************
#ifndef M_PI
#define M_PI                    3.14159265358979323846
#endif

#define deltat 0.004f                                     // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError            // compute beta

//*****************************************************************************
//
//! Initializes the complementary filter DCM attitude estimation state.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param fDeltaT is the amount of time between DCM updates, in seconds.
//! \param fScaleA is the weight of the accelerometer reading in determining
//! the updated attitude estimation.
//! \param fScaleG is the weight of the gyroscope reading in determining the
//! updated attitude estimation.
//! \param fScaleM is the weight of the magnetometer reading in determining the
//! updated attitude estimation.
//!
//! This function initializes the complementary filter DCM attitude estimation
//! state, and must be called prior to performing any attitude estimation.
//!
//! New readings must be supplied to the complementary filter DCM attitude
//! estimation algorithm at the rate specified by the \e fDeltaT parameter.
//! Failure to provide new readings at this rate results in inaccuracies in the
//! attitude estimation.
//!
//! The \e fScaleA, \e fScaleG, and \e fScaleM weights must sum to one.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMInit(tCompDCM *psDCM, float fDeltaT, float fScaleA, float fScaleG,
            float fScaleM)
{
    //
    // Initialize the DCM matrix to the identity matrix.
    //
    psDCM->ppfDCM[0][0] = 1.0;
    psDCM->ppfDCM[0][1] = 0.0;
    psDCM->ppfDCM[0][2] = 0.0;
    psDCM->ppfDCM[1][0] = 0.0;
    psDCM->ppfDCM[1][1] = 1.0;
    psDCM->ppfDCM[1][2] = 0.0;
    psDCM->ppfDCM[2][0] = 0.0;
    psDCM->ppfDCM[2][1] = 0.0;
    psDCM->ppfDCM[2][2] = 1.0;

    //
    // Save the time delta between DCM updates.
    //
    psDCM->fDeltaT = fDeltaT;

    //
    // Save the scaling factors that are applied to the accelerometer,
    // gyroscope, and magnetometer readings.
    //
    psDCM->fScaleA = fScaleA;
    psDCM->fScaleG = fScaleG;
    psDCM->fScaleM = fScaleM;
}

//*****************************************************************************
//
//! Updates the accelerometer reading used by the complementary filter DCM
//! algorithm.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param fAccelX is the accelerometer reading in the X body axis.
//! \param fAccelY is the accelerometer reading in the Y body axis.
//! \param fAccelZ is the accelerometer reading in the Z body axis.
//!
//! This function updates the accelerometer reading used by the complementary
//! filter DCM algorithm.  The accelerometer readings provided to this function
//! are used by subsequent calls to CompDCMStart() and CompDCMUpdate() to
//! compute the attitude estimate.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMAccelUpdate(tCompDCM *psDCM, float fAccelX, float fAccelY,
                   float fAccelZ)
{
    //
    // The user should never pass in values that are not-a-number
    //
    ASSERT(!isnan(fAccelX));
    ASSERT(!isnan(fAccelY));
    ASSERT(!isnan(fAccelZ));

    //
    // Save the new accelerometer reading.
    //
    psDCM->pfAccel[0] = fAccelX;
    psDCM->pfAccel[1] = fAccelY;
    psDCM->pfAccel[2] = fAccelZ;
}

//*****************************************************************************
//
//! Updates the gyroscope reading used by the complementary filter DCM
//! algorithm.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param fGyroX is the gyroscope reading in the X body axis.
//! \param fGyroY is the gyroscope reading in the Y body axis.
//! \param fGyroZ is the gyroscope reading in the Z body axis.
//!
//! This function updates the gyroscope reading used by the complementary
//! filter DCM algorithm.  The gyroscope readings provided to this function are
//! used by subsequent calls to CompDCMUpdate() to compute the attitude
//! estimate.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMGyroUpdate(tCompDCM *psDCM, float fGyroX, float fGyroY, float fGyroZ)
{
    //
    // The user should never pass in values that are not-a-number
    //
    ASSERT(!isnan(fGyroX));
    ASSERT(!isnan(fGyroY));
    ASSERT(!isnan(fGyroZ));

    //
    // Save the new gyroscope reading.
    //
    psDCM->pfGyro[0] = fGyroX - psDCM->fBias[0];
    psDCM->pfGyro[1] = fGyroY - psDCM->fBias[1];
    psDCM->pfGyro[2] = fGyroZ - psDCM->fBias[2];
}

//*****************************************************************************
//
//! Updates the magnetometer reading used by the complementary filter DCM
//! algorithm.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param fMagnetoX is the magnetometer reading in the X body axis.
//! \param fMagnetoY is the magnetometer reading in the Y body axis.
//! \param fMagnetoZ is the magnetometer reading in the Z body axis.
//!
//! This function updates the magnetometer reading used by the complementary
//! filter DCM algorithm.  The magnetometer readings provided to this function
//! are used by subsequent calls to CompDCMStart() and CompDCMUpdate() to
//! compute the attitude estimate.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMMagnetoUpdate(tCompDCM *psDCM, float fMagnetoX, float fMagnetoY,
                     float fMagnetoZ)
{
    //
    // The user should never pass in values that are not-a-number
    //
    ASSERT(!isnan(fMagnetoX));
    ASSERT(!isnan(fMagnetoY));
    ASSERT(!isnan(fMagnetoZ));

    //
    // Save the new magnetometer reading.
    //
    psDCM->pfMagneto[0] = fMagnetoX;
    psDCM->pfMagneto[1] = fMagnetoY;
    psDCM->pfMagneto[2] = fMagnetoZ;
}

//*****************************************************************************
//
//! Starts the complementary filter DCM attitude estimation from an initial
//! sensor reading.
//!
//! \param psDCM is a pointer to the DCM state structure.
//!
//! This function computes the initial complementary filter DCM attitude
//! estimation state based on the initial accelerometer and magnetometer
//! reading.  While not necessary for the attitude estimation to converge,
//! using an initial state based on sensor readings results in quicker
//! convergence.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMStart(tCompDCM *psDCM)
{
    //
    // The size of gravity when the body is at rest.
    //

    float g = sqrt(psDCM->pfAccel[0] * psDCM->pfAccel[0] +
                   psDCM->pfAccel[1] * psDCM->pfAccel[1] +
                   psDCM->pfAccel[2] * psDCM->pfAccel[2]);

    //
    // Initially the elements of the rotation matrix.
    //
    float r31 = -psDCM->pfAccel[0] / g;
    float r32 = psDCM->pfAccel[1] / g;
    float r33 = psDCM->pfAccel[2] / g;
    float r11  = sqrt(1.0f - r31 * r31);

    //
    // The initial Tait-Bryan angles angles in Z-Y-X convention.
    // alpha is chosen to be 0 initially.
    //
    float betaAngle = atan2f(-r31 , r11);
    float gammaAngle = atan2f(r32 / cosf(betaAngle), r33 / cosf(betaAngle));

    //
    // Updates initial Euler angles.
    //
    psDCM->fEuler[0] = gammaAngle;
    psDCM->fEuler[1] = -betaAngle;
    psDCM->fEuler[2] = 0.0f;

    //
    // Updates the initial DCM.
    //
    psDCM->ppfDCM[0][0] = cosf(betaAngle);
    psDCM->ppfDCM[0][1] = sinf(betaAngle) * sinf(gammaAngle);
    psDCM->ppfDCM[0][2] = sinf(betaAngle) * cosf(gammaAngle);
    psDCM->ppfDCM[1][0] = 0.0f;
    psDCM->ppfDCM[1][1] = cosf(gammaAngle);
    psDCM->ppfDCM[1][2] = -sinf(gammaAngle);
    psDCM->ppfDCM[2][0] = r31;
    psDCM->ppfDCM[2][1] = r32;
    psDCM->ppfDCM[2][2] = r33;
}

//*****************************************************************************
//
//! Updates the complementary filter DCM attitude estimation based on an
//! updated set of sensor readings.
//!
//! \param psDCM is a pointer to the DCM state structure.
//!
//! This function updates the complementary filter DCM attitude estimation
//! state based on the current sensor readings.  This function must be called
//! at the rate specified to CompDCMInit(), with new readings supplied at an
//! appropriate rate (for example, magnetometers typically sample at a much
//! slower rate than accelerometers and gyroscopes).
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMUpdate(tCompDCM *psDCM)
{
    bool bNAN;

    //
    // Determine if the newly updated DCM contains any invalid (in other words,
    // NaN) values.
    //
    bNAN = (isnan(psDCM->ppfDCM[0][0]) ||
            isnan(psDCM->ppfDCM[0][1]) ||
            isnan(psDCM->ppfDCM[0][2]) ||
            isnan(psDCM->ppfDCM[1][0]) ||
            isnan(psDCM->ppfDCM[1][1]) ||
            isnan(psDCM->ppfDCM[1][2]) ||
            isnan(psDCM->ppfDCM[2][0]) ||
            isnan(psDCM->ppfDCM[2][1]) ||
            isnan(psDCM->ppfDCM[2][2]));

    //
    // As a debug measure, we check for NaN in the DCM.  The user can trap
    // this event depending on their implementation of __error__.  Should they
    // choose to disable interrupts and loop forever then they will have
    // preserved the stack and can analyze how they arrived at NaN.
    //
    ASSERT(!bNAN);

    //
    // If any part of the matrix is not-a-number then reset the DCM back to the
    // identity matrix.
    //
    if(bNAN)
    {
        psDCM->ppfDCM[0][0] = 1.0;
        psDCM->ppfDCM[0][1] = 0.0;
        psDCM->ppfDCM[0][2] = 0.0;
        psDCM->ppfDCM[1][0] = 0.0;
        psDCM->ppfDCM[1][1] = 1.0;
        psDCM->ppfDCM[1][2] = 0.0;
        psDCM->ppfDCM[2][0] = 0.0;
        psDCM->ppfDCM[2][1] = 0.0;
        psDCM->ppfDCM[2][2] = 1.0;
    }

    float sigma = sqrtf(psDCM->pfGyro[0] * psDCM->pfGyro[0] +
                        psDCM->pfGyro[1] * psDCM->pfGyro[1] +
                        psDCM->pfGyro[2] * psDCM->pfGyro[2]) * psDCM->fDeltaT;
    float bFactor = sinf(sigma) / sigma;
    float bSqFactor = (1 - cosf(sigma)) / (sigma * sigma);

    // B matrix.
    float b[3][3];
    b[0][0] = 0.0;
    b[0][1] = -psDCM->pfGyro[2];
    b[0][2] = psDCM->pfGyro[1];
    b[1][0] = psDCM->pfGyro[2];
    b[1][1] = 0.0;
    b[1][2] = -psDCM->pfGyro[0];
    b[2][0] = -psDCM->pfGyro[1];
    b[2][1] = psDCM->pfGyro[0];
    b[2][2] = 0.0;

    // B^2 matrix.
    float bSq[3][3];
    bSq[0][0] = -psDCM->pfGyro[1] * psDCM->pfGyro[1] -
                   psDCM->pfGyro[2] * psDCM->pfGyro[2];
    bSq[0][1] = psDCM->pfGyro[0] * psDCM->pfGyro[1];
    bSq[0][2] = psDCM->pfGyro[0] * psDCM->pfGyro[2];
    bSq[1][0] = psDCM->pfGyro[0] * psDCM->pfGyro[1];
    bSq[1][1] = -psDCM->pfGyro[0] * psDCM->pfGyro[0] -
            psDCM->pfGyro[2] * psDCM->pfGyro[2];
    bSq[1][2] = psDCM->pfGyro[1] * psDCM->pfGyro[2];
    bSq[2][0] = psDCM->pfGyro[0] * psDCM->pfGyro[2];
    bSq[2][1] = psDCM->pfGyro[1] * psDCM->pfGyro[2];
    bSq[2][2] = -psDCM->pfGyro[0] * psDCM->pfGyro[0] -
            psDCM->pfGyro[1] * psDCM->pfGyro[1];

    // Incrementing matrix.
    float inc[3][3];
    float dt = psDCM->fDeltaT;
    inc[0][0] = 1.0 + dt * bFactor * b[0][0] + dt * dt * bSqFactor * bSq[0][0];
    inc[0][1] = dt * bFactor * b[0][1] + dt * dt * bSqFactor * bSq[0][1];
    inc[0][2] = dt * bFactor * b[0][2] + dt * dt * bSqFactor * bSq[0][2];
    inc[1][0] = dt * bFactor * b[1][0] + dt * dt * bSqFactor * bSq[1][0];
    inc[1][1] = 1.0 + dt * bFactor * b[1][1] + dt * dt * bSqFactor * bSq[1][1];
    inc[1][2] = dt * bFactor * b[1][2] + dt * dt * bSqFactor * bSq[1][2];
    inc[2][0] = dt * bFactor * b[2][0] + dt * dt * bSqFactor * bSq[2][0];
    inc[2][1] = dt * bFactor * b[2][1] + dt * dt * bSqFactor * bSq[2][1];
    inc[2][2] = 1.0 + dt * bFactor * b[2][2] + dt * dt * bSqFactor * bSq[2][2];

    //
    // Multiply DCM matrix by incrementing matrix.
    //
    int i,j;
    float tempDCM[3][3];
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            tempDCM[i][j] = psDCM->ppfDCM[i][0] * inc[0][j] +
                    psDCM->ppfDCM[i][1] * inc[1][j] +
                    psDCM->ppfDCM[i][2] * inc[2][j];
        }
    }

    // Update DCM.
    for (i = 0; i < 3; i++)
          for (j = 0; j < 3; j++)
              psDCM->ppfDCM[i][j] = tempDCM[i][j];

    //
    // Updates Euler angles.
    //
    CompDCMComputeEulers(psDCM, psDCM->fEuler, psDCM->fEuler + 1,
                         psDCM->fEuler + 2);
}

//*****************************************************************************
//
//! Returns the current DCM attitude estimation matrix.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param ppfDCM is a pointer to the array into which to store the DCM matrix
//! values.
//!
//! This function returns the current value of the DCM matrix.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMMatrixGet(tCompDCM *psDCM, float ppfDCM[3][3])
{
    //
    // Return the current DCM matrix.
    //
    ppfDCM[0][0] = psDCM->ppfDCM[0][0];
    ppfDCM[0][1] = psDCM->ppfDCM[0][1];
    ppfDCM[0][2] = psDCM->ppfDCM[0][2];
    ppfDCM[1][0] = psDCM->ppfDCM[1][0];
    ppfDCM[1][1] = psDCM->ppfDCM[1][1];
    ppfDCM[1][2] = psDCM->ppfDCM[1][2];
    ppfDCM[2][0] = psDCM->ppfDCM[2][0];
    ppfDCM[2][1] = psDCM->ppfDCM[2][1];
    ppfDCM[2][2] = psDCM->ppfDCM[2][2];
}

//*****************************************************************************
//
//! Computes the Euler angles from the DCM attitude estimation matrix.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param pfRoll is a pointer to the value into which the roll is stored.
//! \param pfPitch is a pointer to the value into which the pitch is stored.
//! \param pfYaw is a pointer to the value into which the yaw is stored.
//!
//! This function computes the Euler angles that are represented by the DCM
//! attitude estimation matrix.  If any of the Euler angles is not required,
//! the corresponding parameter can be \b NULL.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMComputeEulers(tCompDCM *psDCM, float *pfRoll, float *pfPitch,
                     float *pfYaw)
{
    //
    // Compute the roll, pitch, and yaw as required.
    //
    if(pfRoll)
    {
        *pfRoll = atan2f(psDCM->ppfDCM[2][1], psDCM->ppfDCM[2][2]);
    }
    if(pfPitch)
    {
        *pfPitch = -asinf(psDCM->ppfDCM[2][0]);
    }
    if(pfYaw)
    {
        *pfYaw = atan2f(psDCM->ppfDCM[1][0], psDCM->ppfDCM[0][0]);
    }
}

//*****************************************************************************
//
//! Computes the quaternion from the DCM attitude estimation matrix.
//!
//! \param psDCM is a pointer to the DCM state structure.
//! \param pfQuaternion is an array into which the quaternion is stored.
//!
//! This function computes the quaternion that is represented by the DCM
//! attitude estimation matrix.
//!
//! \return None.
//
//*****************************************************************************
void
CompDCMComputeQuaternion(tCompDCM *psDCM, float pfQuaternion[4])
{
    float fQs, fQx, fQy, fQz;

    //
    // Partially compute Qs, Qx, Qy, and Qz based on the DCM diagonals.  The
    // square root, an expensive operation, is computed for only one of these
    // as determined later.
    //
    fQs = 1 + psDCM->ppfDCM[0][0] + psDCM->ppfDCM[1][1] + psDCM->ppfDCM[2][2];
    fQx = 1 + psDCM->ppfDCM[0][0] - psDCM->ppfDCM[1][1] - psDCM->ppfDCM[2][2];
    fQy = 1 - psDCM->ppfDCM[0][0] + psDCM->ppfDCM[1][1] - psDCM->ppfDCM[2][2];
    fQz = 1 - psDCM->ppfDCM[0][0] - psDCM->ppfDCM[1][1] + psDCM->ppfDCM[2][2];

    //
    // See if Qs is the largest of the diagonal values.
    //
    if((fQs > fQx) && (fQs > fQy) && (fQs > fQz))
    {
        //
        // Finish the computation of Qs.
        //
        fQs = sqrtf(fQs) / 2;

        //
        // Compute the values of the quaternion based on Qs.
        //
        pfQuaternion[0] = fQs;
        pfQuaternion[1] = ((psDCM->ppfDCM[2][1] - psDCM->ppfDCM[1][2]) /
                           (4 * fQs));
        pfQuaternion[2] = ((psDCM->ppfDCM[0][2] - psDCM->ppfDCM[2][0]) /
                           (4 * fQs));
        pfQuaternion[3] = ((psDCM->ppfDCM[1][0] - psDCM->ppfDCM[0][1]) /
                           (4 * fQs));
    }

    //
    // Qs is not the largest, so see if Qx is the largest remaining diagonal
    // value.
    //
    else if((fQx > fQy) && (fQx > fQz))
    {
        //
        // Finish the computation of Qx.
        //
        fQx = sqrtf(fQx) / 2;

        //
        // Compute the values of the quaternion based on Qx.
        //
        pfQuaternion[0] = ((psDCM->ppfDCM[2][1] - psDCM->ppfDCM[1][2]) /
                           (4 * fQx));
        pfQuaternion[1] = fQx;
        pfQuaternion[2] = ((psDCM->ppfDCM[1][0] + psDCM->ppfDCM[0][1]) /
                           (4 * fQx));
        pfQuaternion[3] = ((psDCM->ppfDCM[0][2] + psDCM->ppfDCM[2][0]) /
                           (4 * fQx));
    }

    //
    // Qs and Qx are not the largest, so see if Qy is the largest remaining
    // diagonal value.
    //
    else if(fQy > fQz)
    {
        //
        // Finish the computation of Qy.
        //
        fQy = sqrtf(fQy) / 2;

        //
        // Compute the values of the quaternion based on Qy.
        //
        pfQuaternion[0] = ((psDCM->ppfDCM[0][2] - psDCM->ppfDCM[2][0]) /
                           (4 * fQy));
        pfQuaternion[1] = ((psDCM->ppfDCM[1][0] + psDCM->ppfDCM[0][1]) /
                           (4 * fQy));
        pfQuaternion[2] = fQy;
        pfQuaternion[3] = ((psDCM->ppfDCM[2][1] + psDCM->ppfDCM[1][2]) /
                           (4 * fQy));
    }

    //
    // Qz is the largest diagonal value.
    //
    else
    {
        //
        // Finish the computation of Qz.
        //
        fQz = sqrtf(fQz) / 2;

        //
        // Compute the values of the quaternion based on Qz.
        //
        pfQuaternion[0] = ((psDCM->ppfDCM[1][0] - psDCM->ppfDCM[0][1]) /
                           (4 * fQz));
        pfQuaternion[1] = ((psDCM->ppfDCM[0][2] + psDCM->ppfDCM[2][0]) /
                           (4 * fQz));
        pfQuaternion[2] = ((psDCM->ppfDCM[2][1] + psDCM->ppfDCM[1][2]) /
                           (4 * fQz));
        pfQuaternion[3] = fQz;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
