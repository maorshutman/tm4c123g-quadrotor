
//*****************************************************************************
//
// main.c - A tm4c123g based quadrotor flight controller.
//
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "mpu9150mod.h"
#include "comp_dcm.h"
#include "drivers/rgb.h"
#include "inc/tm4c123gh6pm.h"
#include "hc12.h"
#include "buffer.h"
#include "escpwm.h"
#include "controller.h"
#include "battery_adc.h"


//*****************************************************************************
//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Global array for holding the color values for the RGB.
//
//*****************************************************************************
uint32_t g_pui32Colors[3];

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global Instance structure to manage the PWM state.
//
//*****************************************************************************
tPWM g_sPWMInst;

//*****************************************************************************
//
// Global Instance structure for the PD controller.
//
//*****************************************************************************
tPDController g_sPDControllerInst;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction is complete
//
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 data is ready to be retrieved.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        10

uint32_t g_ui32PrintSkipCounter;

//*****************************************************************************
//
// Global variables for UART printing.
//
//*****************************************************************************
int_fast32_t i32IPart[16], i32FPart[16];
uint_fast32_t ui32Idx, ui32CompDCMStarted;
float pfData[16];
float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;

//*****************************************************************************
//
// Global variable for the number of samples taken for gyro bias estimation.
//
//*****************************************************************************
#define GYRO_BIAS_SAMPLES           2000

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void
MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//
//*****************************************************************************
void
IntGPIOb(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
    {
        //
        // MPU9150 Data is ready for retrieval and processing.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MPU9150I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void
MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");

    //
    // Set RGB Color to RED
    //
    g_pui32Colors[0] = 0xFFFF;
    g_pui32Colors[1] = 0;
    g_pui32Colors[2] = 0;
    RGBColorSet(g_pui32Colors);

    //
    // Increase blink rate to get attention
    //
    RGBBlinkRateSet(10.0f);

    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        //
        // Do Nothing
        //
    }
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void
MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MPU9150AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8I2CDoneFlag = 0;
}

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Configure the mpu6050 module.
//
//*****************************************************************************
void
ConfigureMPU6050()
{
    //
    // Initialize convenience pointers that clean up and clarify the code
    // meaning. We want all the data in a single contiguous array so that
    // we can make our pretty printing easier later.
    //
    pfAccel = pfData;
    pfGyro = pfData + 3;
    pfMag = pfData + 6;
    pfEulers = pfData + 9;
    pfQuaternion = pfData + 12;

    //
    // Enable port B used for motion interrupt.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("\033[2JMPU9150 Raw Example\n");

    //
    // Set the color to a purple approximation.
    //
    g_pui32Colors[RED] = 0x8000;
    g_pui32Colors[BLUE] = 0x8000;
    g_pui32Colors[GREEN] = 0x0000;

    //
    // Initialize RGB driver.
    //
    RGBInit(0);
    RGBColorSet(g_pui32Colors);
    RGBIntensitySet(0.5f);
    RGBEnable();

    //
    // The I2C3 peripheral must be enabled before use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    //
    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    //
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    ROM_IntEnable(INT_GPIOB);

    //
    // Keep only some parts of the systems running while in sleep mode.
    // GPIOB is for the MPU9150 interrupt pin.
    // UART0 is the virtual serial port
    // TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
    // I2C3 is the I2C interface to the ISL29023
    //
    //    ROM_SysCtlPeripheralClockGating(true);
    //    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    //    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    //    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    //    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    //    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C1);
    //    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C1_BASE, INT_I2C1, 0xff, 0xff,
             ROM_SysCtlClockGet());

    //
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Write application specific sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
            MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
            MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
            MPU9150_INT_PIN_CFG_LATCH_INT_EN;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
                 &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Initialize the DCM system. 250 hz sample rate.
    //
    CompDCMInit(&g_sCompDCMInst, 1.0f / 250.0f, 0.0f, 0.0f, 0.0f);

    UARTprintf("\033[2J\033[H");
    UARTprintf("MPU9150 6-Axis Simple Data Application Example\n\n");
    UARTprintf("\033[20GX\033[31G|\033[43GY\033[54G|\033[66GZ\n\n");
    UARTprintf("Accel\033[8G|\033[31G|\033[54G|\n\n");
    UARTprintf("Gyro\033[8G|\033[31G|\033[54G|\n\n");
    UARTprintf("Mag\033[8G|\033[31G|\033[54G|\n\n");
    UARTprintf("\n\033[20GRoll\033[31G|\033[43GPitch\033[54G|\033[66GYaw\n\n");
    UARTprintf("Eulers\033[8G|\033[31G|\033[54G|\n\n");

    UARTprintf("\n\033[17GQ1\033[26G|\033[35GQ2\033[44G|\033[53GQ3\033[62G|"
            "\033[71GQ4\n\n");
    UARTprintf("Q\033[8G|\033[26G|\033[44G|\033[62G|\n\n");

    //
    // Enable blinking indicates config finished successfully
    //
    RGBBlinkRateSet(1.0f);

    ui32CompDCMStarted = 0;
}

//*****************************************************************************
//
// Measures gyroscope bias.
//
//*****************************************************************************
void
CalibrateIMU(float * biasWx, float * biasWy, float * biasWz,
             float * biasAx, float * biasAy, float * biasAz)
{
    float gyro[3];
    float accel[3];
    float gyroBias[3] = {};
    float accelBias[3] = {};
    int i;
    for (i = 0; i < GYRO_BIAS_SAMPLES; i++)
    {
        while(!g_vui8I2CDoneFlag)
        {
            ROM_SysCtlSleep();
        }

        //
        // Clear the flag
        //
        g_vui8I2CDoneFlag = 0;

        //
        // Get floating point version of angular velocities in rad/sec
        //
        MPU9150DataGyroGetFloat(&g_sMPU9150Inst, gyro, gyro + 1,
                                gyro + 2);

        //
        // Get floating point version of acceleration in m/sec^2
        //
        MPU9150DataAccelGetFloat(&g_sMPU9150Inst, accel, accel + 1,
                                accel + 2);

        gyroBias[0] += gyro[0];
        gyroBias[1] += gyro[1];
        gyroBias[2] += gyro[2];
        accelBias[0] += accel[0];
        accelBias[1] += accel[1];
        accelBias[2] += accel[2];
    }

    *biasWx = gyroBias[0] / GYRO_BIAS_SAMPLES;
    *biasWy = gyroBias[1] / GYRO_BIAS_SAMPLES;
    *biasWz = gyroBias[2] / GYRO_BIAS_SAMPLES;

    // DEBUGGING
    *biasAx = 0.0;
    *biasAy = 0.0;
    *biasAz = 0.0;
}

//*****************************************************************************
//
// Main application entry point.
//
//*****************************************************************************
int
main(void)
{
    //
    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //
    // Initialize PWM.
    //
    InitPWM(&g_sPWMInst);

    //
    // Calibrates throttle.
    //
    CalibrateThrottle(&g_sPWMInst);

    //
    // Initialize UART for radio receiver.
    //
    InitHC12UART();

    //
    // Configures mpu6050 module and UART for display.
    //
    ConfigureMPU6050();

    //
    // Measures gyroscope bias.
    //
    CalibrateIMU(g_sCompDCMInst.fGyroBias, g_sCompDCMInst.fGyroBias + 1, g_sCompDCMInst.fGyroBias + 2,
                 g_sCompDCMInst.fAccelBias, g_sCompDCMInst.fAccelBias + 1, g_sCompDCMInst.fAccelBias + 2);

    //
    // Initialize PD controller for hovering.
    //
    InitPDController(&g_sPDControllerInst);

    // DEBUGGING
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);

    //
    // Main loop.
    //
    while(1)
    {
        //
        // Reads IMU data.
        //

        //
        // Go to sleep mode while waiting for data ready.
        //
        while(!g_vui8I2CDoneFlag)
        {
            ROM_SysCtlSleep();
        }

        // DEBUGGING
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x10);
        SysCtlDelay(0.0001 * SysCtlClockGet() / 3);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x0);

        //
        // Clears the flag.
        //
        g_vui8I2CDoneFlag = 0;

        //
        // Get floating point version of the Accel Data in m/s^2.
        //
        MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
                                 pfAccel + 2);

        //
        // Get floating point version of angular velocities in rad/sec
        //
        MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,
                                pfGyro + 2);

        //
        // Get floating point version of magnetic fields strength in tesla
        //
        MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,
                                   pfMag + 2);

        //
        // Check if this is our first data ever.
        //
        if(ui32CompDCMStarted == 0)
        {
            //
            // Set flag indicating that DCM is started.
            // Perform the seeding of the DCM with the first data set.
            //
            ui32CompDCMStarted = 1;
            CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
                                 pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
                               pfAccel[2]);
            CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
                              pfGyro[2]);
            CompDCMStart(&g_sCompDCMInst);
        }
        else
        {
            //
            // DCM Is already started.  Perform the incremental update.
            //
            CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
                                 pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
                               pfAccel[2]);
            CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
                              pfGyro[2]);
            CompDCMUpdate(&g_sCompDCMInst);
        }

        //
        // Increment the skip counter.  Skip counter is used so we do not
        // overflow the UART with data.
        //
        g_ui32PrintSkipCounter++;
        if(g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT)
        {
            //
            // Reset skip counter.
            //
            g_ui32PrintSkipCounter = 0;

            //
            // Get Euler data. (Roll Pitch Yaw)
            //
            CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
                                 pfEulers + 2);

            //
            // Get Quaternions.
            //
            CompDCMComputeQuaternion(&g_sCompDCMInst, pfQuaternion);

            //
            // convert mag data to micro-tesla for better human interpretation.
            //
            pfMag[0] *= 1e6;
            pfMag[1] *= 1e6;
            pfMag[2] *= 1e6;

            //
            // Convert Eulers to degrees. 180/PI = 57.29...
            // Convert Yaw to 0 to 360 to approximate compass headings.

            pfEulers[0] *= 57.295779513082320876798154814105f;
            pfEulers[1] *= 57.295779513082320876798154814105f;
            pfEulers[2] *= 57.295779513082320876798154814105f;
//            if(pfEulers[2] < 0)
//            {
//                pfEulers[2] += 360.0f;
//            }

            //
            // Calibrated accel. data
            //
            pfAccel[0] = g_sCompDCMInst.pfAccel[0];
            pfAccel[1] = g_sCompDCMInst.pfAccel[1];
            pfAccel[2] = g_sCompDCMInst.pfAccel[2];

            //
            // Now drop back to using the data as a single array for the
            // purpose of decomposing the float into a integer part and a
            // fraction (decimal) part.
            //
            for(ui32Idx = 0; ui32Idx < 16; ui32Idx++)
            {
                //
                // Convert float value to a integer truncating the decimal part.
                //
                i32IPart[ui32Idx] = (int32_t) (pfData[ui32Idx]);

                //
                // Multiply by 1000 to preserve first three decimal values.
                // Truncates at the 3rd decimal place.
                //
                i32FPart[ui32Idx] = (int32_t) (pfData[ui32Idx] * 1000.0f);

                //
                // Subtract off the integer part from this newly formed decimal
                // part.
                //
                i32FPart[ui32Idx] = i32FPart[ui32Idx] -
                        (i32IPart[ui32Idx] * 1000);

                //
                // make the decimal part a positive number for display.
                //
                if(i32FPart[ui32Idx] < 0)
                {
                    i32FPart[ui32Idx] *= -1;
                }
            }

            //
            // Print the acceleration numbers in the table.
            //
            UARTprintf("\033[5;17H%3d.%03d", i32IPart[0], i32FPart[0]);
            UARTprintf("\033[5;40H%3d.%03d", i32IPart[1], i32FPart[1]);
            UARTprintf("\033[5;63H%3d.%03d", i32IPart[2], i32FPart[2]);

            //
            // Print the angular velocities in the table.
            //
            UARTprintf("\033[7;17H%3d.%03d", i32IPart[3], i32FPart[3]);
            UARTprintf("\033[7;40H%3d.%03d", i32IPart[4], i32FPart[4]);
            UARTprintf("\033[7;63H%3d.%03d", i32IPart[5], i32FPart[5]);

            //
            // Print the magnetic data in the table.
            //
            UARTprintf("\033[9;17H%3d.%03d", i32IPart[6], i32FPart[6]);
            UARTprintf("\033[9;40H%3d.%03d", i32IPart[7], i32FPart[7]);
            UARTprintf("\033[9;63H%3d.%03d", i32IPart[8], i32FPart[8]);

            //
            // Print the Eulers in a table.
            //
            UARTprintf("\033[14;17H%3d.%03d", i32IPart[9], i32FPart[9]);
            UARTprintf("\033[14;40H%3d.%03d", i32IPart[10], i32FPart[10]);
            UARTprintf("\033[14;63H%3d.%03d", i32IPart[11], i32FPart[11]);

            //
            // Print the quaternions in a table format.
            //
            UARTprintf("\033[19;14H%3d.%03d", i32IPart[12], i32FPart[12]);
            UARTprintf("\033[19;32H%3d.%03d", i32IPart[13], i32FPart[13]);
            UARTprintf("\033[19;50H%3d.%03d", i32IPart[14], i32FPart[14]);
            UARTprintf("\033[19;68H%3d.%03d", i32IPart[15], i32FPart[15]);
        }

        //
        // Reads desired state via UART2 buffer.
        //
        ReadDesiredState(&g_sPDControllerInst, &g_sPWMInst);

        //
        // PD controller.
        //
        ErrorToInput(&g_sPDControllerInst, &g_sCompDCMInst);
        PDContUpdatePWM(&g_sPDControllerInst, &g_sPWMInst);
    }

    return 0;
}
