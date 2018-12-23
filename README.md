
This repository contains an implementation of a quadrotor flight controller and remote control based on the Texas Instruments tm4c123g microcontroller.

<h3>Algorithmic design</h3>	
<p>The flight controller uses the equations of motion of the quadrotor for a PD controller. The moments of inertia, mass and body dimensions need to be supplied.</p>

Reference: 
https://repository.upenn.edu/cgi/viewcontent.cgi?article=1705&context=edissertations

<h3>Hardware used</h3>	

Flight controller:

  <p>-1 x TI tm4c123g MCU<br>
	-1 x MPU6050 IMU</p>

Quadrotor:

  <p>-4 x A2212/13T 1000KV brushless motors<br>
  -4 x ESCs are not branded with standard functionality</p>
  
Remote control:

  <p>-2 x HC12 serial radio transceivers<br>
	-1 x Arduino nano<br>
  -2 x Two axis thumbstick potentiometers</p>

<h3>Remote control design</h3>
<p>The remote control used HC12 serial radio transceivers operating at 9600 bytes/sec (operation range of ~500 m). The HC12 I/O is input and output is serial. The remote control needs to send thrust, yaw pitch and roll instructions. The thumbsticks potentiometers are not precise and thus allow allow only 3 states: increase value, decrease value and neutral.
The serial data is 14 bytes long and the message format used used is as follows:</p>

‘s’, thrust, yaw, pitch, roll, _, _, _, _, _, _, _, message counter, ‘e’

Where thrust, yaw, pitch, roll are in [0, 255]. The underscores are currently unused bytes.

<h3>System operation</h3>
<p>On every startup of the flight controller the ECSs are calibrated. When the calibration ends the propellers start to rotate at a low angular velocity. At this stage the remote control can be used.</p>



