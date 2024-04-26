Project Overview
This project successfully developed a digital system capable of sampling 8-bit data from any available analog inputs on Digilent’s Cora Z7 board's Analog header. 
The system controls the angular position of a servo motor, the intensity of PMOD LEDs, and the speed of a DC motor. It meets the following specifications:

Parts List     
1.	Digilent Cora-Z7-07S 
2.	PMOD buttons and LEDs
3.	Arduino Starter Kit components:

  	o	Photoresistors(s)  

  	o	Potentiometer 

  	o	Ultrasonic sensor (HC-SC04)

  	o	Servo Motor (SG90) 

  	o	DC motor 

  	o	Passive buzzer 

  	o	LCD module (I2C)

  	o	3.3/5V Power Supply

Key Features

Hardware and Tools: Utilizes Digilentinc's Cora-Z7 board, along with Xilinx's Vivado and SDK tools for design.

Design Approach: Utilizes Xilinx's Zynq processor and various IPs to design the system, assign pins, generate the bitstream, and export it to Xilinx SDK.

Analog Control: 
A potentiometer provides a voltage between 0 to 3.3V to control the servo and PMOD LEDs.
Servo rotation ranges from -90 degrees to +90 degrees, controlled by the potentiometer via a custom PWM IP.
PMOD LEDs' intensity controlled using the AXI Timer's PWM output.
DC motor controlled using the AXI Timer's PWM output.
Buzzer controlled by the AXI Timer's PWM output.

User Interface:
Onboard buttons integrated using AXI_GPIO IP blocks with interrupt support.
Button functionalities: system reset, selection between analog sources, and system enable/disable.

Timing and Display:
AXI Timer provides periodic interrupts for timing needs.
I2C LCD unit displays system status and analog source information using custom IP.

Ultrasonic Sensor Integration:
Acts as a protection mechanism, alerting users when an object moves close to the system.
Utilizes Xilinx AXI_Timer’s capture mode to measure the width of the ECHO pulse, indicating the distance to the object.
Alerts users with colored LED lights and a sound alarm (passive buzzer) based on the object's proximity.

Additional Features:
Tri-color LED provides visual alerts based on the distance of the detected object.
Sound alarm frequency varies based on the object's distance.
Distance displayed on the LCD screen.

Debugging and Demonstration:
Recommended to use a debug logic analyzer for debugging purposes.
Demonstrates functionality with a simple data transaction on a selected bus, focusing on essential signals for data transactions.
