/*
This example demonstrates how to implement a Proportional-Derivative (PD) Postion Controller of DC Motor

                   PositionController       DC Motor    
                     +-----------+         +---------+        
              +      |           | voltage |    b    |   position  
pos_ref ----->o----->| Kp + Kd*s |-------->|  -----  |--------->
              ^ -    |           |         |  s + a  |   |    
              |      +-----------+         +---------+   |
              |           10ms                           |
              |                                          |
              +------------------------------------------+

In Simulink, the discrete derivative is always implemented using a first-order filter (Check Discrete PID Controller block)
The continuous version is as follows:

                      PositionController            DC Motor    
                     +------------------+         +---------+        
              +      |              N   | voltage |    b    |   position  
pos_ref ----->o----->| Kp + Kd*s ------ |-------->|  -----  |--------->
              ^ -    |            s + N |         |  s + a  |    |
              |      +------------------+         +---------+    |
              |              10ms                                |
              |                                                  |
              +--------------------------------------------------+

Author: PabloC
Date: 09/04/2024
Dependencies: <XSpaceV20.h>

Install the library dependencies
- XSpaceV20

Update your platformio.ini file to include the following line:

monitor_speed = 1000000
*/

#include <Arduino.h>
#include <XSpaceV20.h>
#include <XSControl.h>

// Definition of board, filter and controller objects
XSpaceV20Board XSBoard;
XSController Controller;

// Constants for the motor control configuration
#define PWM_FREQUENCY 20000 // Defines PWM frequency in Hz for motor control
#define ENCODER_RESOLUTION 1280 // Specifies the resolution of the motor encoder
#define DRV8837_POWER_SUPPLY 5 // Defines the power supply voltage (in volts) for the DRV8837 motor driver

// Variables for storing the raw position measurements
double position; // Raw position measurement

// Task for controlling the motor position
void PositionController(void *pvParameters) {
  double voltage;
  // Wake up the DRV8837 motor driver
  XSBoard.DRV8837_Wake();

  // PID controller parameters (Update this values for your application and your motor)
  double Kp = 0.4; // Proportional gain
  double Kd = 0.02;  // Derivative gain
  double fc = 20; //Low Pass Band filter cuttoff frequency in 20Hz
  double N = 2*PI*fc; // Filter coeficient N = 2*pi*fc
  double Ts = 0.01; // Sampling time (in seconds)
  double pos_ref = 180; // Target position in degrees

  while (true) {
    position = XSBoard.GetEncoderPosition(E1,DEGREES);
    // Calculate the control signal (voltage) to be applied to the motor
    voltage = Controller.PD_ControlLaw(position, pos_ref, Kp, Kd, FORWARD_EULER, N, Ts);
    // Apply the calculated voltage to the motor
    XSBoard.DRV8837_Voltage(voltage);
    // Output the position to the Serial Monitor for debugging
    Serial.println(position);
    // Delay of 10 ms between control cycles
    vTaskDelay(10);
  }
  // Task cleanup, if ever exited
  vTaskDelete(NULL);
}

void setup() {
  // Initialize serial communication at 1,000,000 baud for fast data transfer
  Serial.begin(1000000);
  // Initialize the motor board with the specified configuration
  XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, DRV8837_POWER_SUPPLY);

  // Create Real-Time Operating System (RTOS) tasks for motor control
  xTaskCreate(PositionController, "ControlTask", 2000, NULL, 1, NULL);
}

void loop() {
  // The main loop is intentionally left empty as task scheduling is handled by FreeRTOS.
}
