/*
This example demonstrates how to implement a State Space Speed Controller for a DC Motor.

Block Diagram:

                                                 
                    Integrator                            DC Motor       
                     +-----+                             +---------+  
             +       |  1  |  x2  +----+    -   voltage  |    b    |   speed 
 speed_ref --->o---->| --- |------| K2 |----->o--------->|  -----  |------------> 
               ^ -   |  s  |      +----+    - ^          |  s + a  |     | x1 
               |     +-----+                  |          +---------+     |   
               |       10ms                   |       +----+             |
               |                              +-------| K1 |<------------|                         
               |                                      +----+             |   
               +---------------------------------------------------------+

              NOTE: voltage = -K1*x1 - K2*x2

Author: PabloC
Date: 09/04/2024
Dependencies: <XSpaceV21.h>
              <XSControl.h>

Install the library dependencies:
- XSpaceV21
- XSControl

Update your platformio.ini file to include the following line:

monitor_speed = 1000000
*/

#include <Arduino.h>
#include <XSpaceV21.h>
#include <XSControl.h>

// Definition of board and controller objects
XSpaceV21Board XSBoard; // Object to interface with the XSpaceV21 hardware
XSController Controller; // Object to manage control algorithms

// Constants for the motor control configuration
#define PWM_FREQUENCY 20000 // PWM frequency in Hz for motor control
#define ENCODER_RESOLUTION 1280 // Resolution of the motor encoder (steps per revolution)
#define DRV8837_POWER_SUPPLY 5 // Power supply voltage (in volts) for the DRV8837 motor driver

// Variables to store speed measurements
double speed; // Raw speed measurement from the encoder
double filtered_speed; // Speed measurement after applying the filter

// Task to filter the speed measurement from the motor's encoder
void SpeedFilter(void *pvParameters) {
  while (true) {
    // Measure speed in degrees per second from the encoder
    speed = XSBoard.GetEncoderSpeed(E1, DEGREES_PER_SECOND);
    // Apply a second-order low-pass filter to the speed measurement
    filtered_speed = Controller.SecondOrderLPF(speed, 20, 0.001);
    // Delay for 1 ms between each measurement cycle
    vTaskDelay(1);
  }
  // Task cleanup if the task ever exits
  vTaskDelete(NULL);
}

// Task to control the motor speed
void SpeedController(void *pvParameters) {
  double voltage; // Variable to hold the control voltage to be applied to the motor
  // Wake up the DRV8837 motor driver
  XSBoard.DRV8837_Wake(DRVx1);

  // PID controller parameters
  double K1 = 0.04; // x1 gain
  double K2 = 0.2;  // x2 gain
  double Ts = 0.01; // Sampling time in seconds
  double speed_ref = 180; // Desired speed in degrees per second

  double x1; // State variable representing the current speed
  double x2; // State variable for the error integrator

  while (true) {
    // Calculate the control signal (voltage) to be applied to the motor
    x1 = filtered_speed;
    x2 = Controller.Discrete_Integrator(speed_ref - filtered_speed, FORWARD_EULER, 0.001);
    voltage = -K1 * x1 - K2 * x2;
    // Apply the calculated voltage to the motor
    XSBoard.DRV8837_Voltage(DRVx1, voltage);
    // Output the filtered speed and reference speed to the Serial Monitor for debugging
    Serial.println(String(speed_ref) + " " + String(filtered_speed));
    // Delay for 10 ms between control cycles
    vTaskDelay(10);
  }
  // Task cleanup if the task ever exits
  vTaskDelete(NULL);
}

void setup() {
  // Initialize serial communication at 1,000,000 baud for fast data transfer
  Serial.begin(1000000);
  // Initialize the motor board with the specified configuration
  XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, DRV8837_POWER_SUPPLY);

  // Create Real-Time Operating System (RTOS) tasks for motor control
  xTaskCreate(SpeedFilter, "FilterTask", 2000, NULL, 2, NULL);
  xTaskCreate(SpeedController, "ControlTask", 2000, NULL, 1, NULL);
}

void loop() {
  // The main loop is intentionally left empty as task scheduling is handled by FreeRTOS.
}
