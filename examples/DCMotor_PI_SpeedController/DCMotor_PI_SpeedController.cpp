/*
This example demonstrates how to implement a PI Speed Controller of DC Motor.

Author: PabloC
Date: 09/04/2024
Dependencies: <XSpaceV20.h>

Install the library dependencies
- XSpaceV20

To use the VSCode Serial Monitor with this project, update your platformio.ini file to include the following line:
monitor_speed = 1000000

This sets the baud rate to 1,000,000 bits per second, matching the configuration in the sketch.
*/


#include <Arduino.h>
#include <XSpaceV20.h>
#include <XSControl.h>

// Initialization of board and filter components
XSpaceV20Board XSBoard;
XSFilter Filter;
XSController Controller;

// Constants for the motor control configuration
#define PWM_FREQUENCY 20000 // Defines PWM frequency in Hz for motor control
#define ENCODER_RESOLUTION 960 // Specifies the resolution of the motor encoder
#define DRV8837_POWER_SUPPLY 5 // Defines the power supply voltage (in volts) for the DRV8837 motor driver

// Variables for storing the raw and filtered speed measurements
double speed; // Raw speed measurement
double filtered_speed; // Speed measurement after applying the low-pass filter

// Task dedicated to filtering the speed measurement from the motor's encoder
void SpeedFilter(void *pvParameters) {
  while (true) {
    // Measure speed in degrees per second from the encoder
    speed = XSBoard.GetEncoderSpeed(DEGREES_PER_SECOND);
    // Apply a second-order low-pass filter to the speed measurement
    filtered_speed = Filter.SecondOrderLPF(speed, 20, 0.001);
    // Delay of 1 ms between each measurement cycle
    vTaskDelay(1);
  }
  // Task cleanup, if ever exited
  vTaskDelete(NULL);
}

// Task for controlling the motor speed
void SpeedController(void *pvParameters) {
  double voltage;
  // Wake up the DRV8837 motor driver
  XSBoard.DRV8837_Wake();

  // PID controller parameters
  double Kp = 0.04; // Proportional gain
  double Ki = 0.2;  // Integral gain
  double Ts = 0.01; // Sampling time (in seconds)
  double SetPoint = 180; // Target speed in degrees per second

  while (true) {
    // Calculate the control signal (voltage) to be applied to the motor
    voltage = Controller.ControlLaw(filtered_speed, SetPoint, Kp, Ki, 0, Ts, FORWARD_EULER);
    // Apply the calculated voltage to the motor
    XSBoard.DRV8837_Voltage(voltage);
    // Output the filtered speed to the Serial Monitor for debugging
    Serial.println(filtered_speed);
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

  // Create Real-Time Operating System (RTOS) tasks for speed filtering and motor control
  xTaskCreate(SpeedFilter, "FilterTask", 2000, NULL, 1, NULL);
  xTaskCreate(SpeedController, "ControlTask", 2000, NULL, 1, NULL);
}

void loop() {
  // The main loop is intentionally left empty as task scheduling is handled by FreeRTOS.
}
