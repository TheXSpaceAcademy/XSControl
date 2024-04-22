/*
This example demonstrates how to implement a PI+P Speed-Position Controller of DC Motor (Cascade)

                                                                                 DC Motor
                                              SpeedController        +------------------------------+
              PositionController                +----------+         |  +---------+        +-----+  |
           +       +-----+  speed_ref  +        |      Ki  | voltage |  |    b    | speed  |  1  |  |   position
 pos_ref --->o---->| Kp2 |-------------->o----->| Kp + --- |-------->|--|  -----  |------->| --- |--|-------> 
             ^ -   +-----+               ^ -    |       s  |         |  |  s + a  |   |    |  s  |  |  |
             |       10ms                |      +----------+         |  +---------+   |    +-----+  |  |
             |                           |          10ms             +----------------|-------------+  |
             |                           |                                            |                |
             |                           |  filtered_speed  +-------------+           |                |
             |                           +------------------| SpeedFilter |<----------+                |
             |                                              +-------------+                            |
             |                                                    1ms                                  |
             +-----------------------------------------------------------------------------------------+

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
XSFilter Filter;
XSController Controller;

// Constants for the motor control configuration
#define PWM_FREQUENCY 20000 // Defines PWM frequency in Hz for motor control
#define ENCODER_RESOLUTION 1280 // Specifies the resolution of the motor encoder
#define DRV8837_POWER_SUPPLY 5 // Defines the power supply voltage (in volts) for the DRV8837 motor driver

// Variables for storing the raw and filtered speed measurements
double position; // Raw position measurement
double speed; // Raw speed measurement
double filtered_speed; // Speed measurement after applying the filter
double speed_ref = 0;

// Task dedicated to filtering the speed measurement from the motor's encoder
void SpeedFilter(void *pvParameters) {
  while (true) {
    // Measure speed in degrees per second from the encoder
    speed = XSBoard.GetEncoderSpeed(E1,DEGREES_PER_SECOND);
    // Apply a second-order low-pass filter to the speed measurement
    filtered_speed = Filter.SecondOrderLPF(speed, 20, 0.001);
    // Delay of 1 ms between each measurement cycle
    vTaskDelay(1);
  }
  // Task cleanup, if ever exited
  vTaskDelete(NULL);
}


// Task for controlling the motor position
void PositionController(void *pvParameters) {
  // PID controller parameters
  double Kp2 = 2.78; // Proportional gain
  double Ts = 0.01; // Sampling time (in seconds)
  double pos_ref = 180; // Target position in degrees

  while (true) {
    position = XSBoard.GetEncoderPosition(E1,DEGREES);
    // Calculate the control signal (voltage) to be applied to the motor
    speed_ref = (pos_ref - position)*Kp2;

    // Output the DC Motor position to the Serial Monitor for debugging
    Serial.println(position);

    // Delay of 10 ms between control cycles
    vTaskDelay(10);
  }
  // Task cleanup, if ever exited
  vTaskDelete(NULL);
}

// Task for controlling the motor speed
void SpeedController(void *pvParameters) {
  double voltage;
  // Wake up the DRV8837 motor driver
  XSBoard.DRV8837_Wake();

  // PI controller parameters
  double Kp = 0.00489; // Proportional gain
  double Ki  = 0.02445; // Integral gain
  double Ts  = 0.01; // Sampling time (in seconds)

  while (true) {
    // Calculate the control signal (voltage) to be applied to the motor
    voltage = Controller.PI_ControlLaw(filtered_speed, speed_ref, Kp, Ki, BACKWARD_EULER, Ts);
    // Apply the calculated voltage to the motor
    XSBoard.DRV8837_Voltage(voltage);
    // Delay of 10 ms equal to the sampling time (Ts)
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

  // Create Real-Time Operating System (RTOS) tasks for filtering, speed and position control
  xTaskCreate(SpeedFilter, "FilterTask", 2000, NULL, 2, NULL);
  xTaskCreate(SpeedController, "ControlTask", 2000, NULL, 2, NULL);
  xTaskCreate(PositionController, "ControlTask", 2000, NULL, 2, NULL);
}

void loop() {
  // The main loop is intentionally left empty as task scheduling is handled by FreeRTOS.
}