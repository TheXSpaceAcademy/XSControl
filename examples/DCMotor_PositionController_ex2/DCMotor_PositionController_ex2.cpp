/*
This example demonstrates a Proportional (P) Position Controller for a DC Motor, ideal for precision control tasks.
The controller reads an encoder's position and adjusts motor voltage to maintain target position.

System Block Diagram:

             PositionController              DC Motor
                   ------           --------------------------
                                    +---------+        +-----+
           +       +----+   voltage |    b    | speed  |  1  |  position
  pos_ref--->o---->| Kp |---------->| ------- |------->| --- |--------->
             ^ -   +----+           |  s + a  |        |  s  |    |
             |                      +---------+        +-----+    |
             |                                                    |
             +----------------------------------------------------+

Author: PabloC
Date: 09/04/2024

Install the library dependencies
- XSpaceV20

Update your platformio.ini file to include the following line:

monitor_speed = 1000000
*/

#include <Arduino.h>         // Include the Arduino framework for managing I/O operations.
#include <XSpaceV20.h>       // Include support for XSpaceV20 motor control board functionalities.
#include <XSControl.h>       // Include additional control interfaces or utilities specific to the XS board series.

// Initialize objects for board and controller interfacing.
XSpaceV20Board XSBoard;
XSController Controller;

// Motor control configuration.
#define PWM_FREQUENCY 20000        // Hz, higher values yield smoother motor operation.
#define ENCODER_RESOLUTION 1280    // Higher values increase position measurement accuracy.
#define DRV8837_POWER_SUPPLY 5     // Voltage in volts, do not exceed motor driver specs.

// Motor position control task, continuously adjusts motor voltage based on encoder feedback.
void PositionController(void *pvParameters) {
  double voltage;   // Voltage to apply to motor (V).
  double position;  // Current encoder position (degrees).

  // Controller settings.
  double Kp = 0.6;          // Proportional gain.
  double pos_ref = 180;     // Target position (degrees).

  XSBoard.DRV8837_Wake();   // Ensure motor driver is active.

  while (true) {
    position = XSBoard.GetEncoderPosition(E1, DEGREES);
    voltage = (pos_ref - position) * Kp; // Control signal calculation.
    XSBoard.DRV8837_Voltage(voltage);    // Apply voltage to motor.
    
    Serial.println(position);            // Debugging: Output current position.
    vTaskDelay(10);                      // Control cycle timing (10 ms).
  }
  // Task cleanup.
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(1000000);  // Fast data transfer for debugging.
  XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, DRV8837_POWER_SUPPLY);
  xTaskCreate(PositionController, "ControlTask", 2000, NULL, 2, NULL);
}

void loop() {
  // Main loop remains empty; RTOS handles the task scheduling.
}