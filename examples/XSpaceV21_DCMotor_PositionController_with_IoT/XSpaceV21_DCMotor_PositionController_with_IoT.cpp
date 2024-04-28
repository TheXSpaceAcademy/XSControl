/*
This example demonstrates how to implement a Speed-Position Controller of DC Motor (Cascade) with MQTT for changing the position set_point

        Thing
    Mqtt Publisher ID: Unknown   
          |Topic: mqttfx/set_point
          |
          ↓
   (XSpace MQTT Server) domain: www.xspace.pe   port: 1883  
          |
          |  
          ↓Topic: mqttfx/set_point
      XSpace V2.1
    MQTT Suscriber ID: the_xspacer
          |                                                                            DC Motor
          |                                          SpeedController        +------------------------------+
          |          PositionController                +----------+         |  +---------+        +-----+  |
          ↓       +       +-----+  speed_ref  +        |      Ki  | voltage |  |    b    | speed  |  1  |  |   position
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
Date: 28/04/2024

Install the library dependencies
- XSpaceV21
- XSControl
- XSpaceIoT


Update your platformio.ini file to include the following line:

monitor_speed = 1000000
*/

#include <Arduino.h>
#include <XSpaceV21.h>
#include <XSControl.h>
#include <XSpaceIoT.h>

// Constants for Wi-Fi SSID, Wifi password and MQTT id
const char* WIFI_SSID = "redpucp";
const char* WIFI_PASSWORD = "C9AA28BA93";
const char* MQTT_ID = "the_xspacer"; // MQTT client identifier (MUST BE UNIQUE to prevent connection issues)

// Definition of board, filter and controller objects
XSpaceV21Board XSBoard;
XSFilter Filter;
XSController Controller;
XSThing Thing;

// Constants for the motor control configuration
#define PWM_FREQUENCY 20000 // Defines PWM frequency in Hz for motor control
#define ENCODER_RESOLUTION 960 // Specifies the resolution of the motor encoder
#define DRV8837_POWER_SUPPLY 5 // Defines the power supply voltage (in volts) for the DRV8837 motor driver

// Variables for storing the raw and filtered speed measurements
double position; // Raw position measurement
double speed; // Raw speed measurement
double filtered_speed; // Speed measurement after applying the filter
double speed_ref = 0;
double pos_ref = 0; // Target position in degrees

void Mqtt_TopicAnalizer(char* topicx, byte* Data, unsigned int DataLen){
  
  // Convert received data and topic to strings
  String ReceivedData = String((char*)Data, DataLen);
  String Topic = String((char*)topicx);

  // Check if the received data is from the topic "mqttfx/set_point"
  if(Topic == "mqttfx/set_point"){
    // Print the received data to the serial monitor
    pos_ref = ReceivedData.toDouble();
  }
}

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
  double Kp2 = 2.5; // Proportional gain
  double Ts = 0.01; // Sampling time (in seconds)

  while (true) {
    position = XSBoard.GetEncoderPosition(E1,DEGREES);
    // Calculate the control signal (speed_reef) to be applied to the spped control loop
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
  XSBoard.DRV8837_Wake(DRVx1);

  // PI controller parameters
  double Kp = 0.0610; // Proportional gain
  double Ki  = 0.0610; // Integral gain
  double Ts  = 0.01; // Sampling time (in seconds)

  while (true) {
    // Calculate the control signal (voltage) to be applied to the motor
    voltage = Controller.PI_ControlLaw(filtered_speed, speed_ref, Kp, Ki, TRAPEZOIDAL, Ts);
    // Apply the calculated voltage to the motor
    XSBoard.DRV8837_Voltage(DRVx1,voltage);
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

  Thing.Mqtt_SerialInfo(true); 
  Thing.Mqtt_init("www.xspace.pe",1883,Mqtt_TopicAnalizer,100);
  Thing.Mqtt_Connect(WIFI_SSID, WIFI_PASSWORD, MQTT_ID);

  Thing.Mqtt_Suscribe("mqttfx/set_point");


  // Create Real-Time Operating System (RTOS) tasks for filtering, speed and position control
  xTaskCreate(SpeedFilter, "FilterTask", 2000, NULL, 1, NULL);
  xTaskCreate(SpeedController, "ControlTask", 2000, NULL, 2, NULL);
  xTaskCreate(PositionController, "ControlTask", 2000, NULL, 2, NULL);
}

void loop() {
  // The main loop is intentionally left empty as task scheduling is handled by FreeRTOS.
}