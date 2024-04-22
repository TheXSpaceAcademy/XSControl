/**
 * @file    XSpaceBioV10_AD8232_SimpleECG_with_filter.cpp
 * @brief   Electrocardiogram (ECG) Signal Acquisition Using an AD8232 integrated in XSpace Bio v1.0
 * 
 * 

Author: PabloC
Date: 21/04/2024
Dependencies: <XSpaceBioV10.h>

Install the library dependencies
- XSpaceBioV10
*/

#include <Arduino.h>
#include <XSpaceBioV10.h>
#include <XSControl.h>

// Global variables to store raw and filtered ECG values
double raw_ecg = 0;
double filtered_ecg = 0;

// Create an instance of the XSpaceBioV10Board to interact with the board
XSpaceBioV10Board Board;

// Create an instance of the XSFilter for signal filtering
XSFilter Filter;

/**
 * @brief   Task function for filtering ECG signal
 * 
 * This task continuously reads the raw ECG voltage from the AD8232,
 * applies a second-order low-pass filter, and stores the filtered value.
 */
void FilterTask(void *pv) {
  while (1) {
    // Read raw ECG voltage from AD8232 module
    raw_ecg = Board.AD8232_GetVoltage(AD8232_XS1);
    // Apply second-order low-pass filter with cutoff frequency of 40Hz and sample time of 0.001s
    filtered_ecg = Filter.SecondOrderLPF(raw_ecg, 40, 0.001);
    // Delay task execution for 1 millisecond
    vTaskDelay(1);
  }
  // Delete task (never reached)
  vTaskDelete(NULL);
}

void setup() {
  // Initialize serial communication with the computer at baud rate 115200
  Serial.begin(115200);
  // Perform initial setup for the XSpace Bio v1.0 board
  Board.init();
  // Activate the AD8232 sensor at slot XS1 to start monitoring
  Board.AD8232_Wake(AD8232_XS1);
  // Create filtering task with stack size of 3000 bytes
  xTaskCreate(FilterTask, "FilterTask", 3000, NULL, 1, NULL);
}

void loop() {
  // Print raw and filtered ECG values to Serial Monitor
  Serial.println((String)raw_ecg + " " + (String)filtered_ecg);

  // Pause the loop for 10 milliseconds to control the data rate output
  delay(10);
}