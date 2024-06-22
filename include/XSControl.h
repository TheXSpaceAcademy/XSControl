/*
    XSControl Library
    Author: Pablo Cardenas
    Description: An open-source library tailored for Arduino platforms, providing robust tools 
                 for implementing PID control algorithms. It supports various methods for 
                 integral and derivative approximation to meet diverse control needs.
    Note: Please include a reference to this library in your projects!
*/

#ifndef XSCONTROL_H
#define XSCONTROL_H

#include <Arduino.h>

// Defines methods for approximating the integral and derivative in control calculations
#define FORWARD_EULER 1  // Forward difference approximation for derivatives
#define BACKWARD_EULER 2 // Backward difference approximation for derivatives
#define TRAPEZOIDAL 3    // Trapezoidal rule for numerical integration

class XSController {
private:
    double e_1 = 0;         // Previous error value in the control loop
    double u_1 = 0;         // Previous control signal output
    double _integral = 0;   // Accumulated integral value for integration calculations

public:
   /*
   * Discrete Integrator
   * Computes the integral of the input signal using a discrete approximation method.
   * 
   * @param input The input signal to be integrated.
   * @param aprox_integral The selected method for integral approximation (FORWARD_EULER, BACKWARD_EULER, TRAPEZOIDAL).
   * @param Ts Sampling time in seconds.
   * @return The computed integral of the input signal.
   */
    double Discrete_Integrator(double input, int aprox_integral, double Ts);

    /* PI Control Law
       Computes the control signal using a Proportional-Integral control strategy.
       @param sensed_output The current measured output of the system.
       @param set_point The target output value for the system.
       @param Kp Proportional gain.
       @param Ki Integral gain.
       @param aprox_integral Selected method for integral approximation (FORWARD_EULER, BACKWARD_EULER, TRAPEZOIDAL).
       @param Ts Sampling time in seconds.
       @return The computed control signal to adjust the system output towards the set point.
    */
    double PI_ControlLaw(double sensed_output, double set_point, double Kp, double Ki, int aprox_integral, double Ts);

    /* PD Control Law
       Computes the control signal using a Proportional-Derivative control strategy.
       @param sensed_output The current measured output of the system.
       @param set_point The target output value for the system.
       @param Kp Proportional gain.
       @param Kd Derivative gain.
       @param aprox_derivative Selected method for derivative approximation (FORWARD_EULER, BACKWARD_EULER, TRAPEZOIDAL).
       @param N Filter coefficient for the derivative term to smooth derivative action.
       @param Ts Sampling time in seconds.
       @return The computed control signal to adjust the system output towards the set point.
    */
    double PD_ControlLaw(double sensed_output, double set_point, double Kp, double Kd, int aprox_derivative, double N, double Ts);

    /* PID Control Law
       Computes the control signal using a full Proportional-Integral-Derivative approach.
       @param sensed_output The current measured output of the system.
       @param set_point The target output value for the system.
       @param Kp Proportional gain.
       @param Ki Integral gain.
       @param aprox_integral Selected method for integral approximation (FORWARD_EULER, BACKWARD_EULER, TRAPEZOIDAL).
       @param Kd Derivative gain.
       @param aprox_derivative Selected method for derivative approximation (FORWARD_EULER, BACKWARD_EULER, TRAPEZOIDAL).
       @param N Filter coefficient for smoothing the derivative component.
       @param Ts Sampling time in seconds.
       @return The computed control signal to dynamically adjust the system output towards the set point.
    */
    double PID_ControlLaw(double sensed_output, double set_point, double Kp, double Ki, int aprox_integral, double Kd, int aprox_derivative, double N, double Ts);
};

class XSFilter {
	private:
		// Variables for first-order Low Pass Filter
		double yk_1 = 0; // Previous output value
		double uk_1 = 0; // Previous input value

		// Variables for second-order Low Pass Filter
		double uk2_1 = 0; // Input value at n-1
		double uk2_2 = 0; // Input value at n-2
		double yk2_1 = 0; // Output value at n-1
		double yk2_2 = 0; // Output value at n-2

	public:
		/* First Order Low Pass Filter
		   Filters an input signal using a first-order low pass filter configuration.
		   @param signal_input The input signal to be filtered.
		   @param freq Cutoff frequency in Hertz (Hz).
		   @param Ts Sampling interval in seconds.
		   @return The filtered output signal as a double.
		*/
		double FirstOrderLPF(double signal_input, double freq, double Ts);

		/* Second Order Low Pass Filter
		   Filters an input signal using a second-order low pass filter configuration.
		   @param signal_input The input signal to be filtered.
		   @param freq Cutoff frequency in Hertz (Hz).
		   @param Ts Sampling interval in seconds.
		   @return The filtered output signal as a double.
		*/
		double SecondOrderLPF(double signal_input, double freq, double Ts);
};

class XSData {
   private:

   public:
      void SignalAnalizer(double *muestras, double *time_m, double *muestras_pro, int no_muestras, double *max, double *min, double *freq);
};

#endif