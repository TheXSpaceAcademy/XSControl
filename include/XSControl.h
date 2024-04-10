/*
  XSControl Library

  Version   :  1.0.2
  Author    :  Pablo Cardenas
  Date      :  09/04/2024

  This is an open source library. Please remember to reference it if you use it in your projects!

*/

#ifndef XSCONTROL_H
#define XSCONTROL_H

#include <Arduino.h>
#include <stdint.h>

#define FORWARD_EULER 1
#define BACKWARD_EULER 2
#define TUSTIN 3

class XSController{
  private:
    double e_1 = 0; // Previous error value
    double u_1 = 0; // Previous control signal value
    double _integral=0;
  public:

    /* Control System Algorithm
       Parameters:
       sensed_output => The current sensed value of the system output.
       set_point => The desired value (reference) for the system output.
       Kp => Proportional gain.
       Ki => Integral gain.
       Kd => Derivative gain.
       Ts => Sample time of the control system in seconds.
       aprox => Method of approximation for discrete controller (FORWARD_EULER, TUSTIN).

       Returns:
       The control signal to be applied to the system.
    */
    double ControlLaw(double sensed_output, double set_point, double Kp, double Ki, double Kd, double Ts, int aprox);

};

class XSFilter{
  private:
    // LPF 1st order
    double yk_1 = 0; // Previous output value
    double uk_1 = 0; // Previous input value

    // LPF 2nd order
    double uk2_1 = 0; // First previous input value
    double uk2_2 = 0; // Second previous input value
    double yk2_1 = 0; // First previous output value
    double yk2_2 = 0; // Second previous output value

  public:
    /* First order Low Pass Filter
       Parameters:
       signal_input => The signal input to be filtered.
       freq => Cutoff frequency in Hertz (Hz).
       Ts => Sample time of the filter in seconds.

       Returns:
       The filtered signal as a double.
    */
    double FirstOrderLPF(double signal_input, double freq, double Ts);

    /* Second order Low Pass Filter
       Parameters:
       signal_input => The signal input to be filtered.
       freq => Cutoff frequency in Hertz (Hz).
       Ts => Sample time of the filter in seconds.

       Returns:
       The filtered signal as a double.
    */
    double SecondOrderLPF(double signal_input, double freq, double Ts);
};
#endif