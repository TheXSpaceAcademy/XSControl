/*
  XSControl Library

  Version   :  1.0.4
  Autor     :  Pablo Cardenas
  Fecha     :  12/04/2024

  This is an open source library but dont remeber to reference!

*/

#include <XSControl.h>

/************************XSControl***************************/
double XSController::PI_ControlLaw(double sensed_output, double set_point, double Kp, double Ki, int aprox_integral, double Ts){
	double u = 0; // Control signal
	double e = set_point - sensed_output; // Current error

	// Calculate each component separately
	double P = Kp * e;  // Proportional component
	double I = 0;       // Integral component, to be calculated based on the approximation method
	double D = 0;       // Derivative component, to be calculated based on the approximation method

	switch (aprox_integral) {
		case FORWARD_EULER:
			// Update integral with the accumulated error using Forward Euler method
			this->_integral += this->e_1 * Ts; // Use the previous error for Forward Euler
			I = Ki * this->_integral; // Calculate Integral component
			break;
		case TUSTIN:
			// Using Tustin (trapezoidal approximation) for the Integral component
			this->_integral += (e + this->e_1) * Ts / 2; // Average of current and previous errors
			I = Ki * this->_integral; // Calculate Integral component using the trapezoidal rule
			break;
		case BACKWARD_EULER:
			// Update integral using Backward Euler method, which uses the current error
			this->_integral += e * Ts; // Use the current error for Backward Euler
			I = Ki * this->_integral; // Integral component calculated with Backward Euler
			break;
		default:
			// In case of an unsupported approximation method
			break;
	}

	// Sum P and I components to get the total control signal
	u = P + I;

	// Update previous values for the next iteration
	this->e_1 = e; // Update the previous error for next iteration's derivative calculation

	return u;
}

double XSController::PD_ControlLaw(double sensed_output, double set_point, double Kp, double Kd, int aprox_derivative, double N, double Ts){
	double u = 0; // Control signal
	double e = set_point - sensed_output; // Current error

	// Calculate each component separately
	double P = Kp * e;  // Proportional component
	double D = 0;       // Derivative component, to be calculated based on the approximation method

	switch (aprox_derivative) {
		case FORWARD_EULER:
			// Approximate derivative using Forward Euler method
			D = Kd * N * (e - e_1) / (1 + N * Ts);
			break;
		case TUSTIN:
			// Approximate derivative using Tustin method (bilinear transform)
			D = Kd * N * (2/Ts) * (e - e_1) / (2 + N * Ts);
			break;
		case BACKWARD_EULER:
			// Approximate derivative using Backward Euler method
			D = Kd * N * (e - e_1) / (Ts + N);
			break;
		default:
			// In case of an unsupported approximation method
			break;
	}

	// Sum P and D components to get the total control signal
	u = P + D;

	// Update previous values for the next iteration
	this->e_1 = e; // Update the previous error for next iteration's derivative calculation

	return u;
}

double XSController::PID_ControlLaw(double sensed_output, double set_point, double Kp, double Ki, int aprox_integral, double Kd, int aprox_derivative, double N, double Ts){
	double u = 0; // Control signal
	double e = set_point - sensed_output; // Current error

	// Calculate each component separately
	double P = Kp * e;  // Proportional component
	double I = 0;       // Integral component, to be calculated based on the approximation method
	double D = 0;       // Derivative component, to be calculated based on the approximation method

	switch (aprox_integral) {
		case FORWARD_EULER:
			// Update integral with the accumulated error using Forward Euler method
			this->_integral += this->e_1 * Ts; // Use the previous error for Forward Euler
			I = Ki * this->_integral; // Calculate Integral component
			break;
		case TUSTIN:
			// Using Tustin (trapezoidal approximation) for the Integral component
			this->_integral += (e + this->e_1) * Ts / 2; // Average of current and previous errors
			I = Ki * this->_integral; // Calculate Integral component using the trapezoidal rule
			break;
		case BACKWARD_EULER:
			// Update integral using Backward Euler method, which uses the current error
			this->_integral += e * Ts; // Use the current error for Backward Euler
			I = Ki * this->_integral; // Integral component calculated with Backward Euler
			break;
		default:
			// In case of an unsupported approximation method
			break;
	}

	switch (aprox_derivative) {
		case FORWARD_EULER:
			// Approximate derivative using Forward Euler method
			D = Kd * N * (e - e_1) / (1 + N * Ts);
			break;
		case TUSTIN:
			// Approximate derivative using Tustin method (bilinear transform)
			D = Kd * N * (2/Ts) * (e - e_1) / (2 + N * Ts);
			break;
		case BACKWARD_EULER:
			// Approximate derivative using Backward Euler method
			D = Kd * N * (e - e_1) / (Ts + N);
			break;
		default:
			// In case of an unsupported approximation method
			break;
	}

	// Sum P, I, D components to get the total control signal
	u = P + I + D;

	// Update previous values for the next iteration
	this->e_1 = e; // Update the previous error for next iteration's derivative calculation

	return u;
}

double XSFilter::FirstOrderLPF(double signal_input, double freq, double Ts){
	double uk = signal_input;
	double Tau = 1/(2*PI*freq);
	double yk = Ts/Tau*uk_1 - (Ts-Tau)/Tau*yk_1;
	uk_1 = uk;
	yk_1 = yk;

	return yk;
}

double XSFilter::SecondOrderLPF(double signal_input, double freq, double Ts){
	double uk2 = signal_input;
	double a = freq*(2*PI);
	double yk2 = a*a*Ts*Ts*uk2_2 - (2*a*Ts-2)*yk2_1 - (a*a*Ts*Ts+1-2*a*Ts)*yk2_2;

	uk2_2 = uk2_1;
	yk2_2 = yk2_1;
	uk2_1 = uk2;
	yk2_1 = yk2;

	return yk2;
}