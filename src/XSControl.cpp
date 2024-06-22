/*
  XSControl Library
  Autor     :  Pablo Cardenas
  This is an open source library but dont remeber to reference!
*/

#include <XSControl.h>

/************************XSController***************************/
double XSController::Discrete_Integrator(double input, int aprox_integral, double Ts){
	double I = 0;
	double e = input;

	switch (aprox_integral) {
		case FORWARD_EULER:
			// Update integral with the accumulated error using Forward Euler method
			this->_integral += this->e_1 * Ts; // Use the previous error for Forward Euler
			I = this->_integral; // Calculate Integral component
			break;
		case TRAPEZOIDAL:
			// Using Tustin (trapezoidal approximation) for the Integral component
			this->_integral += (e + this->e_1) * Ts / 2; // Average of current and previous errors
			I = this->_integral; // Calculate Integral component using the trapezoidal rule
			break;
		case BACKWARD_EULER:
			// Update integral using Backward Euler method, which uses the current error
			this->_integral += e * Ts; // Use the current error for Backward Euler
			I = this->_integral; // Integral component calculated with Backward Euler
			break;
		default:
			// In case of an unsupported approximation method
			break;
	}
	this->e_1 = e;
	return I;
}


double XSController::PI_ControlLaw(double sensed_output, double set_point, double Kp, double Ki, int aprox_integral, double Ts){
	double u = 0; // Control signal
	double e = set_point - sensed_output; // Current error

	// Calculate each component separately
	double P = Kp * e;  // Proportional component
	double I = 0;       // Integral component, to be calculated based on the approximation method

	switch (aprox_integral) {
		case FORWARD_EULER:
			// Update integral with the accumulated error using Forward Euler method
			this->_integral += this->e_1 * Ts; // Use the previous error for Forward Euler
			I = Ki * this->_integral; // Calculate Integral component
			break;
		case TRAPEZOIDAL:
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
		case TRAPEZOIDAL:
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
		case TRAPEZOIDAL:
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
		case TRAPEZOIDAL:
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


/************************XSFilter***************************/
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

/************************XSData***************************/
void XSData::SignalAnalizer(double *muestras, double *time_m, double *muestras_pro, int no_muestras, double *max, double *min, double *freq){
  double temp_max = 0;
  double temp_min = 10000000000000;
  
	for(int i=0; i<no_muestras; i++){
		if(muestras[i]>temp_max)temp_max = muestras[i];
		if(muestras[i]<temp_min)temp_min = muestras[i];
	}

	double moffset = (temp_max+temp_min)/2;

	for(int i=0; i<no_muestras; i++){
		muestras_pro[i] =  muestras[i] - moffset;
	}

  *max = temp_max-moffset;
  *min = temp_min-moffset;

  int Posiciones[50];
  int j=0;
  bool numx=true;

  for(int i=5; i<no_muestras; i++){
    if( (muestras_pro[i]>0) && muestras_pro[i-5]<0  && numx==true){
      Posiciones[j] = time_m[i];
	//muestras_pro[i]=10000;
      j++;
      numx=false;
    }
    if(muestras_pro[i]<0 && !numx) numx=true;
  }

  double SumaPeriodos = 0;
  for(int k=j-1; k>0; k--){
    SumaPeriodos = SumaPeriodos + (double)(Posiciones[k]-Posiciones[k-1]);
  }
	SumaPeriodos = SumaPeriodos/1000000;
  
	//muestras_pro[0]=20000;
	//muestras_pro[no_muestras-1]=-20000;

  //*freq = ((double)(j-1))/SumaPeriodos;

}