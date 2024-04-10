/*
  XSControl Library

  Version   :  1.0.0
  Autor     :  Pablo Cardenas
  Fecha     :  09/04/2024

  This is an open source library but dont remeber to reference!

*/

#include <XSControl.h>

/************************XSControl***************************/
double XSController::ControlLaw(double sensed_output, double set_point, double Kp, double Ki,double Ts, int aprox){
    double u=0;

    double e = set_point - sensed_output; 

    switch (aprox)
    {
    case FORWARD_EULER:
        u = 0;
        break;
    case TUSTIN:
        u = (Kp+Ts/2*Ki)*e + (Ts/2*Ki-Kp)*this->e_1 + this->u_1;
        this->e_1 = e;
        this->u_1 = u;
        break;
    
    default:
        break;
    }

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