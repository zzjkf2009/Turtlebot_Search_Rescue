#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

class PIDcontroller {
private:
 double dt = 0.66;
 double Kp = 0.002;
 double Ki = 0.001;
 double Kd = 0.00;
 double pre_error = 0;
 double integral = 0;
public:
double calculatePID(double error);
};

#endif // _PIDCONTROLLER_H_
