#include<cmath>
#include"../include/ROS_opencv_improcess/PIDcontroller.hpp"



double PIDcontroller::calculatePID(double error) {
// proporatoinal term
double Pout = Pterm(error);
// integrate term
double Iout=Iterm(error);
// derivative term
double Dout = Dterm(error);
// calculate the total term
double TotalOut = Pout + Iout + Dout;

return TotalOut;
}

double PIDcontroller::Pterm(double error) {
  return Kp * error;
}

double PIDcontroller::Iterm(double error) {
   integral += error * dt;
   return Ki * integral;
}
double PIDcontroller::Dterm(double error) {
  double derivative = (error - pre_error) / dt;
  pre_error = error;
  return Kd * derivative;
}
