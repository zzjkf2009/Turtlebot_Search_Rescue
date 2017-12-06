#include<cmath>
#include"../include/ROS_opencv_improcess/PIDcontroller.hpp"



double PIDcontroller::calculatePID(double error) {
// proporatoinal term
double Pout = Kp * error;
// integral term
 integral += error * dt;
double Iout = Ki * integral;
// derivative term
double derivative = (error - pre_error) / dt;
double Dout = Kd * derivative;
// calculate the total term
double TotalOut = Pout + Iout + Dout;

pre_error = error;

return TotalOut;
}
