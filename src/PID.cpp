#include "PID.h"
#include <uWS/uWS.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}
/*
std::vector<double> twiddle(std::vector<double> &p, double cte, double speed) {
  double tol = 0.2;
  double max_steering_angle = pi/4.0;
  std::vector<double> dp = [0, 0, 0];
  PID pid;
  pid.UpdateError(cte);


}

std::vector<double> PID::twiddle(double cte) {

}

*/

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
