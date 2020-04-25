#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double steer_value_old = 0;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
//Start twiddle code
bool twiddle_on_ = false;
double twiddle_best_error_ = 1000000;
bool twiddle_state_ = 0;
int twiddle_idx = 0;
int twiddle_iterations_ = 0;
std::vector<double> p = {0.27, 0.001, 3.0};
std::vector<double> dp = {0.05, 0.001, 0.05};
void twiddle(PID &pid_control) {
  std::cout << "State: " << twiddle_state_ << std::endl;
  std::cout << "PID Error: " << pid_control.TotalError() << ", Best Error: " << twiddle_best_error_ << std::endl;
  if (twiddle_state_ == 0) {
    twiddle_best_error_ = pid_control.TotalError();
    p[twiddle_idx] += dp[twiddle_idx];
    twiddle_state_ = 1;
  } 
  } else { //twiddle_state_ = 2
    if (pid_control.TotalError() < twiddle_best_error_) {
      twiddle_best_error_ = pid_control.TotalError();
      dp[twiddle_idx] *= 1.1;
      twiddle_idx = (twiddle_idx + 1) % 3;
      p[twiddle_idx] += dp[twiddle_idx];
      twiddle_state_ = 1;
    } else {
      p[twiddle_idx] += dp[twiddle_idx];
      dp[twiddle_idx] *= 0.9;
      twiddle_idx = (twiddle_idx + 1) % 3;
      p[twiddle_idx] += dp[twiddle_idx];
      twiddle_state_ = 1;
      //pid.Init(p[0], p[1], p[2]);
    }
  }
  pid_control.Init(p[0], p[1], p[2]);
}
// End Twiddle

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  double init_kp = 0.2;
  double init_ki = 0.0003;
  double init_kd = 10.;
  pid.Init(init_kp, init_ki, init_kd);
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double tht;
          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if(std::abs(steer_value - steer_value_old) < 0.2)
          {
            steer_value = steer_value_old;
          }
          else if(steer_value > 1)
          {
            steer_value = 0.5;
            steer_value_old = 0.5;
          }
          else if(steer_value<-1)
          {
            steer_value = -0.5;
            steer_value_old = -0.5;
          }
          else
          {
            steer_value_old =steer_value;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.1;// (1 - std::abs(steer_value)) * 0.5 + 0.2;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}