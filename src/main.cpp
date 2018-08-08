#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"
#include "poly.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void globalPointsToVehiclePoints(vector<double>& ptsx, vector<double>& ptsy,
  double px, double py, double psi) {
    double cos_psi = cos(psi);
    double sin_psi = sin(psi);
    for (int i = 0; i < ptsx.size(); ++i) {
      double dx = ptsx[i] - px;
      double dy = ptsy[i] - py;
      ptsx[i] = dx * cos_psi + dy * sin_psi;
      ptsy[i] = - dx * sin_psi + dy * cos_psi;
    }
}

double calculateInitialCte(Eigen::VectorXd coeffs, double y = 0) {
  return y - polyeval(coeffs, 0);
}

double calculateInitialEpsi(Eigen::VectorXd coeffs, double psi = 0) {
  Eigen::VectorXd deriv(coeffs.size());
  for (int i = 1; i < coeffs.size(); ++i) {
    deriv(i - 1) = coeffs(i) * i;
  }
  return psi - atan(polyeval(coeffs, 0));
}

int main() {
  using namespace Eigen;

  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  const double steer_factor = 1.0 / MPC::steering_limit_rad;
  const double expected_latency = 0.12;

  h.onMessage([&mpc, &steer_factor, &expected_latency](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {

        auto timing_start = std::chrono::high_resolution_clock::now();

        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta0 = j[1]["steering_angle"];
          double a0 = j[1]["throttle"];

          // in map coordinate system
          delta0 *= MPC::steering_limit_rad;
          VectorXd initialStateWithLatency(4);
          initialStateWithLatency = globalKinematicStep(px, py, psi, v, expected_latency, delta0, a0);

          globalPointsToVehiclePoints(ptsx, ptsy, initialStateWithLatency(0),
            initialStateWithLatency(1), initialStateWithLatency(2));
          VectorXd xvals = Map<VectorXd, Unaligned>(ptsx.data(), ptsx.size());
          VectorXd yvals = Map<VectorXd, Unaligned>(ptsy.data(), ptsy.size());

          // in vehicle coordinate system, simulated latency
          VectorXd coeffs = polyfit(xvals, yvals, 3);
          // x, y, psi, v, cte, epsi
          VectorXd solverInit(6);
          solverInit << 0, 0, 0,
            initialStateWithLatency(3),
            calculateInitialCte(coeffs),
            calculateInitialEpsi(coeffs);

          vector<double> sol = mpc.Solve(solverInit, coeffs);

          double steer_value = sol[0] * steer_factor;
          double throttle_value = sol[1];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          cout << "steering_angle: " << steer_value << "\t" << "throttle: " << throttle_value << endl;

          msgJson["mpc_x"] = mpc.solx;
          msgJson["mpc_y"] = mpc.soly;

          msgJson["next_x"] = ptsx;
          msgJson["next_y"] = ptsy;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

        auto timing_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = timing_finish - timing_start;
        // cout << "latency ~= " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << endl;

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
