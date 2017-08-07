#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "Car.h"
#include "BehaviorPlanner.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of('}');
  if (found_null != string::npos) {
    return "";
  }
  if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  BehaviorPlanner planner;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {


    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto dataValue = hasData(data);

      if (!dataValue.empty()) {
        auto j = json::parse(dataValue);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // car data
          double carX         = j[1]["x"];
          double carY         = j[1]["y"];
          double carS         = j[1]["s"];
          double carD         = j[1]["d"];
          double carYaw       = j[1]["yaw"];
          double carSpeed     = j[1]["speed"];

          // previous path data
          auto previousPathX = j[1]["previous_path_x"];
          auto previousPathY = j[1]["previous_path_y"];
          // previous end path data
          double endPathS    = j[1]["end_path_s"];
          double endPathD    = j[1]["end_path_d"];

          auto sensorFusion   = j[1]["sensor_fusion"];

          json msgJson;

          // path vectors
          vector<double> nextXVals;
          vector<double> nextYVals;

          int previousPathLength = previousPathX.size();

          planner.initState(carS, carD, previousPathLength);
          planner.decideState(sensorFusion);
          planner.perform(sensorFusion);

          // add points for spline including previously generated points
          std::vector<double> Xpts, Ypts, Tpts;

          if (previousPathLength > 0) {
            for (int i = 0; i < 2; i++) {
              // current coordinates
              Tpts.push_back(i * step);
              Xpts.push_back(previousPathX[i]);
              Ypts.push_back(previousPathY[i]);
            }
          } else {
            Tpts.push_back(0);
            Xpts.push_back(carX);
            Ypts.push_back(carY);
          }

          // create spline for smoothness
          for (double t = 3 * trajectoryLength / 4.0; t <= trajectoryLength; t += trajectoryLength / 8.0) {
            double s = polyEval(planner.sCoeffs, t);
            double d = polyEval(planner.dCoeffs, t);
            if (s > maximumS) s -= maximumS;
            vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            Xpts.push_back(xy[0]);
            Ypts.push_back(xy[1]);
            Tpts.push_back(t);
          }

          planner.trajectorySmooth(nextXVals, nextYVals, Tpts, Xpts, Ypts);

          msgJson["next_x"] = nextXVals;
          msgJson["next_y"] = nextYVals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
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
