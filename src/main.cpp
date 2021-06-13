#include <uWS/uWS.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "path_planner.h"
#include"helpers.h"
#include "json.hpp"
#include "interface.h"
#include"map.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using interfaces::input;
using interfaces::output;
using interfaces::SnsFusionData;
using interfaces::PreviousOutputs;
using interfaces::EgoData;
using interfaces::mapData;
using path_planner::PathPlanner;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
int main() {
 uWS::Hub h;
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  path_planner::Map mMap;
  mMap.read(map_file_);
  SnsFusionData snsFus;
  PathPlanner pth(snsFus);
  bool initialized = false;
  int count =0;
  input Input{0};
  output Output;
  Input.currentData.car_target_speed=4.0;
  Input.currentData.car_danger=false;
  h.onMessage([&initialized,&snsFus,&mMap,&Input,&Output,&count ,&pth]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          Input.currentData.car_x = j[1]["x"];
          Input.currentData.car_y = j[1]["y"];
          Input.currentData.car_s = j[1]["s"];
          Input.currentData.car_d = j[1]["d"];
          Input.currentData.car_yaw = j[1]["yaw"];
          Input.currentData.car_speed = j[1]["speed"];
          Output.next_x_vals.clear();
          Output.next_y_vals.clear();
		      // Previous path data given to the Planner
          Input.prevOp.previous_path_x=j[1]["previous_path_x"].get<vector<double>>();
          Input.prevOp.previous_path_y=j[1]["previous_path_y"].get<vector<double>>();
          // Previous path's end s and d values 
          Input.prevOp.end_path_s = j[1]["end_path_s"];
          Input.prevOp.end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"].get<vector<vector<double>>>();
          snsFus.raw_sensor_fusion=sensor_fusion;
          Input.snsFusion=snsFus;
          std::vector<double> frenet_car = mMap.getFrenet( Input.currentData.car_x,  Input.currentData.car_y, helper::deg2rad( Input.currentData.car_yaw));
          Input.currentData.car_s = frenet_car[0];
          Input.currentData.car_d = frenet_car[1];
          Input.currentData.car_lane=helper::getLane(Input.currentData.car_d);
          json msgJson;
          //
          if (!initialized)
          {
              pth.init(Input,mMap);
              initialized = true;
          }
          auto start = std::chrono::steady_clock::now();
          auto Adjust = pth.process(Input,mMap,Output);
          // update the target values.
          Input.currentData.car_target_speed=Adjust.velocity;
          Input.currentData.car_danger=Adjust.danger;
          auto stop = std::chrono::steady_clock::now();
          count++;
          if(0)
          {
            std::cout << "t(us): "<< 
            std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()
            <<"µs"<< std::endl;
          }
          //try_it_out(Input,Output);
          msgJson["next_x"] = Output.next_x_vals;
          msgJson["next_y"] = Output.next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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