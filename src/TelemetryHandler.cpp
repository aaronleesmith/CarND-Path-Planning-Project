//
// Created by Aaron Smith on 7/29/17.
//

#include "TelemetryHandler.h"

void TelemetryHandler::ProcessTelemetry(json telemetry_data) {
  /**
   * Process data for ego.
   */
  double s = telemetry_data[1]["s"];
  double d = telemetry_data[1]["d"];
  double x = telemetry_data[1]["x"];
  double y = telemetry_data[1]["y"];
  double yaw = telemetry_data[1]["yaw"];
  double speed = ((double) telemetry_data[1]["speed"]) / 2.23694;

  if(vehicles.count(-1) == 0) {
    Vehicle our_car = Vehicle();
    our_car.init(-1, s, d, x, y, yaw, speed);
    vehicles[our_car.id] = our_car;
  } else {
    vehicles[-1].update(s, d, x, y, yaw, speed);
  }

  auto previous_path_x_ = telemetry_data[1]["previous_path_x"];
  auto previous_path_y_ = telemetry_data[1]["previous_path_y"];

  previous_path_x.clear();
  previous_path_y.clear();

  for(int i = 0; i < previous_path_x_.size(); i++) {
    previous_path_x.push_back(previous_path_x_[i]);
    previous_path_y.push_back(previous_path_y_[i]);
  }

  end_path_s = (double) telemetry_data[1]["end_path_s"];
  end_path_d = (double) telemetry_data[1]["end_path_d"];

  /**
   * Process data for other cars on the road.
   */
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry_data[1]["sensor_fusion"];

  for(int i = 0; i < sensor_fusion.size(); i++ ) {
    auto sf = sensor_fusion[i];

    int id = sf[0];
    double s = sf[5];
    double d = sf[6];
    double x = sf[1];
    double y = sf[2];
    double vx = sf[3];
    double vy = sf[4];
    double yaw = atan2(vy, vx);
    double speed = sqrt(pow(vx, 2) + pow(vy, 2));

    if(vehicles.count(sf[0]) == 0) {
      Vehicle v = Vehicle();
      v.init(id, s, d, x, y, yaw, speed, vx, vy);
      vehicles[v.id] = v;
    } else {
      vehicles[sf[0]].update(s, d, x, y, yaw, speed, vx, vy);
    }
  }
}