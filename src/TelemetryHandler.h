//
// Created by Aaron Smith on 7/29/17.
//

#ifndef PATH_PLANNING_TELEMETRYHANDLER_H
#define PATH_PLANNING_TELEMETRYHANDLER_H

#include <vector>
#include "Vehicle.h"
#include "json.hpp"

using json = nlohmann::json;
using namespace std;

/**
 * Helper class which takes raw telemetry data and calculates and saves Ego telemetry data and sensor fusion data.
 * Keeps a map of all vehicles and their IDs and updates sensor fusion data for each vehicle when available.
 */
class TelemetryHandler {

public:
  map<int, Vehicle> vehicles = {};
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;

  /**
   * Processes telemetry data and saves data to the vehicles vector.
   * @param vehicles
   * @param event
   */
  void ProcessTelemetry(json telemetry_data);

  Vehicle GetEgo() {
    if (vehicles.count(-1) == 0) {
      throw "No data on Ego yet.";
    } else {
      return vehicles[-1];
    }
  }

  bool IsTelemetryEvent(json t) {
    return t[0].get<string>() == "telemetry";
  }
};


#endif //PATH_PLANNING_TELEMETRYHANDLER_H
