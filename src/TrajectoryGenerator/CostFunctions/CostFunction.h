//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_COSTFUNCTION_H
#define PATH_PLANNING_COSTFUNCTION_H

#endif //PATH_PLANNING_COSTFUNCTION_H

#include "../Trajectory.h"
#include "../../Vehicle.h"

#include <map>
#include <string>
#include <vector>

using namespace std;

class CostFunction {

public:
  /**
   * Calculates a cost (between 0 and 1) for a given trajectory.
   * @param trajectory Trajectory object which contains s and d coefficients.
   * @param target_vehicle_id ID of the vehicle which we are setting the trajectory relative to.
   * @param t desired time to reach the goal (relative to now as t=0)
   * @param vehicles vehicle dictionary, contains the position and speed information of all vehicles on the road.
   * @return
   */
  virtual double CalculateCost(Trajectory trajectory,
                               int target_vehicle_id,
                               double t,
                               map<int, Vehicle> vehicles);

  /**
   * A function that returns a value between 0 and 1 for x in the
   * range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
   * @param x
   */
  static double Logistic(double x) {
    return 2.0 / (1 + exp(-1 * x)) - 1.0;
  }

  static double NearestApproachToAnyVehicle(Trajectory trajectory, map<int, Vehicle> vehicles) {
    double closest = 999999;
    for(auto const v_map : vehicles) {
      double d = NearestApproach(trajectory, v_map.second);
      if(d < closest) {
        closest = d;
      }
    }

    return closest;
  }

  static double NearestApproach(Trajectory trajectory, Vehicle vehicle) {
    double closest = 999999;
    for(int i = 0; i < 100; i++) {
      double t = (double) i / 100 * trajectory.t;
      double cur_s = trajectory.S(t);
      double cur_d = trajectory.D(t);

      vector<vector<double>> target_state = vehicle.StateIn(t);
      double tar_s = target_state[0][0];
      double tar_d = target_state[1][0];
      double dist = sqrt(pow(cur_s - tar_s, 2) + pow(cur_d - tar_d, 2));
      if(dist < closest) {
        closest = dist;
      }
    }
    return closest;
  }
};