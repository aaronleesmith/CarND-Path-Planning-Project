//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_MAXJERKCOSTFUNCTION_H
#define PATH_PLANNING_MAXJERKCOSTFUNCTION_H


#include "CostFunction.h"

class MaxJerkCostFunction: public CostFunction {

public:
  static const double MAX_JERK = 10;

  double CalculateCost(Trajectory trajectory,
                       int target_vehicle_id,
                       double t,
                       map<int, Vehicle> vehicles) {
    double max_jerk = -1;
    double dt = trajectory.t / 100.0;
    for(int i = 0; i < 100; i++) {
      double t = dt * i;
      double jerk = abs(trajectory.SDotDotDot(t));
      if(jerk > max_jerk) {
        max_jerk = jerk;
      }
    }

    return max_jerk > MAX_JERK ? 1 : 0;
  }
};


#endif //PATH_PLANNING_MAXJERKCOSTFUNCTION_H
