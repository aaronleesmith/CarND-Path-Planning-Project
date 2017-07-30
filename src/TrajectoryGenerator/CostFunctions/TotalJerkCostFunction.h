//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_TOTALJERKCOSTFUNCTION_H
#define PATH_PLANNING_TOTALJERKCOSTFUNCTION_H


#include "CostFunction.h"

class TotalJerkCostFunction: public CostFunction {

public:
  static const double EXPETCED_JERK_PER_SECOND = 10;

  double CalculateCost(Trajectory trajectory,
                       int target_vehicle_id,
                       double t,
                       map<int, Vehicle> vehicles) {
    double total_jerk = 0;
    double dt = trajectory.t / 100.0;
    for (int i = 0; i < 100; i++) {
      double t = dt * i;
      double jerk = trajectory.SDotDotDot(t);
      total_jerk += jerk;
    }
    double jerk_per_second = total_jerk / trajectory.t;
    return CostFunction::Logistic(jerk_per_second / EXPETCED_JERK_PER_SECOND);
  }
};


#endif //PATH_PLANNING_TOTALJERKCOSTFUNCTION_H
