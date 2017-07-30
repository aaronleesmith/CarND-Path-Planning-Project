//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_TOTALACCELERATIONCOSTFUNCTION_H
#define PATH_PLANNING_TOTALACCELERATIONCOSTFUNCTION_H


#include "CostFunction.h"

class TotalAccelerationCostFunction: public CostFunction {

public:
  static const double EXPECTED_ACCELERATION_PER_SECOND = 10;

  double CalculateCost(Trajectory trajectory,
                       int target_vehicle_id,
                       double t,
                       map<int, Vehicle> vehicles) {
    double total_acceleration = 0;
    double dt = trajectory.t / 100.0;
    for (int i = 0; i < 100; i++) {
      double t = dt * i;
      double acc = trajectory.SDotDot(t);
      total_acceleration += acc;
    }
    double acc_per_second = total_acceleration / trajectory.t;
    return CostFunction::Logistic(acc_per_second / EXPECTED_ACCELERATION_PER_SECOND);
  }
};


#endif //PATH_PLANNING_TOTALACCELERATIONCOSTFUNCTION_H
