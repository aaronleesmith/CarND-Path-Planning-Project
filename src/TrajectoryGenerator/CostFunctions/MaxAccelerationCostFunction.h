  //
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_MAXACCELERATIONCOSTFUNCTION_H
#define PATH_PLANNING_MAXACCELERATIONCOSTFUNCTION_H


#include "CostFunction.h"

class MaxAccelerationCostFunction: public CostFunction {

public:
  static const double MAX_ACCELERATION = 10;

  double CalculateCost(Trajectory trajectory,
                       int target_vehicle_id,
                       double t,
                       map<int, Vehicle> vehicles) {
    double max_acceleration = -1;
    double dt = trajectory.t / 100.0;
    for(int i = 0; i < 100; i++) {
      double t = dt * i;
      double acc = abs(trajectory.SDotDot(t));
      if(acc > max_acceleration) {
        max_acceleration = acc;
      }
    }

    return max_acceleration > MAX_ACCELERATION ? 1 : 0;
  }
};


#endif //PATH_PLANNING_MAXACCELERATIONCOSTFUNCTION_H
