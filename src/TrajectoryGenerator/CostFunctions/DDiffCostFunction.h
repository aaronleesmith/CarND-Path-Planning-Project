//
// Created by Aaron Smith on 7/30/17.
//

#ifndef PATH_PLANNING_DDIFFCOSTFUNCTION_H
#define PATH_PLANNING_DDIFFCOSTFUNCTION_H

#include "CostFunction.h"

class DDiffCostFunction: public CostFunction {

public:
  double CalculateCost(Trajectory trajectory,
                       int target_vehicle_id,
                       double t,
                       map<int, Vehicle> vehicles) {
    double cost = 0;
    Trajectory target = vehicles.at(target_vehicle_id).TrajectoryIn(t);

    vector<double> actual = { trajectory.D(t), trajectory.DDot(t), trajectory.DDotDot(t) };
    vector<double> expected = { target.D(t), target.DDot(t), target.DDotDot(t) };
    vector<double> sigma = { 1, 1, 1 };
    for(int i = 0; i < 3; i++) {
      double diff = abs(actual[i] - expected[i]);
      cost += CostFunction::Logistic(diff / sigma[i]);
    }
    return cost;
  }
};


#endif //PATH_PLANNING_DDIFFCOSTFUNCTION_H
