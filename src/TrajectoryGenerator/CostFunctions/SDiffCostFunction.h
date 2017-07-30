//
// Created by Aaron Smith on 7/30/17.
//

#ifndef PATH_PLANNING_SDIFFCOSTFUNCTION_H
#define PATH_PLANNING_SDIFFCOSTFUNCTION_H

#include "CostFunction.h"

class SDiffCostFunction: public CostFunction {

public:
  double CalculateCost(Trajectory trajectory,
                       int target_vehicle_id,
                       double t,
                       map<int, Vehicle> vehicles) {
    double cost = 0;
    Trajectory target = vehicles.at(target_vehicle_id).TrajectoryIn(t);

    vector<double> actual = { trajectory.S(t), trajectory.SDot(t), trajectory.SDotDot(t) };
    vector<double> expected = { target.S(t), target.SDot(t), target.SDotDot(t) };
    vector<double> sigma = { 10, 14, 2 }; // todo: these come from the python code. What are these values?
    for(int i = 0; i < 3; i++) {
      double diff = abs(actual[i] - expected[i]);
      cost += CostFunction::Logistic(diff / sigma[i]);
    }
    return cost;
  }
};

#endif //PATH_PLANNING_SDIFFCOSTFUNCTION_H
