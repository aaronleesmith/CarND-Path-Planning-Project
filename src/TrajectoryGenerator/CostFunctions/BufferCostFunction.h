//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_BUFFERCOSTFUNCTION_H
#define PATH_PLANNING_BUFFERCOSTFUNCTION_H


#include "CostFunction.h"
#include "../../Vehicle.h";

class BufferCostFunction: public CostFunction {

public:
  double CalculateCost(Trajectory trajectory,
                       int target_vehicle_id,
                       double t,
                       map<int, Vehicle> vehicles) {
    double nearest = CostFunction::NearestApproachToAnyVehicle(trajectory, vehicles);
    return CostFunction::Logistic(2 * Vehicle::VEHICLE_RADIUS / nearest);
  }
};


#endif //PATH_PLANNING_BUFFERCOSTFUNCTION_H
