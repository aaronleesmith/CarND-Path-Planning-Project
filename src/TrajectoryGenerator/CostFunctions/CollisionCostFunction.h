//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_COLLISIONCOSTFUNCTION_H
#define PATH_PLANNING_COLLISIONCOSTFUNCTION_H


#include "CostFunction.h"

class CollisionCostFunction: public CostFunction {

public:
  double CalculateCost(Trajectory trajectory,
                       int target_vehicle_id,
                       double t,
                       map<int, Vehicle> vehicles) {
    double nearest = CostFunction::NearestApproachToAnyVehicle(trajectory, vehicles);
    return nearest < 2 * Vehicle::VEHICLE_RADIUS ? 1 : 0;
  }
};


#endif //PATH_PLANNING_COLLISIONCOSTFUNCTION_H
