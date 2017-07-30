//
// Created by Aaron Smith on 7/30/17.
//

#ifndef PATH_PLANNING_STATE_H
#define PATH_PLANNING_STATE_H

#include <map>

#include "../../Vehicle.h"
#include "../../TrajectoryGenerator/Trajectory.h"

using namespace std;

class State {
public:
  virtual Trajectory GenerateTrajectory(Vehicle ego, map<int, Vehicle> vehicles);
};


#endif //PATH_PLANNING_STATE_H
