//
// Created by Aaron Smith on 7/24/17.
//

#ifndef PATH_PLANNING_BEHAVIORCONTROLLER_H
#define PATH_PLANNING_BEHAVIORCONTROLLER_H

#include <vector>
#include <map>

#include "VehicleFSM.h"
#include "../Vehicle.h"

using namespace std;

class BehaviorController {

public:
  BehaviorController() {
    finiteStateMachine = VehicleFSM();
  };

  virtual ~BehaviorController() {};

  VehicleFSM finiteStateMachine;

  /**
   * Evaluates the positions of the vehicles on the road to determine which state transitions are currently safe.
   * @return list of events which are safe to transition to.
   */
  vector<tinyfsm::Event> getSafeStateTransitions(map<int, Vehicle>);
};


#endif //PATH_PLANNING_BEHAVIORCONTROLLER_H
