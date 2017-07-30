//
// Created by Aaron Smith on 7/24/17.
//

#ifndef PATH_PLANNING_VEHICLEFSM_H
#define PATH_PLANNING_VEHICLEFSM_H

#include "tinyfsm.hpp"
#include "events.cpp"
#include <iostream>
using namespace std;

class Ready;
class KeepLane;
class ChangeLaneLeft;
class ChangeLaneRight;

class VehicleFSM : public tinyfsm::Fsm<VehicleFSM> {

public:
  /**
   * Default reaction for unhandled events.
   */
  void react(tinyfsm::Event const &) {}

  void react(KeepReady                const &) { transit<Ready>(); };
  void react(KeepInLane               const &) { transit<KeepLane>(); }
  void react(InitiateChangeLaneLeft   const &) { transit<ChangeLaneLeft>(); }
  void react(InitiateChangeLaneRight  const &) { transit<ChangeLaneRight>(); }

  virtual void entry(void) { };
  void exit(void) { };
};

class Ready : public VehicleFSM {
  void entry() override {
    cout << "Vehicle has entered READY state." << endl;
  }
};

class KeepLane : public VehicleFSM {
  void entry() override {
    cout << "Vehicle has entered KEEP LANE state." << endl;
  }
};

class ChangeLaneLeft : public VehicleFSM {
  void entry() override {
    cout << "Vehicle has entered CHANGE LANE LEFT state." << endl;
  }
};

class ChangeLaneRight: public VehicleFSM {
  void entry() override {
    cout << "Vehicle has entered CHANGE_LANE_RIGHT state." << endl;
  }
};

#endif //PATH_PLANNING_VEHICLEFSM_H
