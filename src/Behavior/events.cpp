//
// Created by Aaron Smith on 7/24/17.
//

#include "tinyfsm.hpp"

struct KeepReady              : tinyfsm::Event { };
struct KeepInLane             : tinyfsm::Event { };
struct InitiateChangeLaneLeft : tinyfsm::Event { };
struct InitiateChangeLaneRight: tinyfsm::Event { };
