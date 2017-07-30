//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_EFFICIENCYCOSTFUNCTION_H
#define PATH_PLANNING_EFFICIENCYCOSTFUNCTION_H


#include "CostFunction.h"

class EfficiencyCostFunction: CostFunction {

public:
  double calculateCost();
};


#endif //PATH_PLANNING_EFFICIENCYCOSTFUNCTION_H
