//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include <vector>

class TrajectoryGenerator {

public:
  virtual std::vector<std::vector<double>> generateTrajectoryCoefficients(std::vector<double> s_i,
                                                                  std::vector<double> s_f,
                                                                  double d,
                                                                  int t);
};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
