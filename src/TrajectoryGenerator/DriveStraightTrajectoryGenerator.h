//
// Created by Aaron Smith on 7/24/17.
//

#ifndef PATH_PLANNING_DRIVESTRAIGHTTRAJECTORYGENERATOR_H
#define PATH_PLANNING_DRIVESTRAIGHTTRAJECTORYGENERATOR_H

#include <vector>
#include "TrajectoryGenerator.h"
#include "Trajectory.h"

class DriveStraightTrajectoryGenerator {

public:
  DriveStraightTrajectoryGenerator() {};

  virtual ~DriveStraightTrajectoryGenerator() {};

  Trajectory generateTrajectoryCoefficients(std::vector<double> s_i,
                                            std::vector<double> s_f,
                                            double d,
                                            int t);
};


#endif //PATH_PLANNING_DRIVESTRAIGHTTRAJECTORYGENERATOR_H
