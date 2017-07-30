//
// Created by Aaron Smith on 7/24/17.
//

#ifndef PATH_PLANNING_JMTSOLVER_H
#define PATH_PLANNING_JMTSOLVER_H

#include <vector>

class JMTSolver {
public:
  JMTSolver() {};

  virtual ~JMTSolver() {};

  std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);
};


#endif //PATH_PLANNING_JMTSOLVER_H
