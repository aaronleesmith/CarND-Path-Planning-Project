//
// Created by Aaron Smith on 7/24/17.
//

#include "DriveStraightTrajectoryGenerator.h"
#include "JMTSolver.h"
#include <vector>

using namespace std;

/**
 * Generates a trajectory which drives in a straight line for time t.
 * @param s_i [si, si_dot, si_dot_dot]
 * @param s_f [sf, sf_dot, sf_dot_dot]
 * @param d the vehicle's current d value.
 * @param t time
 * @returns A 2-dim vector where v[0] contains the s coordinates and v[1] contains the d coordinates.
 */
Trajectory DriveStraightTrajectoryGenerator::generateTrajectoryCoefficients(vector<double> s_i,
                                                                            vector<double> s_f,
                                                                            double d,
                                                                            int t) {
  JMTSolver solver = JMTSolver();

  /**
   * The D trajectory is easy here. Stay on the current d.
   */
  vector<double> d_traj = solver.JMT({d, 0, 0}, {d, 0, 0}, t);

  /**
   * The S trajectory uses the s_i and s_f inputs as well as t.
   */
  vector<double> s_traj = solver.JMT(s_i, s_f, t);

  return Trajectory(s_traj, d_traj, t);

}