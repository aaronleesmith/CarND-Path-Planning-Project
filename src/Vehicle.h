//
// Created by Aaron Smith on 7/23/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "TrajectoryGenerator/Trajectory.h"

using namespace std;

class Vehicle {

  int max_history_length = 100;

public:
  static constexpr double VEHICLE_RADIUS = 1.5;

  Vehicle() {};

  virtual ~Vehicle() {};

  int id;

  double s;
  double d;

  double x;
  double y;

  double vx;
  double vy;

  double yaw;
  double speed;

  double s_accel = 0;
  double d_accel = 0;

  // Historical data
  std::vector<double> s_hist;
  std::vector<double> d_hist;
  std::vector<double> x_hist;
  std::vector<double> y_hist;
  std::vector<double> vx_hist;
  std::vector<double> vy_hist;
  std::vector<double> yaw_hist;
  std::vector<double> speed_hist;

  bool is_initialized = false;

  void init(int id_init, double s_init, double d_init, double x_init, double y_init, double yaw_init, double speed_init);

  void init(int id_init, double s_init, double d_init, double x_init, double y_init, double yaw_init, double speed_init, double vx, double vy);

  void update(double s, double d, double x, double y, double yaw, double speed);

  void update(double s, double d, double x, double y, double yaw, double speed, double vx, double vy);

  /**
   * Currently all vehicles have 0 acceleration and do not move in the d-direction.
   * @param t
   * @return
   */
  vector<vector<double>> StateIn(double t) {
    return {
      {
        s + (speed * t) + s_accel * pow(t, 2) / 2.0,
        speed + s_accel * t,
        s_accel
      },
      {
        d,
        0,
        0
      }
    };
  }

  Trajectory TrajectoryIn(double t) {
    return Trajectory({ s + (speed * t) + s_accel * pow(t, 2) / 2.0, speed + s_accel * t, s_accel }, { d, 0, 0 }, t);
  }

  void printLog();
  /**
   * Gets the lateral (d) movement vector of d, ddot, ddoubledot.
   * @param t
   * @return
   */
//  Eigen::Vector3f getLateralMovementVector(double t);

  /**
   * Gets the longitudian (s) movement vector of s, sdot, sdoubledot.
   * @param t
   * @return
   */
//  Eigen::Vector3f getLongitudinalMovementVector(double t);
};


#endif //PATH_PLANNING_VEHICLE_H
