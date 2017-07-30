//
// Created by Aaron Smith on 7/23/17.
//

#include "Vehicle.h"
#include "Eigen-3.3/Eigen/Dense"
#include <iostream>

void Vehicle::init(int id, double s, double d, double x, double y, double yaw, double speed) {
  this->id = id;
  this->s = s;
  this->d = d;
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->speed = speed;

  is_initialized = true;
}

void Vehicle::init(int id, double s, double d, double x, double y, double yaw, double speed, double vx, double vy) {
  this->id = id;
  this->s = s;
  this->d = d;
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->speed = speed;
  this->vx = vx;
  this->vy = vy;

  is_initialized = true;
}

void Vehicle::update(double s, double d, double x, double y, double yaw, double speed) {
  s_hist.push_back(this->s);
  this->s = s;

  d_hist.push_back(this->d);
  this->d = d;

  x_hist.push_back(this->x);
  this->x = x;

  y_hist.push_back(this->y);
  this->y = y;

  yaw_hist.push_back(this->yaw);
  this->yaw = yaw;

  speed_hist.push_back(this->speed);
  this->speed = speed;
}

void Vehicle::update(double s, double d, double x, double y, double yaw, double speed, double vx, double vy) {
  this->update(s, d, x, y, yaw, speed);

  vx_hist.push_back(this->vx);
  this->vx = vx;

  vy_hist.push_back(this->vy);
  this->vy = vy;
}

void Vehicle::printLog() {
  std::cout << "Vehicle (" << id << ") -- (S, D): (" << s << ", " << d << "). (x, y): (" << x << ", " << y << "). Speed: " << speed << "m/s. Yaw: " << yaw << "." << std::endl;
}

// todo: to accomplish this I think we'll need acceleration data.
//Eigen::Vector3f Vehicle::getLongitudinalMovementVector(double t) {
//
//}

// todo: to accomplish this I think we'll need acceleration data.
//Eigen::Vector3f Vehicle::getLateralMovementVector(double t) {
//
//}