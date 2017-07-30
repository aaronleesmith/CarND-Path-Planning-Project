//
// Created by Aaron Smith on 7/27/17.
//

#include "PathConverter.h"

void PathConverter::InitWithMapWaypoints(std::vector<double> map_waypoints_x,
                                         std::vector<double> map_waypoints_y,
                                         std::vector<double> map_waypoints_s,
                                         std::vector<double> map_waypoints_dx,
                                         std::vector<double> map_waypoints_dy) {

  x_spline.set_points(map_waypoints_s, map_waypoints_x);
  y_spline.set_points(map_waypoints_s, map_waypoints_y);
  dx_spline.set_points(map_waypoints_s, map_waypoints_dx);
  dy_spline.set_points(map_waypoints_s, map_waypoints_dy);

}
std::vector<double> PathConverter::GetXYFromSD(const double s, const double d) {
  const double x_edge = this->x_spline(s);
  const double y_edge = this->y_spline(s);
  const double dx = this->dx_spline(s);
  const double dy = this->dy_spline(s);

  const double x = x_edge + dx * d;
  const double y = y_edge + dy * d;

  return {x, y};
}