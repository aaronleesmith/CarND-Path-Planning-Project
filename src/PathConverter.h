//
// Created by Aaron Smith on 7/27/17.
//

#ifndef PATH_PLANNING_PATHCONVERTER_H
#define PATH_PLANNING_PATHCONVERTER_H

#include <vector>

#include "spline.h"

class PathConverter {

private:
  tk::spline x_spline;
  tk::spline y_spline;
  tk::spline dx_spline;
  tk::spline dy_spline;

public:
  /**
   * Initializes the splines used for interpolation given the map waypoints.
   * @param map_waypoints_x
   * @param map_waypoints_y
   * @param map_waypoints_s
   * @param map_waypoints_dx
   * @param map_waypoints_dy
   */
  void InitWithMapWaypoints(std::vector<double> map_waypoints_x,
                            std::vector<double> map_waypoints_y,
                            std::vector<double> map_waypoints_s,
                            std::vector<double> map_waypoints_dx,
                            std::vector<double> map_waypoints_dy);

  /**
   * Interpolates using the pre-built splines to get a smooth x, y value from a given Frenet coordinate.
   * @param s
   * @param d
   * @return
   */
  std::vector<double> GetXYFromSD(const double s, const double d);

};


#endif //PATH_PLANNING_PATHCONVERTER_H
