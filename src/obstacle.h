//
// Created by luyifan on 19-7-20.
//

#ifndef PLANNER_OBSTACLE_H
#define PLANNER_OBSTACLE_H

#include "common.h"

class Obstacle{
public:
  Obstacle();
  Obstacle(double x, double y, double s, double l);
  ~Obstacle();

  inline double x() { return x_; }
  inline double y() { return y_; }
  inline double s() { return s_; }
  inline double l() { return l_; }

  inline void set_x(double x) { x_ = x; }
  inline void set_y(double y) { y_ = y; }
  inline void set_s(double s) { s_ = s; }
  inline void set_l(double l) { l_ = l; }

private:
  double x_;
  double y_;
  double s_;
  double l_;

  std::vector<Point2D> polygon_;
};

#endif //PLANNER_OBSTACLE_H
