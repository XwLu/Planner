//
// Created by luyifan on 19-7-20.
//

#ifndef PLANNER_MAP_H
#define PLANNER_MAP_H

#include "common.h"

class WayPoint{
public:
  WayPoint(double x, double y, double s, long long id){
    x_ = x;
    y_ = y;
    s_ = s;
    id_ = id;
  }

  inline double x(){ return x_; }
  inline double y(){ return y_; }
  inline double s(){ return s_; }
  inline double id(){ return id_; }

  inline void setX(double x){ x_ = x;}
  inline void setY(double y){ y_ = y;}
  inline void setS(double s){ s_ = s;}

private:
  double x_;
  double y_;
  double s_;
  long long id_;
};

class Transform{
public:
  Transform();
  ~Transform();

  bool GetReferenceLine(std::string map_file);

  Eigen::Vector2d GetSD(double x, double y, double theta);
  Eigen::Vector2d GetXY(double s, double d);
  int FindClosestWP(double x, double y);
  int NextWP(double x, double y, double theta);

  inline double length() { return reference_line_.back().s(); }
  inline const vector<WayPoint>& reference_line() { return reference_line_; }

private:
  vector<WayPoint> reference_line_;
};

#endif //PLANNER_MAP_H
