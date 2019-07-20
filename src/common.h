//
// Created by luyifan on 19-7-20.
//

#ifndef PLANNER_COMMON_H
#define PLANNER_COMMON_H

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/QR>

struct Point2D{
  double x;
  double y;
};


using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::endl;

const double MY_INF = std::numeric_limits<double>::infinity();
const double MY_PI = 3.1415926;

#endif //PLANNER_COMMON_H
