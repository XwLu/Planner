//
// Created by luyifan on 19-7-20.
//

#include "trajectory.h"

TrajectoryPoint::TrajectoryPoint() {
    x_ = 0.0;
    y_ = 0.0;
    s_ = 0.0;
    l_ = 0.0;
}

TrajectoryPoint::TrajectoryPoint(double x, double y, double s, double l) {
    x_ = x;
    y_ = y;
    s_ = s;
    l_ = l;
}

TrajectoryPoint::~TrajectoryPoint() {}

