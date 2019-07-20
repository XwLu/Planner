//
// Created by luyifan on 19-7-20.
//

#ifndef PLANNER_PLANNER_H
#define PLANNER_PLANNER_H

#include "trajectory.h"
#include "obstacle.h"
#include "transform.h"
#include "optimizer.h"

class Planner{
public:
  Planner();
  ~Planner();

  void Init(Transform* tf,
            vector<Obstacle>* obstacles);
  void Run();
  inline vector<TrajectoryPoint>& trajectory(){ return trajectory_; }
private:
  vector<TrajectoryPoint> trajectory_;
  double resolution_;
  vector<Obstacle>* obstacles_;
  Transform* tf_;
  double forward_length_;

  g2o::SparseOptimizer* optimizer_;
  vector<PointVertex*> vs_;
};

#endif //PLANNER_PLANNER_H
