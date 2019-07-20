//
// Created by luyifan on 19-7-20.
//

#ifndef PLANNER_VISUALIZE_H
#define PLANNER_VISUALIZE_H

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include "common.h"
#include "trajectory.h"
#include "obstacle.h"

class Visualize{
public:
  Visualize(ros::NodeHandle* pnh);
  ~Visualize();

  void ShowTrajectory(const vector<TrajectoryPoint>& trajectory);

  void ShowObstacles(const vector<Obstacle>& obstacles);

private:
  ros::NodeHandle* pnh_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_obstacles_;

  std::string topic_trajectory_vis_;
  std::string topic_obstacles_vis_;
};

#endif //PLANNER_VISUALIZE_H
