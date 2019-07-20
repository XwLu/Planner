//
// Created by luyifan on 19-7-20.
//

#include "visualize.h"

Visualize::Visualize(ros::NodeHandle *pnh): pnh_(pnh) {
  topic_trajectory_vis_ = "vis_trajectory";
  topic_obstacles_vis_ = "vis_obstacles";
  pub_trajectory_ = pnh_->advertise<visualization_msgs::MarkerArray>(topic_trajectory_vis_, 1);
  pub_obstacles_ = pnh_->advertise<visualization_msgs::MarkerArray>(topic_obstacles_vis_, 1);
}

Visualize::~Visualize() {}

void Visualize::ShowTrajectory(const vector<TrajectoryPoint>& trajectory) {
  visualization_msgs::MarkerArray path;
  visualization_msgs::Marker point;
  point.header.frame_id = "/world";
  point.header.stamp = ros::Time::now();
  point.ns = "visualization";
  point.scale.x = 0.25;
  point.scale.y = 0.25;
  point.scale.z = 0.25;
  point.action = visualization_msgs::Marker::ADD;
  point.type = visualization_msgs::Marker::SPHERE;
  point.lifetime = ros::Duration(0.2);

  int id = 0;
  for(auto pt : trajectory){
    point.id = id++;
    ///位置
    point.pose.position.x = pt.s();
    point.pose.position.y = pt.l();
    point.pose.position.z = 0;
    //point.pose.orientation = ;

    ///颜色
    float value = 10;
    point.color.a = 1.0;
    if(value == 0){
      point.color.b = 1;
      point.color.g = 0;
      point.color.r = 0;
    } else{
      point.color.b = 0;
      point.color.r = 1;
      point.color.g = ((value<256) ? value : 255) / 255.0;
    }
    path.markers.emplace_back(point);
  }
  pub_trajectory_.publish(path);
}

void Visualize::ShowObstacles(const vector<Obstacle> &obstacles) {
  visualization_msgs::MarkerArray obstacles_vis;
  visualization_msgs::Marker point;
  point.header.frame_id = "/world";
  point.header.stamp = ros::Time::now();
  point.ns = "visualization";
  point.scale.x = 0.5;
  point.scale.y = 0.5;
  point.scale.z = 0.5;
  point.action = visualization_msgs::Marker::ADD;
  point.type = visualization_msgs::Marker::CYLINDER;
  point.lifetime = ros::Duration(0.2);

  int id = 0;
  for(auto obs : obstacles){
    point.id = id++;
    ///位置
    point.pose.position.x = obs.s();
    point.pose.position.y = obs.l();
    point.pose.position.z = 0;
    //point.pose.orientation = ;

    ///颜色
    point.color.a = 1.0;
    point.color.b = 0;
    point.color.r = 0;
    point.color.g = 1;

    obstacles_vis.markers.emplace_back(point);
  }
  pub_obstacles_.publish(obstacles_vis);
}
