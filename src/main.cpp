//
// Created by luyifan on 19-7-20.
//
#include "visualize.h"
#include "transform.h"
#include "obstacle.h"
#include "planner.h"

bool CreateObstacles(vector<Obstacle>& obstacles, int number, double s_range){
  if(!obstacles.empty()){
    cerr<<"There has been obstacles in vector!";
    return false;
  }

  for(int i = 1; i < number; i++){
    double s = i * s_range / number;
    double l = pow(-1.0, double(i)) * 1.5;
    Obstacle obs(0.0, 0.0, s, l);
    obstacles.emplace_back(std::move(obs));
  }
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "planner");
  ros::NodeHandle pnh("~");

  //轨迹显示模块
  Visualize vis(&pnh);

  //获取 reference line
  Transform tf;
  tf.GetReferenceLine("/home/luyifan/Projects/catkin_ws/src/planner/map/map.csv");

  //创建虚拟障碍物
  vector<Obstacle> obstacles;
  if (!CreateObstacles(obstacles, 5, 50)){
    return 0;
  }

  //初始化规划器
  Planner planner;
  planner.Init(&tf, &obstacles);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    planner.Run();
    vis.ShowTrajectory(planner.trajectory());
    loop_rate.sleep();
  }
  cout<<"Program Done!"<<endl;
}