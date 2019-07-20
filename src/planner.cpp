//
// Created by luyifan on 19-7-20.
//

#include "planner.h"


Planner::Planner() {
  resolution_ = 0.5;//优化路点的间隔
  forward_length_ = 50;//向前规划50米
  obstacles_ = nullptr;
}

Planner::~Planner() {}

void Planner::Init(Transform* tf,
                   vector<Obstacle>* obstacles) {
  //TODO: 将obstacle的(x, y)根据path的坐标转换到frenet坐标系下
  obstacles_ = obstacles;
  tf_ = tf;
  for(size_t i = 0; i < int(forward_length_/resolution_); i++){
    double s = i * resolution_;
    TrajectoryPoint pt(0.0, 0.0, s, 0.0);
    trajectory_.emplace_back(std::move(pt));
  }

  // 构建图优化，先设定g2o
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<1, 1>> BlockSolverType;  // 每个误差项优化变量维度为1，误差值维度为1
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

  // 梯度下降方法，可以从GN, LM, DogLeg 中选
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  optimizer_ = new g2o::SparseOptimizer();     // 图模型
  optimizer_->setAlgorithm(solver);   // 设置求解器
  optimizer_->setVerbose(true);       // 打开调试输出
  double w_sigma = 1.0;

  // 往图中增加顶点
  int id = 0;
  for(size_t i = 0; i < trajectory_.size(); i++){
    PointVertex* v = new PointVertex();
    v->setEstimate( 0.0 );
    v->setId(id);
    optimizer_->addVertex( v );
    id++;
    vs_.emplace_back( v );
  }

  // 往图中增加边
  id = 0;
  for (int i = 0; i < trajectory_.size() - 1; ++i) {
    Point2PointEdge *edge = new Point2PointEdge();
    edge->setId(id);
    edge->setVertex(0, vs_[i]);                // 设置连接的顶点
    edge->setVertex(1, vs_[i+1]);
    edge->setMeasurement(0.0);      // 观测数值
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); // 信息矩阵：协方差矩阵之逆
    optimizer_->addEdge(edge);
    id++;
  }

  // 往图中增加边

  for(size_t i = 0; i < trajectory_.size(); ++i) {
    for(auto obs : *obstacles_){
      if(fabs(obs.s() -  trajectory_[i].s()) < 2){
        Point2ObstacleEdge* edge = new Point2ObstacleEdge(Eigen::Vector2d(obs.s(), obs.l()));
        edge->setId(id);
        edge->setVertex(0, vs_[i]);
        //edge->setMeasurement();
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); // 信息矩阵：协方差矩阵之逆
        optimizer_->addEdge(edge);
        id++;
      }
    }
  }


  // 往图中增加边
  for(size_t i = 0; i < trajectory_.size(); ++i){
    Point2CenterLineEdge* edge = new Point2CenterLineEdge();
    edge->setId(id);
    edge->setVertex(0, vs_[i]);
    edge->setMeasurement(0.0);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); // 信息矩阵：协方差矩阵之逆
    optimizer_->addEdge(edge);
    id++;
  }

}

void Planner::Run() {
  // 执行优化
  cout << "start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer_->initializeOptimization();
  optimizer_->optimize(10);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  int id = 0;
  for(auto v : vs_){
    double l = v->estimate();
    cout<<"l: "<<l<<endl;
    trajectory_[id].set_l(l);
    id++;
  }
}