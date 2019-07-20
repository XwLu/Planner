//
// Created by luyifan on 19-7-20.
//

#include "planner.h"
#include "optimizer.h"

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
  g2o::SparseOptimizer optimizer;     // 图模型
  optimizer.setAlgorithm(solver);   // 设置求解器
  optimizer.setVerbose(true);       // 打开调试输出

  vector<PointVertex*> vs;
  // 往图中增加顶点
  for(size_t i = 0; i < trajectory_.size(); i++){
    PointVertex* v = new PointVertex();
    v->setEstimate( 0.0 );
    v->setId(i);
    optimizer.addVertex( v );
    vs.emplace_back( v );
  }

  // 往图中增加边
  for (int i = 0; i < trajectory_.size() - 1; i++) {
    Point2PointEdge *edge = new Point2PointEdge();
    edge->setId(i);
    edge->setVertex(0, vs[i]);                // 设置连接的顶点
    edge->setVertex(1, vs[i+1]);
    //edge->setMeasurement(y_data[i]);      // 观测数值
    double w_sigma = 1.0;
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); // 信息矩阵：协方差矩阵之逆
    optimizer.addEdge(edge);
  }

  // 往图中增加边

}

void Planner::Run() {

}