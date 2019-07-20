//
// Created by luyifan on 19-7-20.
//

#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class PointVertex : public g2o::BaseVertex<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 重置
  virtual void setToOriginImpl() override {
    _estimate = 0.0;
  }

  // 更新
  virtual void oplusImpl(const double *update) override {
    _estimate += *update;
  }

  // 存盘和读盘：留空
  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class Point2PointEdge : public g2o::BaseBinaryEdge<1, double, PointVertex, PointVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Point2PointEdge() : BaseBinaryEdge() {};

  // 计算误差
  virtual void computeError() override {
    const PointVertex* v1 = static_cast<PointVertex*>(_vertices[0]);
    const PointVertex* v2 = static_cast<PointVertex*>(_vertices[1]);
    double l1 = v1->estimate();
    double l2 = v2->estimate();
    _error(0, 0) = _measurement - 1*(l1 - l2);
  }

  //计算雅可比矩阵
  virtual void linearizeOplus() override {
    PointVertex* v1 = static_cast<PointVertex*>(_vertices[0]);
    PointVertex* v2 = static_cast<PointVertex*>(_vertices[1]);
    double l1 = v1->estimate();
    double l2 = v2->estimate();
    _jacobianOplusXi(0, 0) = -1;
    _jacobianOplusXi(1, 0) = +1;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class Point2ObstacleEdge : public g2o::BaseUnaryEdge<1, double, PointVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Point2ObstacleEdge(Eigen::Vector2d obs) : BaseUnaryEdge(), _obs(obs) {
  }

  // 计算曲线模型误差
  virtual void computeError() override {
    const PointVertex *v = static_cast<const PointVertex *> (_vertices[0]);
    const double l = v->estimate();
    _error(0, 0) = 1*(-_obs[1] - l);
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    const PointVertex *v = static_cast<const PointVertex *> (_vertices[0]);
    const double l = v->estimate();
    _jacobianOplusXi(0) = -1;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  Eigen::Vector2d _obs;  // x 值， y 值为 _measurement
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class Point2CenterLineEdge : public g2o::BaseUnaryEdge<1, double, PointVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Point2CenterLineEdge() : BaseUnaryEdge() {}

  // 计算曲线模型误差
  virtual void computeError() override {
    const PointVertex *v = static_cast<const PointVertex *> (_vertices[0]);
    const double l = v->estimate();
    _error(0, 0) = 0.5*(l - _measurement);
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    const PointVertex *v = static_cast<const PointVertex *> (_vertices[0]);
    const double l = v->estimate();
    _jacobianOplusXi(0) = 0.5;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

};
