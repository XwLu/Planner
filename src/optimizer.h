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

  Point2PointEdge() : BaseBinaryEdge() {
    _w = 1.2;
  };

  // 计算误差
  virtual void computeError() override {
    const PointVertex* v1 = static_cast<PointVertex*>(_vertices[0]);
    const PointVertex* v2 = static_cast<PointVertex*>(_vertices[1]);
    double l1 = v1->estimate();
    double l2 = v2->estimate();
    _error(0, 0) = _measurement - _w*(l1 - l2);
  }

  //计算雅可比矩阵
  virtual void linearizeOplus() override {
    PointVertex* v1 = static_cast<PointVertex*>(_vertices[0]);
    PointVertex* v2 = static_cast<PointVertex*>(_vertices[1]);
    double l1 = v1->estimate();
    double l2 = v2->estimate();
    _jacobianOplusXi(0, 0) = -_w;
    _jacobianOplusXi(1, 0) = +_w;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  double _w;
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class Point2ObstacleEdge : public g2o::BaseUnaryEdge<1, double, PointVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Point2ObstacleEdge(Eigen::Vector2d obs, double point_s) : BaseUnaryEdge(), _obs(obs), _point_s(point_s) {
    _dist_threshold = 2.5;
    _w = 0.5;
  }

  // 计算曲线模型误差
  virtual void computeError() override {
    const PointVertex *v = static_cast<const PointVertex *> (_vertices[0]);
    const double l = v->estimate();
    double dist = sqrt(pow(l - _obs[1], 2.0) + pow(_point_s - _obs[0], 2.0));
    if(dist < _dist_threshold)
      _error(0, 0) = (dist - _dist_threshold) / _w;
    else
      _error(0, 0) = 0.0;
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    const PointVertex *v = static_cast<const PointVertex *> (_vertices[0]);
    const double l = v->estimate();
    double dist = sqrt(pow(l - _obs[1], 2.0) + pow(_point_s - _obs[0], 2.0));
    if (dist < _dist_threshold)
      _jacobianOplusXi(0) = (l - _obs[1])/(_w * dist);
    else
      _jacobianOplusXi(0) = 0.0;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  Eigen::Vector2d _obs;  // x 值， y 值为 _measurement
  double _point_s;
  double _dist_threshold;
  double _w;
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class Point2CenterLineEdge : public g2o::BaseUnaryEdge<1, double, PointVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Point2CenterLineEdge() : BaseUnaryEdge() {
    _w = 0.5;
  }

  // 计算曲线模型误差
  virtual void computeError() override {
    const PointVertex *v = static_cast<const PointVertex *> (_vertices[0]);
    const double l = v->estimate();
    _error(0, 0) = _w*(l - _measurement);
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    const PointVertex *v = static_cast<const PointVertex *> (_vertices[0]);
    const double l = v->estimate();
    _jacobianOplusXi(0) = _w;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  double _w;

};
