#pragma once

#include "OsqpEigen/Solver.hpp"



class PiecewiseJerkProblem
{
public:
  // 进行优化，最大迭代次数
  bool Optimize(Eigen::VectorXd* qp_solution, const int max_iter = 4000);

  // 计算 P 矩阵
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);
  // 这个是使用 OSQPEigen 接口的计算方法
  void CalculateHessianMatrix(Eigen::SparseMatrix<double>* hessian);

  // 计算 Q 向量
  void CalculateOffset(std::vector<c_float>* q);
  void CalculateGradient(Eigen::VectorXd* gradient);

  // 计算 A 矩阵
  // 计算仿射约束
  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds);
  void CalculateConstraintsMatrix(Eigen::SparseMatrix<double>* linear_matrix,
                                  int num_of_constraints);

  // 计算边界
  void CalculateUpperBound(Eigen::VectorXd* upper_bound);
  void CalculateLowerBound(Eigen::VectorXd* lower_bound);
  // 初始化 OSQP 问题：初始化各个矩阵，以及其它的一些设置
  bool FormulateProblem(OSQPData* data);
  bool FormulateProblem(OsqpEigen::Solver* solver);


protected:
  size_t num_of_knots_{50};

  // 各个参数的权重
  double weight_x_ = 1.0;
  double weight_dx_ = 2.0;
  double weight_ddx_ = 3.0;

  // output
  std::vector<double> x_{};
  std::vector<double> dx_{};
  std::vector<double> ddx_{};


  // constrict
  // 之所以是一个 pair，是因为包含上下两个边界
  // std::pair<lower_bound, upper_bound>
  std::vector<std::pair<double, double>> x_dounds_{};
  std::vector<std::pair<double, double>> dx_dounds_{};
  std::vector<std::pair<double, double>> ddx_dounds_{};

  double delta_s_{0.1};
};