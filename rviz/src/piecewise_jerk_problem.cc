#include "rviz/osqp/piecewise_jerk_problem.h"
#include <rclcpp/logger.hpp>


// 进行优化，最大迭代次数
bool PiecewiseJerkProblem::Optimize(Eigen::VectorXd* qp_solution,
                                    const int max_iter)
{
  OsqpEigen::Solver solver;

  // 设置 Settings 参数
  solver.settings()->setWarmStart(true);

  // 设置 Data 参数
  FormulateProblem(&solver);

  // 清除求解器如果已初始化，然后重新初始化
  if(solver.isInitialized())
  {
    solver.clearSolver();
  }

  // 获取并打印上下边界
  //
  // c_float *l; ///< dense array for lower bound (size m) 下限密集数组（大小 m）
  // c_float *u; ///< dense array for upper bound (size m) 上限密集数组（大小 m）


  // error 出现了一个很奇怪的问题,在 FormlateProblem 中对 solver 进行初始化 lower_bound 和 upper_bound 会无效
  // 计算边界
  Eigen::VectorXd lower_bound(2);
  Eigen::VectorXd upper_bound(2);
  lower_bound << 1, 1;
  std::cout << "lower_bound:";
  std::cout << lower_bound.transpose() << std::endl;
  if(!solver.data()->setLowerBound(lower_bound))
  {
    std::cerr << "设置下界失败!" << std::endl;
    return false;
  }

  upper_bound << 2, 2;
  std::cout << "upper_bound:";
  std::cout << upper_bound.transpose() << std::endl;
  if(!solver.data()->setUpperBound(upper_bound))
  {
    std::cerr << "设置上界失败!" << std::endl;
    return false;
  }

  c_float* l = solver.data()->getData()->l;
  c_float* u = solver.data()->getData()->u;

  std::cout << "当前下界: ";
  for(int idx = 0; idx < 2; ++idx)
  {
    std::cout << idx << l[idx] << " ";
  }

  std::cout << "当前上界: ";
  for(int idx = 0; idx < 2; ++idx)
  {
    std::cout << idx << u[idx] << " ";
  }

  std::cout << "\n";

  // 初始化求解器
  if(!solver.initSolver())
  {
    std::cerr << "初始化失败!" << std::endl;
    return false;
  }


  // 进行求解
  // if(!solver.solve())
  if(OsqpEigen::ErrorExitFlag::NoError != solver.solveProblem())
  {
    std::cerr << "求解失败!" << std::endl;
    return false;
  }


  // 获取最优解
  *qp_solution = solver.getSolution();


  return true;
}

// 计算 P 矩阵
// 计算内核函数应该是各个不同的问题进行分别计算
void PiecewiseJerkProblem::CalculateKernel(std::vector<c_float>* p_data,
                                           std::vector<c_int>* p_indices,
                                           std::vector<c_int>* p_indptr)
{
  // todo 这里需要在各自的子类问题中进行具体的实现，这里值是进行一个 demo 演示
  // 这里计算 wx 矩阵
  for(size_t idx = 0; idx < num_of_knots_; ++idx)
  {
    p_data->push_back(weight_x_);
    p_indices->push_back(idx);
    p_indptr->push_back(idx);
  }

  // 这里计算 wdx 矩阵
  for(size_t idx = num_of_knots_; idx < 2 * num_of_knots_; ++idx)
  {
    p_data->push_back(weight_dx_);
    p_indices->push_back(idx);
    p_indptr->push_back(idx);
  }

  // 这里计算 wddx 矩阵
  for(size_t idx = 2 * num_of_knots_; idx < 3 * num_of_knots_; ++idx)
  {
    p_data->push_back(weight_ddx_);
    p_indices->push_back(idx);
    p_indptr->push_back(idx);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  // tips 上面是自己原始的实现思路,看下了 apollo 的实现思路,发现自己没有理解到 indptr 的含义
  // idea 这里 apollo 将 类型从 size_t 转到了 int 类型
  const int num = static_cast<int>(num_of_knots_);
  const int num_of_variable = num * 3;


  // columns 列
  // 这里使用 columns 来表示 P 矩阵中的列,它的下标就是第 idx 列,在初始化时(就已经确定了有多少列)
  // 其中的行的表示用(row_indx, data) 这样的 pair 来表示,也就是说只存放有数据的行,如何该行没有数据,就可以不用存
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variable);
  int value_idx = 0;
  // 这里计算 wx 矩阵
  for(size_t idx = 0; idx < num_of_knots_; ++idx)
  {
    columns[idx].emplace_back(idx, weight_x_);
    ++value_idx;
  }

  // 这里计算 wdx 矩阵
  for(size_t idx = num_of_knots_; idx < 2 * num_of_knots_; ++idx)
  {
    columns[idx].emplace_back(idx, weight_dx_);
    ++value_idx;
  }

  // 这里计算 wddx
  for(size_t idx = 2 * num_of_knots_; idx < 3 * num_of_knots_; ++idx)
  {
    columns[idx].emplace_back(idx, weight_ddx_);
    ++value_idx;
  }

  // todo
  // check value_idx == num_of_variable
  size_t num_of_cols = 0;
  for(size_t idx = 0; idx < 3 * num_of_knots_; ++idx)
  {
    p_indptr->push_back(num_of_cols);
    for(auto item : columns[idx])
    {
      p_data->push_back(item.second);
      p_indices->push_back(item.first);
      ++num_of_cols;
    }
  }
  p_indptr->push_back(num_of_cols);
  //tips 这样好像会有一个问题, data 和 indices 没有和 indptr 对应上
  // 我的初始想法是:每个权重需要三个维度才能确认: col  row  data
  // 但是 apollo 中的做法似乎每个权重,并不是又三个权重确认的

  // 这里存储矩阵用的是 CSC 稀疏矩阵格式
  // CSC（Compressed Sparse Column，压缩列存储）格式是一种用于存储稀疏矩阵的高效数据结构。
}

void PiecewiseJerkProblem::CalculateHessianMatrix(
    Eigen::SparseMatrix<double>* hessian)
{
  hessian->resize(3 * num_of_knots_, 3 * num_of_knots_);

  std::vector<Eigen::Triplet<double>> triplet_list;

  // 构造 wx 参数矩阵
  for(size_t idx = 0; idx < num_of_knots_; ++idx)
  {
    triplet_list.emplace_back(idx, idx, weight_x_);
  }

  // 构造 wdx 参数矩阵
  for(size_t idx = num_of_knots_; idx < 2 * num_of_knots_; ++idx)
  {
    triplet_list.emplace_back(idx, idx, weight_dx_);
  }

  // 构造 wddx 参数矩阵
  for(size_t idx = 2 * num_of_knots_; idx < 3 * num_of_knots_; ++idx)
  {
    triplet_list.emplace_back(idx, idx, weight_ddx_);
  }

  hessian->setFromTriplets(triplet_list.begin(), triplet_list.end());
}

// 计算 Q 矩阵
void PiecewiseJerkProblem::CalculateOffset(std::vector<c_float>* q) {}

// 计算 A 矩阵
// 计算仿射约束
void PiecewiseJerkProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data,
    std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr,
    std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds)
{}


void PiecewiseJerkProblem::CalculateGradient(Eigen::VectorXd* gradient)
{
  // tips 在这个示例中，gradient 是 0
}


void PiecewiseJerkProblem::CalculateConstraintsMatrix(
    Eigen::SparseMatrix<double>* linear_matrix, int num_of_constraints)
{
  linear_matrix->resize(num_of_constraints, 3 * num_of_knots_);
  std::vector<Eigen::Triplet<double>> trip_vec;
  // todo 目前先提供两个约束，测试接口是否准备
  trip_vec.emplace_back(0, 0, 1);
  trip_vec.emplace_back(1, 0, 1);
  linear_matrix->setFromTriplets(trip_vec.begin(), trip_vec.end());
}


void PiecewiseJerkProblem::CalculateUpperBound(Eigen::VectorXd* upper_bound)
{
  (*upper_bound) << 2, 2;
}

void PiecewiseJerkProblem::CalculateLowerBound(Eigen::VectorXd* lower_bound)
{
  (*lower_bound) << -1, 0;
}


// 初始化 OSQP 问题：初始化各个矩阵，以及其它的一些设置
bool PiecewiseJerkProblem::FormulateProblem(OsqpEigen::Solver* solver)
{
  // 计算 P 矩阵， indices 索引
  // std::vector<c_float> p_data;  // 存放的是 P 矩阵中的数据
  // std::vector<c_int> p_indices; // 存放的是 P 矩阵中的行
  // std::vector<c_int>
  //     p_indptr; // 存放的是 P 矩阵中的列指针向量：表示的是到当前列以前：稀疏矩阵中共有多少个元素（不包括当前列的个数）

  // CalculateKernel(&p_data, &p_indices, &p_indptr);

  //tips 使用 OSQP-Eigen 的接口
  // 计算 P 矩阵
  Eigen::SparseMatrix<double> hessian;
  CalculateHessianMatrix(&hessian);

  // 计算 Q 向量
  Eigen::VectorXd gradient(3 * num_of_knots_);
  CalculateGradient(&gradient);


  // 计算 A 矩阵
  Eigen::SparseMatrix<double> linear_matrix;
  CalculateConstraintsMatrix(&linear_matrix, 2);

  // 计算边界
  Eigen::VectorXd lower_bound(2);
  Eigen::VectorXd upper_bound(2);

  // CalculateLowerBound(&lower_bound);
  // CalculateUpperBound(&upper_bound);

  solver->settings()->setWarmStart(true);
  solver->data()->setNumberOfVariables(3 * num_of_knots_);
  solver->data()->setNumberOfConstraints(2);
  if(!solver->data()->setHessianMatrix(hessian))
  {
    return false;
  }
  if(!solver->data()->setGradient(gradient))
  {
    return false;
  }
  if(!solver->data()->setLinearConstraintsMatrix(linear_matrix))
  {
    return false;
  }
  // if(solver.data()->setBounds())


  lower_bound << 1, 1;
  std::cout << "lower_bound:";
  std::cout << lower_bound.transpose() << std::endl;
  if(!solver->data()->setLowerBound(lower_bound))
  {
    std::cerr << "设置下界失败!" << std::endl;
    return false;
  }

  upper_bound << 2, 2;
  std::cout << "upper_bound:";
  std::cout << upper_bound.transpose() << std::endl;
  if(!solver->data()->setUpperBound(upper_bound))
  {
    std::cerr << "设置上界失败!" << std::endl;
    return false;
  }


  return true;
}