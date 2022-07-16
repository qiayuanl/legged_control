//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include "legged_wbc/task.h"

#include <memory>

namespace legged
{
// Hierarchical Optimization Quadratic Program
class HoQp
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;
  HoQp(const Task& task);
  HoQp(const Task&, HoQpPtr higher_problem);

  matrix_t getStackedZMatrix() const
  {
    return stacked_z_;
  }

  Task getStackedTasks() const
  {
    return stacked_tasks_;
  }

  vector_t getStackedSlackSolutions() const
  {
    return stacked_slack_vars_;
  }

  vector_t getSolutions() const
  {
    vector_t x = x_prev_ + stacked_z_prev_ * decision_vars_solutions_;
    return x;
  }

  size_t getSlackedNumVars() const
  {
    return stacked_tasks_.d_.rows();
  }

private:
  void initVars();
  void formulateProblem();
  void solveProblem();

  void buildHMatrix();
  void buildCVector();
  void buildDMatrix();
  void buildFVector();

  void buildZMatrix();
  void stackSlackSolutions();

  Task task_, stacked_tasks_prev_, stacked_tasks_;
  HoQpPtr higher_problem_;

  bool has_eq_constraints_, has_ineq_constraints_;
  size_t num_slack_vars_, num_decision_vars_;
  matrix_t stacked_z_prev_, stacked_z_;
  vector_t stacked_slack_solutions_prev_, x_prev_;
  size_t num_prev_slack_vars_;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  vector_t c_, f_;
  vector_t stacked_slack_vars_, slack_vars_solutions_, decision_vars_solutions_;

  // Convenience matrices that are used multiple times
  matrix_t eye_nv_nv_;
  matrix_t zero_nv_nx_;
};

}  // namespace legged
