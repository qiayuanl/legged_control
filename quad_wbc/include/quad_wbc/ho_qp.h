//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include "quad_wbc/task.h"

#include <memory>

namespace quad_ros
{
// Hierarchical Optimization Quadratic Program
class HoQp
{
public:
  using HoQpPtr = std::shared_ptr<HoQp>;
  HoQp(const Task& task);
  HoQp(const Task&, HoQpPtr higher_problem);

  matrix_t getStackedHMatrix() const
  {
    return stacked_z_;
  }

  Task getStackedTasks() const
  {
    return Task();
  }

  vector_t getStackedSlackSolutions() const
  {
    return vector_t::Zero(num_slack_vars_);
  }

  vector_t getSolutions() const
  {
    return vector_t::Zero(num_slack_vars_);
  }

  size_t getSlackedNumVars() const
  {
    return stacked_tasks_.d_.rows();
  }

private:
  void initVars();
  void formulateProblem();
  void solveProblem();

  void buildZMatrix();
  void buildHMatrix();
  void buildCVector();
  void buildDMatrix();
  void buildFVector();

  Task task_, stacked_tasks_prev_, stacked_tasks_;
  HoQpPtr higher_problem_;

  bool has_eq_constraints_, has_ineq_constraints_;
  size_t num_slack_vars_, num_decision_vars_;
  matrix_t stacked_z_prev_, stacked_z_;
  vector_t stacked_slack_solutions_prev_, x_prev_;
  size_t num_prev_slack_vars_;

  matrix_t h_, c_, d_, f_;

  // Convenience matrices that are used multiple times
  matrix_t eye_nv_nv_;
  matrix_t zero_nv_nx_;
  matrix_t a_curr_z_prev_;
};

}  // namespace quad_ros