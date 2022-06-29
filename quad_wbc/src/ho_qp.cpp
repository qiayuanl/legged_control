//
// Created by qiayuan on 2022/6/28.
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#include "quad_wbc/ho_qp.h"

#include <utility>
#include <qpOASES.hpp>

namespace quad_ros
{
HoQp::HoQp(const Task& task) : HoQp(task, nullptr)
{
}

HoQp::HoQp(const Task& task, HoQp::HoQpPtr higher_problem) : task_(task), higher_problem_(std::move(higher_problem))
{
  initVars();
  formulateProblem();
  solveProblem();
  // For next problem
  buildZMatrix();
  stackSlackSolutions();
}

void HoQp::initVars()
{
  // Task variables
  has_eq_constraints_ = task_.a_.rows() > 0;
  has_ineq_constraints_ = num_slack_vars_ > 0;
  num_slack_vars_ = task_.d_.rows();

  // Pre-Task variables
  bool has_higher_problem = higher_problem_ != nullptr;
  if (has_higher_problem)
  {
    stacked_z_prev_ = higher_problem_->getStackedZMatrix();
    stacked_tasks_prev_ = higher_problem_->getStackedTasks();
    stacked_slack_solutions_prev_ = higher_problem_->getStackedSlackSolutions();
    x_prev_ = higher_problem_->getSolutions();
    num_prev_slack_vars_ = higher_problem_->getSlackedNumVars();

    num_decision_vars_ = stacked_z_prev_.cols();
  }
  else
  {
    num_decision_vars_ = std::max(task_.a_.cols(), task_.d_.cols());

    stacked_tasks_prev_ = Task(num_decision_vars_);
    stacked_z_prev_ = matrix_t::Identity(num_decision_vars_, num_decision_vars_);
    stacked_slack_solutions_prev_ = Eigen::VectorXd::Zero(0);
    x_prev_ = Eigen::VectorXd::Zero(num_decision_vars_);
    num_prev_slack_vars_ = 0;
  }

  stacked_tasks_ = task_ + stacked_tasks_prev_;

  // Init convenience matrices
  eye_nv_nv_ = matrix_t::Identity(num_slack_vars_, num_slack_vars_);
  zero_nv_nx_ = matrix_t::Zero(num_slack_vars_, num_decision_vars_);
}

void HoQp::formulateProblem()
{
  buildHMatrix();
  buildCVector();
  buildDMatrix();
  buildFVector();
}

void HoQp::buildHMatrix()
{
  matrix_t h = matrix_t::Zero(num_decision_vars_ + num_slack_vars_, num_decision_vars_ + num_slack_vars_);

  Eigen::MatrixXd z_t_a_t_a_z(num_decision_vars_, num_decision_vars_);
  if (has_eq_constraints_)
  {
    // Make sure that all eigenvalues of A_t_A are non-negative,
    // which could arise due to numerical issues
    matrix_t a_curr_z_prev = task_.a_ * stacked_z_prev_;
    z_t_a_t_a_z =
        a_curr_z_prev.transpose() * a_curr_z_prev + 1e-12 * matrix_t::Identity(num_decision_vars_, num_decision_vars_);
    // This way of splitting up the multiplication is about twice as fast as multiplying 4 matrices
  }
  else
  {
    z_t_a_t_a_z.setZero();
  }

  h << z_t_a_t_a_z, zero_nv_nx_.transpose(), zero_nv_nx_, eye_nv_nv_;
  h_ = h;
}

void HoQp::buildCVector()
{
  vector_t c = vector_t::Zero(num_decision_vars_ + num_slack_vars_);
  vector_t zero_vec = vector_t::Zero(num_slack_vars_);

  vector_t temp(num_decision_vars_);
  if (has_eq_constraints_)
    temp = (task_.a_ * stacked_z_prev_).transpose() * (task_.a_ * x_prev_ - task_.b_);
  else
    temp.setZero();

  c << temp, zero_vec;
  c_ = c;
}

void HoQp::buildDMatrix()
{
  matrix_t d(2 * num_slack_vars_ + num_prev_slack_vars_, num_decision_vars_ + num_slack_vars_);
  d.setZero();

  matrix_t stacked_zero = matrix_t::Zero(num_prev_slack_vars_, num_slack_vars_);

  matrix_t d_curr_z;
  if (has_ineq_constraints_)
    d_curr_z = task_.d_ * stacked_z_prev_;
  else
    d_curr_z = matrix_t::Zero(0, num_decision_vars_);

  // NOTE: This is upside down compared to the paper,
  // but more consistent with the rest of the algorithm
  d << zero_nv_nx_, -eye_nv_nv_, stacked_tasks_prev_.d_ * stacked_z_prev_, stacked_zero, d_curr_z, -eye_nv_nv_;

  d_ = d;
}

void HoQp::buildFVector()
{
  vector_t f = vector_t::Zero(2 * num_slack_vars_ + num_prev_slack_vars_);

  vector_t zero_vec = vector_t::Zero(num_slack_vars_);

  vector_t f_minus_d_x_prev;
  if (has_ineq_constraints_)
    f_minus_d_x_prev = task_.f_ - task_.d_ * x_prev_;
  else
    f_minus_d_x_prev = vector_t::Zero(0);

  f << zero_vec, stacked_tasks_prev_.f_ - stacked_tasks_prev_.d_ * x_prev_ + stacked_slack_solutions_prev_,
      f_minus_d_x_prev;

  f_ = f;
}

void HoQp::buildZMatrix()
{
  if (has_eq_constraints_)
  {
    matrix_t a = task_.a_;
    assert((a.cols() > 0));
    stacked_z_ = stacked_z_prev_ * a.fullPivLu().kernel();
  }
  else
    stacked_z_ = stacked_z_prev_;
}

void HoQp::solveProblem()
{
  auto qp_problem = qpOASES::QProblem(d_.cols(), d_.rows());
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp_problem.setOptions(options);
  int n_wsr = 10;
  qpOASES::returnValue rvalue =
      qp_problem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), n_wsr);

  if (rvalue != qpOASES::SUCCESSFUL_RETURN)
    return;

  std::vector<qpOASES::real_t> qp_sol(d_.rows(), 0);
  qp_problem.getPrimalSolution(qp_sol.data());

  decision_vars_solutions_.resize(num_decision_vars_);
  slack_vars_solutions_.resize(num_slack_vars_);
  for (size_t i = 0; i < num_decision_vars_; ++i)
    decision_vars_solutions_[i] = qp_sol[i];
  for (size_t i = 0; i < num_slack_vars_; ++i)
    slack_vars_solutions_[i] = qp_sol[i + num_decision_vars_];
}

void HoQp::stackSlackSolutions()
{
  if (higher_problem_ != nullptr)
    stacked_slack_vars_ = slack_vars_solutions_;
  else
    stacked_slack_vars_ = concatenateVectors(higher_problem_->getStackedSlackSolutions(), slack_vars_solutions_);
}

vector_t HoQp::concatenateVectors(const vector_t& v1, const vector_t& v2)
{
  if (v1.cols() == 0)
    return v2;
  else if (v2.cols() == 0)
    return v1;
  assert(v1.cols() == v2.cols());
  vector_t res(v1.rows() + v2.rows());
  res << v1, v2;

  return res;
}

}  // namespace quad_ros
