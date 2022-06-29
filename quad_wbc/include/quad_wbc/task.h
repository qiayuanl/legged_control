//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include <ocs2_core/Types.h>

namespace quad_ros
{
using namespace ocs2;

class Task
{
public:
  Task() = default;
  Task(const matrix_t& a, const vector_t& b, const matrix_t& d, const vector_t& f) : a_(a), d_(d), b_(b), f_(f)
  {
  }

  Task(size_t num_decision_vars)
    : Task(matrix_t::Zero(0, num_decision_vars), vector_t::Zero(0), matrix_t::Zero(0, num_decision_vars),
           vector_t::Zero(0))
  {
  }

  Task operator+(const Task& rhs)
  {
    return Task(concatenateMatrices(a_, rhs.a_), concatenateMatrices(d_, rhs.d_), concatenateMatrices(b_, rhs.b_),
                concatenateMatrices(f_, rhs.f_));
  }
  matrix_t a_, d_;
  vector_t b_, f_;

private:
  matrix_t concatenateMatrices(matrix_t m1, matrix_t m2)
  {
    if (m1.cols() == 0)
      return m2;
    else if (m2.cols() == 0)
      return m1;

    assert(m1.cols() == m2.cols());
    matrix_t res(m1.rows() + m2.rows(), m1.cols());
    res << m1, m2;

    return res;
  }
};

}  // namespace quad_ros
