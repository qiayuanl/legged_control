//
// Created by qiayuan on 2022/6/29.
//
#include <gtest/gtest.h>

#include "legged_wbc/ho_qp.h"

using namespace legged;

TEST(HoQP, twoTask)
{
  srand(0);
  Task task_0, task_1;
  task_0.a_ = matrix_t::Random(2, 4);
  task_0.b_ = vector_t::Ones(2);
  task_0.d_ = matrix_t::Random(2, 4);
  task_0.f_ = vector_t::Ones(2);
  task_1 = task_0;
  task_1.a_ = matrix_t::Ones(2, 4);
  std::shared_ptr<HoQp> ho_qp_0 = std::make_shared<HoQp>(task_0);
  std::shared_ptr<HoQp> ho_qp_1 = std::make_shared<HoQp>(task_1, ho_qp_0);

  vector_t x_0 = ho_qp_0->getSolutions(), x_1 = ho_qp_1->getSolutions();
  vector_t slack_0 = ho_qp_0->getStackedSlackSolutions(), slack_1 = ho_qp_1->getStackedSlackSolutions();

  std::cout << x_0.transpose() << std::endl;
  std::cout << x_1.transpose() << std::endl;
  std::cout << slack_0.transpose() << std::endl;
  std::cout << slack_1.transpose() << std::endl;

  scalar_t prec = 1e-6;

  if (slack_0.isApprox(vector_t::Zero(slack_0.size())))
    EXPECT_TRUE((task_0.a_ * x_0).isApprox(task_0.b_, prec));
  if (slack_1.isApprox(vector_t::Zero(slack_1.size())))
  {
    EXPECT_TRUE((task_1.a_ * x_1).isApprox(task_1.b_, prec));
    EXPECT_TRUE((task_0.a_ * x_1).isApprox(task_0.b_, prec));
  }

  vector_t y = task_0.d_ * x_0;
  for (int i = 0; i < y.size(); ++i)
    EXPECT_TRUE(y[i] <= task_0.f_[i] + slack_0[i]);
  y = task_1.d_ * x_1;
  for (int i = 0; i < y.size(); ++i)
    EXPECT_TRUE(y[i] <= task_1.f_[i] + slack_1[i]);
}
