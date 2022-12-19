//
// Created by qiayuan on 2022/6/29.
//
#include <gtest/gtest.h>

#include "legged_wbc/HoQp.h"

using namespace legged;

TEST(HoQP, twoTask) {
  srand(0);
  Task task0, task1;
  task0.a_ = matrix_t::Random(2, 4);
  task0.b_ = vector_t::Ones(2);
  task0.d_ = matrix_t::Random(2, 4);
  task0.f_ = vector_t::Ones(2);
  task1 = task0;
  task1.a_ = matrix_t::Ones(2, 4);
  std::shared_ptr<HoQp> hoQp0 = std::make_shared<HoQp>(task0);
  std::shared_ptr<HoQp> hoQp1 = std::make_shared<HoQp>(task1, hoQp0);

  vector_t x0 = hoQp0->getSolutions(), x_1 = hoQp1->getSolutions();
  vector_t slack0 = hoQp0->getStackedSlackSolutions(), slack_1 = hoQp1->getStackedSlackSolutions();

  std::cout << x0.transpose() << std::endl;
  std::cout << x_1.transpose() << std::endl;
  std::cout << slack0.transpose() << std::endl;
  std::cout << slack_1.transpose() << std::endl;

  scalar_t prec = 1e-6;

  if (slack0.isApprox(vector_t::Zero(slack0.size()))) EXPECT_TRUE((task0.a_ * x0).isApprox(task0.b_, prec));
  if (slack_1.isApprox(vector_t::Zero(slack_1.size()))) {
    EXPECT_TRUE((task1.a_ * x_1).isApprox(task1.b_, prec));
    EXPECT_TRUE((task0.a_ * x_1).isApprox(task0.b_, prec));
  }

  vector_t y = task0.d_ * x0;
  for (int i = 0; i < y.size(); ++i) EXPECT_TRUE(y[i] <= task0.f_[i] + slack0[i]);
  y = task1.d_ * x_1;
  for (int i = 0; i < y.size(); ++i) EXPECT_TRUE(y[i] <= task1.f_[i] + slack_1[i]);
}
