//
// Created by qiayuan on 2022/6/29.
//
#include <gtest/gtest.h>

#include "quad_wbc/ho_qp.h"

using namespace quad_ros;

class HoQpTest : public testing::Test
{
protected:
  HoQpTest()
  {
    a_ = matrix_t::Ones(2, 4);
    b_ = vector_t::Ones(2);
    d_ = matrix_t::Ones(2, 4);
    f_ = 10 * vector_t::Ones(2);
    std::shared_ptr<HoQp> ho_qp_0 = std::make_shared<HoQp>(Task(a_, b_, d_, f_));

    std::shared_ptr<HoQp> ho_qp_1 = std::make_shared<HoQp>(Task(a_, b_, d_, f_), ho_qp_0);
    //    HoQp ho_qp = HoQp(Task(a_, b_, d_, f_), std::make_shared<HoQp>(Task(a_, b_, d_, f_)));
    x_ = ho_qp_0->getSolutions();
  }
  matrix_t a_, d_;
  vector_t b_, f_, x_;
};

TEST_F(HoQpTest, oneTask)
{
  EXPECT_TRUE((a_ * x_).isApprox(b_));
  vector_t y = d_ * x_;
  for (int i = 0; i < y.size(); ++i)
    EXPECT_TRUE(y[i] <= f_[i]);
}
