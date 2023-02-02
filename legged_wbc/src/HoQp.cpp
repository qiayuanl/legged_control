//
// Created by qiayuan on 2022/6/28.
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#include "legged_wbc/HoQp.h"

#include <qpOASES.hpp>
#include <utility>

namespace legged {

HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
  initVars();
  formulateProblem();
  solveProblem();
  // For next problem
  buildZMatrix();
  stackSlackSolutions();
}

void HoQp::initVars() {
  // Task variables
  numSlackVars_ = task_.d_.rows();
  hasEqConstraints_ = task_.a_.rows() > 0;
  hasIneqConstraints_ = numSlackVars_ > 0;

  // Pre-Task variables
  if (higherProblem_ != nullptr) {
    stackedZPrev_ = higherProblem_->getStackedZMatrix();
    stackedTasksPrev_ = higherProblem_->getStackedTasks();
    stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions();
    xPrev_ = higherProblem_->getSolutions();
    numPrevSlackVars_ = higherProblem_->getSlackedNumVars();

    numDecisionVars_ = stackedZPrev_.cols();
  } else {
    numDecisionVars_ = std::max(task_.a_.cols(), task_.d_.cols());

    stackedTasksPrev_ = Task(numDecisionVars_);
    stackedZPrev_ = matrix_t::Identity(numDecisionVars_, numDecisionVars_);
    stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);
    xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);
    numPrevSlackVars_ = 0;
  }

  stackedTasks_ = task_ + stackedTasksPrev_;

  // Init convenience matrices
  eyeNvNv_ = matrix_t::Identity(numSlackVars_, numSlackVars_);
  zeroNvNx_ = matrix_t::Zero(numSlackVars_, numDecisionVars_);
}

void HoQp::formulateProblem() {
  buildHMatrix();
  buildCVector();
  buildDMatrix();
  buildFVector();
}

void HoQp::buildHMatrix() {
  matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);

  if (hasEqConstraints_) {
    // Make sure that all eigenvalues of A_t_A are non-negative, which could arise due to numerical issues
    matrix_t aCurrZPrev = task_.a_ * stackedZPrev_;
    zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * matrix_t::Identity(numDecisionVars_, numDecisionVars_);
    // This way of splitting up the multiplication is about twice as fast as multiplying 4 matrices
  } else {
    zTaTaz.setZero();
  }

  h_ = (matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_)  // clang-format off
            << zTaTaz, zeroNvNx_.transpose(),
                zeroNvNx_, eyeNvNv_)  // clang-format on
           .finished();
}

void HoQp::buildCVector() {
  vector_t c = vector_t::Zero(numDecisionVars_ + numSlackVars_);
  vector_t zeroVec = vector_t::Zero(numSlackVars_);

  vector_t temp(numDecisionVars_);
  if (hasEqConstraints_) {
    temp = (task_.a_ * stackedZPrev_).transpose() * (task_.a_ * xPrev_ - task_.b_);
  } else {
    temp.setZero();
  }

  c_ = (vector_t(numDecisionVars_ + numSlackVars_) << temp, zeroVec).finished();
}

void HoQp::buildDMatrix() {
  matrix_t stackedZero = matrix_t::Zero(numPrevSlackVars_, numSlackVars_);

  matrix_t dCurrZ;
  if (hasIneqConstraints_) {
    dCurrZ = task_.d_ * stackedZPrev_;
  } else {
    dCurrZ = matrix_t::Zero(0, numDecisionVars_);
  }

  // NOTE: This is upside down compared to the paper,
  // but more consistent with the rest of the algorithm
  d_ = (matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_)  // clang-format off
            << zeroNvNx_, -eyeNvNv_,
                stackedTasksPrev_.d_ * stackedZPrev_, stackedZero,
                dCurrZ, -eyeNvNv_)  // clang-format on
           .finished();
}

void HoQp::buildFVector() {
  vector_t zeroVec = vector_t::Zero(numSlackVars_);

  vector_t fMinusDXPrev;
  if (hasIneqConstraints_) {
    fMinusDXPrev = task_.f_ - task_.d_ * xPrev_;
  } else {
    fMinusDXPrev = vector_t::Zero(0);
  }

  f_ = (vector_t(2 * numSlackVars_ + numPrevSlackVars_) << zeroVec,
        stackedTasksPrev_.f_ - stackedTasksPrev_.d_ * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev)
           .finished();
}

void HoQp::buildZMatrix() {
  if (hasEqConstraints_) {
    assert((task_.a_.cols() > 0));
    stackedZ_ = stackedZPrev_ * (task_.a_ * stackedZPrev_).fullPivLu().kernel();
  } else {
    stackedZ_ = stackedZPrev_;
  }
}

void HoQp::solveProblem() {
  auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  qpProblem.setOptions(options);
  int nWsr = 20;

  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
  vector_t qpSol(numDecisionVars_ + numSlackVars_);

  qpProblem.getPrimalSolution(qpSol.data());

  decisionVarsSolutions_ = qpSol.head(numDecisionVars_);
  slackVarsSolutions_ = qpSol.tail(numSlackVars_);
}

void HoQp::stackSlackSolutions() {
  if (higherProblem_ != nullptr) {
    stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(), slackVarsSolutions_);
  } else {
    stackedSlackVars_ = slackVarsSolutions_;
  }
}

}  // namespace legged
