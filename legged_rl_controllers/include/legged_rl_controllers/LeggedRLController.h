//
// Created by luohx on 22-12-5.
//

#pragma once

#include "legged_rl_controllers/RLControllerBase.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedRLController : public RLControllerBase {
  using tensor_element_t = float;

 public:
  LeggedRLController() = default;
  ~LeggedRLController() override = default;

 protected:
  bool loadModel(ros::NodeHandle& nh) override;
  bool loadRLCfg(ros::NodeHandle& nh) override;
  void computeActions() override;
  void computeObservation() override;
  void handleWalkMode() override;

 private:
  // onnx policy model
  std::string policyFilePath_;
  std::shared_ptr<Ort::Env> onnxEnvPrt_;
  std::unique_ptr<Ort::Session> sessionPtr_;
  std::vector<const char*> inputNames_;
  std::vector<const char*> outputNames_;
  std::vector<std::vector<int64_t>> inputShapes_;
  std::vector<std::vector<int64_t>> outputShapes_;

  RLRobotCfg robotCfg_{};
  vector3_t baseLinVel_;
  vector3_t basePosition_;
  vector_t lastActions_;
  vector_t defaultJointAngles_;

  int actionsSize_;
  int observationSize_;
  size_t index[12] = {0, 1, 2, 6, 7, 8, 3, 4, 5, 9, 10 ,11};
  std::vector<tensor_element_t> actions_;
  std::vector<tensor_element_t> observations_;
};

}  // namespace legged
