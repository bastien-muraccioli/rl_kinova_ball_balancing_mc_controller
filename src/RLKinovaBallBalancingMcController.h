#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueJointTask.h>

#include "api.h"

#include "RLPolicyInterface.h"
#include "utils.h"
#include <string>
#include <vector>

struct RLKinovaBallBalancingMcController_DLLAPI RLKinovaBallBalancingMcController : public mc_control::fsm::Controller
{
  RLKinovaBallBalancingMcController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;
  void initializeRLObservation();

  // Task
  std::shared_ptr<mc_tasks::TorqueJointTask> torqueJointTask;
  
  int dofNumber = 0;

  // Public RL related variables
  Eigen::VectorXd q_rl;
  Eigen::VectorXd q_zero;               // Reference joint positions
  
  Eigen::VectorXd currentObservation;
  Eigen::VectorXd currentAction; // Raw output from the policy
  Eigen::VectorXd currentActionScaled; // Scaled action after applying actionScale, share the same size as q_rl, for the joints that are not controlled by the policy the value is 0 in currentActionScaled.

  Eigen::VectorXd actionScale;
  double policyStepSize; // Time interval between policy updates in seconds

  size_t currentPolicyIndex = 0;
  std::unique_ptr<RLPolicyInterface> rlPolicy;
  utils utilsClass; // Utility functions for RL controller

  std::string controlFrameName = "racquet_frame";

  Eigen::Vector6d plateWeightEffect;

  // observation data - Policy specific

  // // Old policy observation
  // Eigen::VectorXd jointPos;
  // Eigen::VectorXd jointVel;

  // Eigen::Vector3d eePos;
  // Eigen::Vector3d eeVel;
  // Eigen::Vector6d eeWrench;

  // observation
  static constexpr int HISTORY_SIZE = 5; // Number of past time steps to include in the observation
  std::array<Eigen::VectorXd, HISTORY_SIZE> jointPos, jointVel, jointAction;
  std::array<Eigen::Vector3d, HISTORY_SIZE> eePos, eeLinVel, eeAngVel;
  std::array<Eigen::Vector4d, HISTORY_SIZE> eeQuat;
  std::array<Eigen::Vector6d, HISTORY_SIZE> eeWrench;

private:
  mc_rtc::Configuration config_;
  void addLog();
  void addGui();

  void initializeRobot();
  void configRL();
  void initializeRLPolicy();
  void switchPolicy(int policyIndex);  // Switch to a different policy at runtime

  bool manageModeSwitching(); // Handle switching between Torque and Position control modes
  bool byPassQPControl(); // Directly use RL output without QP modifications

  Eigen::Vector6d sensorBias_;

  std::string robotName_;
  std::vector<std::string> jointNames_;

  // Mode switching
  bool useQP_ = true;
  bool isTorqueControl_ = false;
  bool controlModeChanged_ = false;

  // Constraint configuration
  double velPercent_ = 0.95;
  double dsPercent_ = 0.01;
  double diPercent_ = 0.1;

  // Gains
  double pdGainsRatio_ = 1.0;
  Eigen::VectorXd kp_;  // Gains set to the robot/simulator = pd_gains_ratio * kp_base
  Eigen::VectorXd kd_;  // Gains set to the robot/simulator = pd_gains_ratio * kd_base
  Eigen::VectorXd kpBase_; // Base RL PD gains from config
  Eigen::VectorXd kdBase_; // Base RL PD gains from config

  // RL
  std::vector<std::string> policyPaths_;
};
