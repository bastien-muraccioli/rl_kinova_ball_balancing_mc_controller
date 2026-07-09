#include "RLKinovaBallBalancingMcController.h"

#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/src/Core/Matrix.h>

#include <mc_rtc/gui/Transform.h>

RLKinovaBallBalancingMcController::RLKinovaBallBalancingMcController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;
  currentPolicyIndex = size_t(config_("default_policy_index", 0));
  //Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 9.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, 1.9, 70.0}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);
  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));
  // Initialize Task
  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(
      solver(), robot().robotIndex(), 100.0, 1);
  solver().addTask(torqueJointTask);

  initializeRobot();
  initializeRLPolicy();

  addGui();
  addLog();

  // Kinova Dependant -----
  datastore().make<std::string>("TorqueMode", "Custom");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });
  mc_rtc::log::success("RLKinovaBallBalancingMcController init done");
}

bool RLKinovaBallBalancingMcController::run()
{
  bool run = manageModeSwitching();
  if(byPassQPControl()) // Run RL without taking the QP into account
  {
    return true;
  }
  return run; // Return false if QP fails
}

void RLKinovaBallBalancingMcController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void RLKinovaBallBalancingMcController::initializeRobot()
{
  useQP_ = config_("policies")[currentPolicyIndex]("use_QP", true);
  isTorqueControl_ = config_("policies")[currentPolicyIndex]("is_torque_control", false);
  if(isTorqueControl_)
  {
    mc_rtc::log::info("[RLKinovaBallBalancingMcController] Using Torque Control mode");
    datastore().make<std::string>("ControlMode", "Torque");
  }
  else
  {
    mc_rtc::log::info("[RLKinovaBallBalancingMcController] Using Position Control mode");
    datastore().make<std::string>("ControlMode", "Position");
  }

  // get the joints order (urdf) depending on the robot used
  robotName_ = robot().name();
  // Remove the floating base joints if they exist
  dofNumber = (robot().mb().nrJoints() > 0 && robot().mb().joint(0).type() == rbd::Joint::Free)
          ? robot().mb().nrDof() - 6
          : robot().mb().nrDof();

  q_rl = Eigen::VectorXd::Zero(dofNumber);
  q_zero = Eigen::VectorXd::Zero(dofNumber);
  kp_ = Eigen::VectorXd::Zero(dofNumber);
  kd_ = Eigen::VectorXd::Zero(dofNumber);
  kpBase_ = Eigen::VectorXd::Zero(dofNumber);
  kdBase_ = Eigen::VectorXd::Zero(dofNumber);

  actionScale = Eigen::VectorXd::Zero(dofNumber);
  currentActionScaled = Eigen::VectorXd::Zero(dofNumber);

  jointNames_ = robot().refJointOrder();
  
  // Get the gains from the configuration or set default values
  pdGainsRatio_ = config_("policies")[currentPolicyIndex]("pd_gains_ratio", 1.0);
  std::map<std::string, double> actionScale_map = config_("policies")[currentPolicyIndex]("action_scale");
  std::map<std::string, double> kp_map = config_("policies")[currentPolicyIndex]("kp");
  std::map<std::string, double> kd_map = config_("policies")[currentPolicyIndex]("kd");
  std::map<std::string, double> q0_map = config_("policies")[currentPolicyIndex]("q0");
  
  // Get the default posture target from the robot's posture task
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = getPostureTask(robot().name());
  auto posture = FSMPostureTask->posture();
  int i = 0;
  for (const auto &joint_name : jointNames_)
  {
    mc_rtc::log::info("[RLKinovaBallBalancingMcController] Found joint: {}", joint_name);
    if (const auto &t = posture[robot().jointIndexByName(joint_name)]; !t.empty()) {
        kpBase_[i] = kp_map.at(joint_name);
        kdBase_[i] = kd_map.at(joint_name);
        q_zero[i] = q0_map.at(joint_name);
        q_rl[i] = t[0];
        actionScale[i] = actionScale_map.at(joint_name);
        i++;
    }
  }

  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = sqrt(pdGainsRatio_) * kdBase_;
  torqueJointTask->setStiffness(kp_);
  torqueJointTask->setDamping(kd_);
}

void RLKinovaBallBalancingMcController::initializeRLPolicy()
{
  // load policy specific configuration
  policyPaths_ = config_("policy_path", std::vector<std::string>{"walking_better_h1.onnx"});
  configRL();
  currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
  currentAction = Eigen::VectorXd::Zero(rlPolicy->getActionSize());

  auto & real_robot = realRobot(robots()[0].name());
  sensorBias_ = Eigen::Vector6d::Zero();
  auto wrench = real_robot.forceSensor("EEForceSensor").wrench();
  sensorBias_.head<3>() = wrench.force();
  sensorBias_.tail<3>() = wrench.couple();

  mc_rtc::log::info("[RLKinovaBallBalancingMcController] Sensor bias for EEForceSensor: {}", sensorBias_.transpose());

  initializeRLObservation();

  for (int i = 0; i < HISTORY_SIZE; ++i) {
    jointPos[i] = jointPos[0]; 
    jointVel[i] = jointVel[0];
    jointAction[i] = jointAction[0];
    eePos[i] = eePos[0];
    eeQuat[i] = eeQuat[0];
    eeLinVel[i] = eeLinVel[0];
    eeAngVel[i] = eeAngVel[0];
    eeWrench[i] = eeWrench[0];
  }
}

void RLKinovaBallBalancingMcController::initializeRLObservation()
{ 
  auto & real_robot = realRobot(robots()[0].name());
  Eigen::VectorXd q = Eigen::VectorXd::Zero(dofNumber);
  Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(dofNumber);
  const auto & q_mbc = real_robot.mbc().q;
  const auto & q_dot_mbc = real_robot.mbc().alpha;
  size_t i = 0;
  for(const auto &joint_name : jointNames_)
  {
    size_t mbcIndex = real_robot.mb().jointIndexByName(joint_name);
    if(mbcIndex >= q_mbc.size() || mbcIndex >= q_dot_mbc.size())
    {
      mc_rtc::log::error("Joint '{}' not found in the robot's MBC. Skipping.", joint_name);
      continue;
    }
    if(q_mbc[mbcIndex].empty()) { continue; }

    q[i] = q_mbc[mbcIndex][0];
    q_dot[i] = q_dot_mbc[mbcIndex][0];
    i++;
  }

  sva::PTransformd eePosW = real_robot.bodyPosW(controlFrameName);
  Eigen::Vector3d eeCurrentPosition = eePosW.translation();

  Eigen::Vector3d eeCurrentLinearVel = real_robot.bodyVelW(controlFrameName).linear();
  Eigen::Vector3d eeCurrentAngularVel = real_robot.bodyVelW(controlFrameName).angular();

  Eigen::Vector4d eeCurrentOrientation = Eigen::Vector4d::Zero();
  Eigen::Quaterniond quat = Eigen::Quaterniond(eePosW.rotation().transpose()).normalized();
  eeCurrentOrientation << quat.w(), quat.x(), quat.y(), quat.z();

  // sva::ForceVecd wrench = real_robot.forceSensor("EEForceSensor").wrenchWithoutGravity(real_robot);
  sva::ForceVecd wrench = real_robot.forceSensor("EEForceSensor").wrench();
  Eigen::Vector6d eeCurrentWrench = Eigen::Vector6d::Zero();
  eeCurrentWrench.head<3>() = wrench.force();
  eeCurrentWrench.tail<3>() = wrench.couple();

  jointPos[0] = q - q_zero;
  jointVel[0] = q_dot;
  jointAction[0] = currentAction;
  eePos[0] = eeCurrentPosition;
  eeQuat[0] = eeCurrentOrientation;
  eeLinVel[0] = eeCurrentLinearVel;
  eeAngVel[0] = eeCurrentAngularVel;
  eeWrench[0] = eeCurrentWrench;// - sensorBias_;

  mc_rtc::log::info("jointPos[0]: {}", jointPos[0].transpose());
  mc_rtc::log::info("jointVel[0]: {}", jointVel[0].transpose());
  mc_rtc::log::info("jointAction[0]: {}", jointAction[0].transpose());
  mc_rtc::log::info("eePos[0]: {}", eePos[0].transpose());
  mc_rtc::log::info("eeQuat[0]: {}", eeQuat[0].transpose());
  mc_rtc::log::info("eeLinVel[0]: {}", eeLinVel[0].transpose());
  mc_rtc::log::info("eeAngVel[0]: {}", eeAngVel[0].transpose());
  mc_rtc::log::info("eeWrench[0]: {}", eeWrench[0].transpose());
}

void RLKinovaBallBalancingMcController::switchPolicy(int policyIndex)
{
  if(policyIndex < 0 || policyIndex >= static_cast<int>(policyPaths_.size())) {
    mc_rtc::log::error("Invalid policy index: {}", policyIndex);
    return;
  }
  
  mc_rtc::log::info("Switching from policy [{}] to policy [{}]", currentPolicyIndex, policyIndex);
  currentPolicyIndex = size_t(policyIndex);
  
  // Update policy-specific boolean flags
  useQP_ = config_("policies")[currentPolicyIndex]("use_QP", true);
  isTorqueControl_ = config_("policies")[currentPolicyIndex]("is_torque_control", false);
  if(isTorqueControl_) datastore().get<std::string>("ControlMode") = "Torque";
  else datastore().get<std::string>("ControlMode") = "Position";

  configRL();

  // Update PD gains
  pdGainsRatio_ = config_("policies")[currentPolicyIndex]("pd_gains_ratio", 1.0);
  std::map<std::string, double> kp_map = config_("policies")[currentPolicyIndex]("kp");
  std::map<std::string, double> kd_map = config_("policies")[currentPolicyIndex]("kd");
  std::map<std::string, double> q0_map = config_("policies")[currentPolicyIndex]("q0");
  std::map<std::string, double> actionScale_map = config_("policies")[currentPolicyIndex]("action_scale");

  for(int i = 0; i < dofNumber; ++i) {
    const auto & jName = robot().mb().joint(static_cast<int>(i + 1)).name();  // +1 to skip Root
    if(kp_map.count(jName)) {
      kpBase_(i) = kp_map[jName];
    }
    if(kd_map.count(jName)) {
      kdBase_(i) = kd_map[jName];
    }
    if(q0_map.count(jName)) {
      q_zero[i] = q0_map[jName];
    }
    if(actionScale_map.count(jName)) {
      actionScale[i] = actionScale_map[jName];
    }
  }
  // Update PD gains
  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = sqrt(pdGainsRatio_) * kdBase_;
  torqueJointTask->setStiffness(kp_);
  torqueJointTask->setDamping(kd_);
}

bool RLKinovaBallBalancingMcController::byPassQPControl()
{
  if(useQP_) return false; // QP is not bypassed, do nothing
  if(!isTorqueControl_)
  {
    mc_rtc::log::warning("[RLKinovaBallBalancingMcController] QP can't be bypassed in position control mode.");
    mc_rtc::log::warning("[RLKinovaBallBalancingMcController] QP is enforced.");
    return false;
  }

  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  auto tau = robot().mbc().jointTorque;
  auto q_map = robot().encoderValues();
  auto q_dot_map = robot().encoderVelocities();

  Eigen::VectorXd q = Eigen::VectorXd::Map(q_map.data(), int(q_map.size()));
  Eigen::VectorXd q_dot = Eigen::VectorXd::Map(q_dot_map.data(), int(q_dot_map.size()));
  Eigen::VectorXd tau_rl = (kp_).cwiseProduct(q_rl - q) - (kd_).cwiseProduct(q_dot);
  
  int i = 0;
  for (const auto &joint_name : jointNames_)
  {
    tau[robot().jointIndexByName(joint_name)][0] = tau_rl[i];
    i++;
  }
  // Update joint torques 
  robot().mbc().jointTorque = tau;
  return true;
}

void RLKinovaBallBalancingMcController::addLog()
{
  // Robot State variables
  logger().addLogEntry("RLKinovaBallBalancingMcController_kp_base", [this]() { return kpBase_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_kd_base", [this]() { return kdBase_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_kp_current", [this]() { return kp_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_kd_current", [this]() { return kd_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_pd_gains_ratio", [this]() { return pdGainsRatio_; });

  // RL variables
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_q", [this]() { return q_rl; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_qZero", [this]() { return q_zero; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_currentObservation", [this]() { return currentObservation; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_RL_currentAction", [this]() { return currentAction; });
  
  // Controller state variables
  logger().addLogEntry("RLKinovaBallBalancingMcController_useQP", [this]() { return useQP_; });
  logger().addLogEntry("RLKinovaBallBalancingMcController_isTorqueControl", [this]() { return isTorqueControl_; });

  // Log observation
  for (int i = 0; i < HISTORY_SIZE; ++i) {
    logger().addLogEntry("RLKinovaBallBalancingMcController_eePos_" + std::to_string(i), [this, i]() { return eePos[i]; });
    logger().addLogEntry("RLKinovaBallBalancingMcController_eeLinVel_" + std::to_string(i), [this, i]() { return eeLinVel[i]; });
    logger().addLogEntry("RLKinovaBallBalancingMcController_eeAngVel_" + std::to_string(i), [this, i]() { return eeAngVel[i]; });
    logger().addLogEntry("RLKinovaBallBalancingMcController_eeQuat_" + std::to_string(i), [this, i]() { return eeQuat[i]; });
    logger().addLogEntry("RLKinovaBallBalancingMcController_eeWrench_" + std::to_string(i), [this, i]() { return eeWrench[i]; });
  }
}

void RLKinovaBallBalancingMcController::addGui()
{
  gui()->addElement({"RLKinovaBallBalancingMcController", "Policy"},
  mc_rtc::gui::Label("Current policy", [this]() -> const std::string & 
    { 
      return policyPaths_[currentPolicyIndex]; 
    }),
    mc_rtc::gui::ComboInput(
      "Select policy",
      policyPaths_,
      [this]() -> const std::string & 
      { 
        return policyPaths_[currentPolicyIndex]; 
      },
      [this](const std::string & selected) 
      {  // Capture config by VALUE (makes a safe copy)
        // Find the index of the selected policy
        auto it = std::find(policyPaths_.begin(), policyPaths_.end(), selected);
        if(it != policyPaths_.end()) 
        {
          int newIndex = static_cast<int>(std::distance(policyPaths_.begin(), it));
          mc_rtc::log::info("User requested policy switch to [{}]: {}", newIndex, selected);
          // Switch to new policy without reinitializing robot
          switchPolicy(newIndex);
        }
      }),
    mc_rtc::gui::Button("Reload current policy", [this]() 
    {
      mc_rtc::log::info("User requested to reload current policy [{}]", currentPolicyIndex);
      switchPolicy(int(currentPolicyIndex));
    })
  );

  // Add PD gains ratio slider
  gui()->addElement({"RLKinovaBallBalancingMcController", "PD Gains"},
    mc_rtc::gui::NumberSlider(
      "PD Gains Ratio", [this]() { return pdGainsRatio_; },
      [this](double v) { 
        pdGainsRatio_ = v;
        kp_ = pdGainsRatio_ * kpBase_;
        kd_ = sqrt(pdGainsRatio_) * kdBase_;
        torqueJointTask->setStiffness(kp_);
        torqueJointTask->setDamping(kd_);
      }, 0.0, 2.0),
    mc_rtc::gui::Label("Current kp", kp_),
    mc_rtc::gui::Label("Current kd", kd_)
  );

  gui()->addElement({"ControlMode"}, 
    mc_rtc::gui::Button("Switch Control Mode", [this]()
      {
        controlModeChanged_ = true;
        isTorqueControl_ = !isTorqueControl_;
      }),
      mc_rtc::gui::Label("Current Control Mode", [this]()
        {
          return isTorqueControl_ ? "Torque Control" : "Position Control";
        }),
      mc_rtc::gui::Button("Toggle QP Control", [this]()
        {
          useQP_ = !useQP_;
        }),
      mc_rtc::gui::Label("QP Control", [this]()
      {
        return useQP_ ? "Enforced" : "Bypassed";
      })
    );
}

void RLKinovaBallBalancingMcController::configRL()
{
  mc_rtc::log::info("Loading RL policy [{}]: {}", currentPolicyIndex, policyPaths_[currentPolicyIndex]);
  try {
    rlPolicy = std::make_unique<RLPolicyInterface>(policyPaths_[currentPolicyIndex]);
    if(rlPolicy) {
      mc_rtc::log::success("RL policy loaded successfully");
      // Initialize observation vector with the correct size from the loaded policy
      currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
      mc_rtc::log::info("Initialized observation vector with size: {}", rlPolicy->getObservationSize());
      currentAction = Eigen::VectorXd::Zero(rlPolicy->getActionSize());
      mc_rtc::log::info("Initialized action vector with size: {}", rlPolicy->getActionSize());
    } else {
      mc_rtc::log::error_and_throw("RL policy creation failed - policy is null");
    }
  } catch(const std::exception& e) {
    mc_rtc::log::error_and_throw("Failed to load RL policy: {}", e.what());
  }
  policyStepSize = config_("policies")[currentPolicyIndex]("policy_step_size", 0.01);

  int physicsStepSize = config_("policies")[currentPolicyIndex]("physics_step_size", 0.001);
  if(physicsStepSize - timeStep > 1e-6) {
    mc_rtc::log::warning("Physics step size ({:.3f} s) is larger than controller time step ({:.3f} s). This may cause issues with the policy. Consider fixing the controller time step.", physicsStepSize, timeStep);
  }
}

bool RLKinovaBallBalancingMcController::manageModeSwitching()
{
  if(controlModeChanged_)
  {
    if(isTorqueControl_)
    {
      mc_rtc::log::info("Switching to Torque Control");
      datastore().assign<std::string>("ControlMode", "Torque");
    }
    else
    {
      mc_rtc::log::info("Switching to Position Control");
      datastore().assign<std::string>("ControlMode", "Position");
    }
    controlModeChanged_ = false;
  }

  if(isTorqueControl_)
  {
    return mc_control::fsm::Controller::run(
          mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
  else 
  {
    return mc_control::fsm::Controller::run();
  }
}