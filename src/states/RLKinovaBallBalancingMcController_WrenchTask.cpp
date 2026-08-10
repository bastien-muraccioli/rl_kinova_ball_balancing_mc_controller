#include "RLKinovaBallBalancingMcController_WrenchTask.h"

#include "../RLKinovaBallBalancingMcController.h"

void RLKinovaBallBalancingMcController_WrenchTask::configure(const mc_rtc::Configuration & config) {}

void RLKinovaBallBalancingMcController_WrenchTask::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
  ctl.solver().addTask(ctl.torqueJointTask);
  // ctl.solver().addTask(ctl.wrenchTask);
  ctl.solver().addTask(ctl.torqueCartesianTask_test);
}

bool RLKinovaBallBalancingMcController_WrenchTask::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  // ctl.wrenchTask->targetWrench(ctl.wrenchTask_target);
  ctl.torqueCartesianTask_test->setTorqueFeedforward(ctl.tau_d);
  return false;
}

void RLKinovaBallBalancingMcController_WrenchTask::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.teardown_rl_state(ctl);
  ctl.solver().removeTask(ctl.torqueJointTask);
  // ctl.solver().removeTask(ctl.wrenchTask);
  ctl.solver().removeTask(ctl.torqueCartesianTask_test);
}

EXPORT_SINGLE_STATE("RLKinovaBallBalancingMcController_WrenchTask", RLKinovaBallBalancingMcController_WrenchTask)
