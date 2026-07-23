#include "RLKinovaBallBalancingMcController_WrenchTask.h"

#include "../RLKinovaBallBalancingMcController.h"

void RLKinovaBallBalancingMcController_WrenchTask::configure(const mc_rtc::Configuration & config) {}

void RLKinovaBallBalancingMcController_WrenchTask::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
  ctl.solver().addTask(ctl.torqueJointTask);
  ctl.solver().addTask(ctl.wrenchTask);
}

bool RLKinovaBallBalancingMcController_WrenchTask::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_rl);
  ctl.wrenchTaskUpdate(); // Update ctl.wrenchTask_target
  ctl.wrenchTask->targetWrench(ctl.wrenchTask_target);
  return false;
}

void RLKinovaBallBalancingMcController_WrenchTask::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.teardown_rl_state(ctl);
  ctl.solver().removeTask(ctl.torqueJointTask);
  ctl.solver().removeTask(ctl.wrenchTask);
}

EXPORT_SINGLE_STATE("RLKinovaBallBalancingMcController_WrenchTask", RLKinovaBallBalancingMcController_WrenchTask)
