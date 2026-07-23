#include "RLKinovaBallBalancingMcController_WrenchTaskNS.h"

#include "../RLKinovaBallBalancingMcController.h"

void RLKinovaBallBalancingMcController_WrenchTaskNS::configure(const mc_rtc::Configuration & config) {}

void RLKinovaBallBalancingMcController_WrenchTaskNS::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
  ctl.compliantPostureTask->target(ctl.posture_init_default);
  ctl.compliantPostureTask->damping(5.0);
  ctl.compliantPostureTask->stiffness(0.0);
  ctl.solver().addTask(ctl.compliantPostureTask);
  ctl.solver().addTask(ctl.wrenchTask);
}

bool RLKinovaBallBalancingMcController_WrenchTaskNS::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.wrenchTaskUpdate();
  ctl.wrenchTask->targetWrench(ctl.wrenchTask_target);
  return false;
}

void RLKinovaBallBalancingMcController_WrenchTaskNS::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.teardown_rl_state(ctl);
  ctl.solver().removeTask(ctl.compliantPostureTask);
  ctl.solver().removeTask(ctl.wrenchTask);
}

EXPORT_SINGLE_STATE("RLKinovaBallBalancingMcController_WrenchTaskNS", RLKinovaBallBalancingMcController_WrenchTaskNS)
