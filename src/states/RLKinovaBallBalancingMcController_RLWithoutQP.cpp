#include "RLKinovaBallBalancingMcController_RLWithoutQP.h"

#include "../RLKinovaBallBalancingMcController.h"

void RLKinovaBallBalancingMcController_RLWithoutQP::configure(const mc_rtc::Configuration & config) {}

void RLKinovaBallBalancingMcController_RLWithoutQP::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.start_rl_state(ctl, "RL_State");
  ctl.useQP = false; // Force QP to be bypassed in this state
}

bool RLKinovaBallBalancingMcController_RLWithoutQP::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  return false;
}

void RLKinovaBallBalancingMcController_RLWithoutQP::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.teardown_rl_state(ctl);
  ctl.useQP = true; // Restore QP control after exiting this state
}

EXPORT_SINGLE_STATE("RLKinovaBallBalancingMcController_RLWithoutQP", RLKinovaBallBalancingMcController_RLWithoutQP)
