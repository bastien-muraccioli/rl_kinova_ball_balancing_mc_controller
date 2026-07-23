#include "RLKinovaBallBalancingMcController_Initial.h"

#include "../RLKinovaBallBalancingMcController.h"

void RLKinovaBallBalancingMcController_Initial::configure(const mc_rtc::Configuration & config) {}

void RLKinovaBallBalancingMcController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  ctl.solver().addTask(ctl.torqueJointTask);
}

bool RLKinovaBallBalancingMcController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.run_rl_state(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.q_zero);
  return false;
}

void RLKinovaBallBalancingMcController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  ctl.utilsClass.teardown_rl_state(ctl);
  ctl.solver().removeTask(ctl.torqueJointTask);
}

EXPORT_SINGLE_STATE("RLKinovaBallBalancingMcController_Initial", RLKinovaBallBalancingMcController_Initial)
