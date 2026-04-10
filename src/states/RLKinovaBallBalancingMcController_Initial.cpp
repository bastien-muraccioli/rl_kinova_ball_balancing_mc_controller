#include "RLKinovaBallBalancingMcController_Initial.h"

#include "../RLKinovaBallBalancingMcController.h"

void RLKinovaBallBalancingMcController_Initial::configure(const mc_rtc::Configuration & config) {}

void RLKinovaBallBalancingMcController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
}

bool RLKinovaBallBalancingMcController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
  output("OK");
  return true;
}

void RLKinovaBallBalancingMcController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<RLKinovaBallBalancingMcController &>(ctl_);
}

EXPORT_SINGLE_STATE("RLKinovaBallBalancingMcController_Initial", RLKinovaBallBalancingMcController_Initial)
