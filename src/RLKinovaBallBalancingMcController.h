#pragma once

#include <mc_control/fsm/Controller.h>

#include "api.h"

struct RLKinovaBallBalancingMcController_DLLAPI RLKinovaBallBalancingMcController : public mc_control::fsm::Controller
{
  RLKinovaBallBalancingMcController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  mc_rtc::Configuration config_;
};
