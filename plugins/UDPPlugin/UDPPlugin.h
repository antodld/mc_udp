/*
 * Copyright 2021 CNRS-UM LIRMM
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

namespace mc_plugin
{

struct UDPPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController &controller,
            const mc_rtc::Configuration &config) override;

  void reset(mc_control::MCGlobalController &controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController &controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~UDPPlugin() override;
};

}  // namespace mc_plugin
