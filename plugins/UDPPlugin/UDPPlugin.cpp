#include "UDPPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include "UDPRobotControl.h"

namespace mc_plugin
{

struct UDPPluginSchema
{
  MC_RTC_NEW_SCHEMA(UDPPluginSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(UDPPluginSchema, __VA_ARGS__))
  using UDPRobotSchemaMap = std::map<std::string, UDPRobotSchema>;
  MEMBER(UDPRobotSchemaMap, robots, "UDP configuration of the robots to control");
#undef MEMBER
};

UDPPlugin::~UDPPlugin() {}

void UDPPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("Plugin config is:\n{}", config.dump(true, true));

  UDPPluginSchema udpConfig = config;
  if(udpConfig.robots.empty())
  {
    mc_rtc::log::warning("[UDPPlugin] No robot configured");
  }

  for(const auto & [robotName, robotUDPConfig] : udpConfig.robots)
  {
    udpRobotControls_.emplace_back(new UDPRobotControl(gc, robotName, robotUDPConfig));
  }
}

void UDPPlugin::reset(mc_control::MCGlobalController & controller) {}

void UDPPlugin::before(mc_control::MCGlobalController & gc)
{
  for(auto & udpRobotControl : udpRobotControls_)
  {
    udpRobotControl->updateSensors();
  }
}

void UDPPlugin::after(mc_control::MCGlobalController & controller)
{
  for(auto & udpRobotControl : udpRobotControls_)
  {
    udpRobotControl->updateControl();
  }
}

mc_control::GlobalPlugin::GlobalPluginConfiguration UDPPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("UDPPlugin", mc_plugin::UDPPlugin)
