#include "UDPPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

UDPPlugin::~UDPPlugin() {}

void UDPPlugin::init(mc_control::MCGlobalController &gc,
                           const mc_rtc::Configuration &config)
{
  
}

void UDPPlugin::reset(mc_control::MCGlobalController &controller) {}

void UDPPlugin::before(mc_control::MCGlobalController &gc)
{
}

void UDPPlugin::after(mc_control::MCGlobalController &controller)
{
}

mc_control::GlobalPlugin::GlobalPluginConfiguration
UDPPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

}  // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("UDPPlugin", mc_plugin::UDPPlugin)
