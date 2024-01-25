#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mc_udp/client/Client.h>
#include <mc_udp/data/RobotControl.h>
#include <atomic>
#include <condition_variable>
#include <thread>

namespace mc_control
{
struct MCGlobalController;
}

namespace mc_plugin
{
struct UDPIgnoredJointsSchema
{
  MC_RTC_NEW_SCHEMA(UDPIgnoredJointsSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(UDPIgnoredJointsSchema, __VA_ARGS__))
  MEMBER(std::vector<std::string>, joints, "joints to ignore");
  using JointValuesMap = std::map<std::string, double>;
  MEMBER(JointValuesMap, values, "values to set for ignored joints");
  using JointVelocitiesMap = std::map<std::string, double>;
  MEMBER(JointVelocitiesMap, velocities, "velocities to set for ignored joints");
#undef MEMBER
};

struct UDPRobotSchema
{
  MC_RTC_NEW_SCHEMA(UDPRobotSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(UDPRobotSchema, __VA_ARGS__))
  MEMBER(std::string, host, "UDP host")
  MEMBER(int, port, "UDP port")
  MEMBER(bool, singleClient, "Use a single client for sensors and control")
  MEMBER(bool, withEncoderVelocities, "Send/receive encoder velocities")
  MEMBER(UDPIgnoredJointsSchema, ignoredJoints, "joints to ignore");
#undef MEMBER
};

struct UDPRobotControl
{
  UDPRobotControl(mc_control::MCGlobalController & controller,
                  const std::string & robotName,
                  const UDPRobotSchema & udpRobotConfig);
  ~UDPRobotControl();

  void updateSensors();
  void updateControl();

protected:
  mc_control::MCGlobalController & controller_; //< Global controller used to control this robot
  UDPRobotSchema config_; //< UDP Configuration for this robot
  std::string robotName_;
  std::string robotModuleName_;
  std::string name_;

  bool controllerInit_ = false;

  std::map<size_t, double> ignoredJoints; //< Map of ignored joint indices to their values
  std::map<size_t, double> ignoredVelocities; //< Map of ignored joint indices to their velocities

  mc_udp::RobotSensors sensors_; //< Latest received sensors data
  std::atomic<bool> gotSensors_{false};
  std::mutex sensorsMutex_;
  mc_udp::RobotControl controlData_; //< Latest control data to send
  std::mutex controlMutex_;
  std::thread udpThread_;
  std::condition_variable udpInitCV_;
  std::mutex udpInitMutex_;
  std::atomic<bool> run_{true};

  std::mutex sendControlMutex_;
  std::atomic<bool> sendControl_{false};
  std::condition_variable sendControlCV_;
};
} // namespace mc_plugin
