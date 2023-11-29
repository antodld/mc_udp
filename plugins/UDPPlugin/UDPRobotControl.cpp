#include "UDPRobotControl.h"
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/clock.h>
#include <mc_rtc/io_utils.h>

namespace mc_plugin
{
using clock = mc_rtc::clock;
using duration_ms = mc_rtc::duration_ms;

UDPRobotControl::UDPRobotControl(mc_control::MCGlobalController & controller,
                                 const std::string & robotName,
                                 const UDPRobotSchema & udpRobotConfig)
: controller_(controller), config_(udpRobotConfig), robotName_(robotName), name_("UDPRobotControl::" + robotName)
{
  /**
   * Initialize ignored joint values and velocities
   * - from their configuration when provided
   * - from halfsitting otherwise
   */
  auto initIgnoredJoints = [this]() {
    const auto & ignoredJointsConfig = config_.ignoredJoints;
    const auto & joints = ignoredJointsConfig.joints;
    const auto & ignoredValues = ignoredJointsConfig.values;
    const auto & ignoredVelocityValues = ignoredJointsConfig.velocities;
    std::map<size_t, double> ignoredJoints;
    std::map<size_t, double> ignoredVelocities;
    for(const auto & jN : joints)
    {
      if(controller_.robot().hasJoint(jN))
      {
        const auto & rjo = controller_.robot().refJointOrder();
        const auto idx = std::distance(rjo.begin(), std::find(rjo.begin(), rjo.end(), jN));
        double qInit = 0;
        double alphaInit = 0;
        // Ignored joint values
        if(ignoredValues.count(jN) > 0)
        { // Use user-provided value for the ignored joint
          qInit = ignoredValues.at(jN);
        }
        else
        {
          // Use halfsitting configuration
          qInit = controller_.robot().stance().at(jN)[0];
        }
        ignoredJoints[idx] = qInit;

        // Ignored velocity values
        if(ignoredVelocityValues.count(jN) > 0)
        { // Use user-provided value for the ignored joint
          alphaInit = ignoredVelocityValues.at(jN);
        }
        ignoredVelocities[idx] = alphaInit;
        mc_rtc::log::warning("[UDP] Joint {} is ignored, value = {}, velocity= {}", jN, qInit, alphaInit);
      }
      else
      {
        mc_rtc::log::warning("[UDP] Ignored joint {} is not present in robot ", jN, controller_.robot().name());
      }
    }
  };
  initIgnoredJoints();

  udpThread_ = std::thread([this, robotName]() {
    const auto & host = config_.host;
    const auto & port = config_.port;
    const auto & singleClient = config_.singleClient;

    mc_udp::Client sensorsClient(host, port);
    mc_rtc::log::info("Connecting UDP sensors client to {}:{}", host, port);
    mc_udp::Client * controlClientPtr = &sensorsClient;
    if(!singleClient)
    {
      mc_rtc::log::info("Connecting UDP control client to {}:{}", host, port + 1);
      controlClientPtr = new mc_udp::Client(host, port + 1);
    }
    mc_udp::Client & controlClient = *controlClientPtr;

    std::vector<double> qIn;
    std::vector<double> alphaIn;
    while(run_)
    {
      if(sensorsClient.recv())
      {
        auto & sensorsMessages = sensorsClient.sensors().messages;
        if(!sensorsMessages.count(robotName))
        {
          mc_rtc::log::error("[{}] Server is providing sensors message for:", name_);
          for(const auto & m : sensorsClient.sensors().messages)
          {
            mc_rtc::log::error("- {}", m.first);
          }
          mc_rtc::log::error_and_throw<std::runtime_error>(
              "[{}] Server is not providing sensors message for robot ({})", name_, robotName);
        }
        const auto & sensors = sensorsClient.sensors().messages.at(robotName);
        {
          std::lock_guard<std::mutex> sensorsLock(sensorsMutex_);
          sensors_ = sensors;
          gotSensors_ = true;
        }
        if(!controllerInit_)
        {
          std::unique_lock<std::mutex> lock(udpInitMutex_);
          udpInitCV_.wait(lock, [this]() {
            if(controllerInit_)
            {
              mc_rtc::log::info("READY");
            }
            return controllerInit_;
          });
        }
        // Controller has now been initialized, we are ready to receive more data
        sensorsClient.init();
        if(!config_.singleClient)
        {
          controlClient.init();
        }
      }
    }
  });
}

UDPRobotControl::~UDPRobotControl()
{
  run_ = false;
  udpThread_.join();
}

void UDPRobotControl::updateSensors()
{
  auto sensors = mc_udp::RobotSensors{};
  if(gotSensors_)
  {
    std::lock_guard<std::mutex> sensorsLock(sensorsMutex_);
    sensors = sensors_;
    mc_rtc::log::info("[{}] Got encoders {}", name_, mc_rtc::io::to_string(sensors.encoders));
    gotSensors_ = false;
  }
  else
  {
    /* mc_rtc::log::warning("[{}] Did not get sensors", name_); */
    return;
  }
  Eigen::Vector3d rpy;
  rpy << sensors.orientation[0], sensors.orientation[1], sensors.orientation[2];
  Eigen::Vector3d pos;
  pos << sensors.position[0], sensors.position[1], sensors.position[2];
  Eigen::Vector3d vel;
  vel << sensors.angularVelocity[0], sensors.angularVelocity[1], sensors.angularVelocity[2];
  Eigen::Vector3d acc;
  acc << sensors.linearAcceleration[0], sensors.linearAcceleration[1], sensors.linearAcceleration[2];
  // XXX inefficient
  auto qIn = sensors.encoders;
  auto alphaIn = sensors.encoderVelocities;
  for(const auto & j : ignoredJoints)
  {
    qIn[j.first] = j.second;
  }
  if(config_.withEncoderVelocities)
  {
    for(const auto & j : ignoredVelocities)
    {
      alphaIn[j.first] = j.second;
    }
  }

  controller_.setEncoderValues(robotName_, sensors.encoders);
  controller_.setEncoderVelocities(robotName_, sensors.encoderVelocities);
  controller_.setJointTorques(robotName_, sensors.torques);
  controller_.setSensorOrientation(robotName_, Eigen::Quaterniond(mc_rbdyn::rpyToMat(rpy)));
  controller_.setSensorPosition(robotName_, pos);
  controller_.setSensorAngularVelocity(robotName_, vel);
  controller_.setSensorLinearAcceleration(robotName_, acc);

  if(!controllerInit_)
  {
    mc_rtc::log::info("Initializing controller");
    auto init_start = clock::now();
    controller_.init(qIn);
    controller_.running = true;
    auto init_end = clock::now();
    duration_ms init_dt = init_end - init_start;
    for(const auto & robot : controller_.controller().robots())
    {
      const auto & rjo = robot.module().ref_joint_order();
      if(rjo.size() == 0)
      {
        continue;
      }
      auto & cc = control_;
      auto & qOut = cc.encoders;
      auto & alphaOut = cc.encoderVelocities;
      if(qOut.size() != rjo.size())
      {
        qOut.resize(rjo.size());
      }
      if(config_.withEncoderVelocities && alphaOut.size() != rjo.size())
      {
        alphaOut.resize(rjo.size());
      }
    }
    mc_rtc::log::info("[MCUDPControl] Init duration {}", init_dt.count());
    controllerInit_ = true;
    udpInitCV_.notify_all();
  }
}

void UDPRobotControl::updateControl()
{
  // TODO update control
}

} // namespace mc_plugin
