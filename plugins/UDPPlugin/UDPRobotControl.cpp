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
: controller_(controller), config_(udpRobotConfig), robotName_(robotName), name_("UDPRobotControl::" + robotName), robotModuleName_(controller.controller().robots().robot(robotName).module().name)
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
    auto & robot = controller_.robot(robotName_);
    for(const auto & jN : joints)
    {
      if(robot.hasJoint(jN))
      {
        const auto & rjo = robot.refJointOrder();
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
          qInit = robot.stance().at(jN)[0];
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
        mc_rtc::log::warning("[UDP] Ignored joint {} is not present in robot ", jN, robot.name());
      }
    }
  };
  initIgnoredJoints();

  udpThread_ = std::thread([this, robotName]() {
    const auto & host = config_.host;
    const auto & port = config_.port;
    const auto & singleClient = config_.singleClient;
    bool clientsInitialized = false;

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
    uint64_t prev_id = 0;
    while(run_)
    {
      // mc_rtc::log::info("Trying to receive sensors");
      if(sensorsClient.recv())
      {
        auto & sensorsMessages = sensorsClient.sensors().messages;
        if(!sensorsMessages.count(robotModuleName_))
        {
          mc_rtc::log::error("[{}] Server is providing sensors message for:", name_);
          for(const auto & m : sensorsMessages)
          {
            mc_rtc::log::error("- {}", m.first);
          }
          mc_rtc::log::error_and_throw<std::runtime_error>(
              "[{}] Server is not providing sensors message for robot ({})", name_, robotName);
        }
        const auto & sensors = sensorsMessages.at(robotName);
        {
          std::lock_guard<std::mutex> sensorsLock(sensorsMutex_);
          sensors_ = sensors;
          gotSensors_ = true;
        }
        if(!clientsInitialized)
        {
          mc_rtc::log::info("[{}] Waiting for controller initialization", name_);
          std::unique_lock<std::mutex> lock(udpInitMutex_);
          udpInitCV_.wait(lock, [this, &sensorsClient, &controlClient]() {
            if(controllerInit_)
            {
              // Controller has now been initialized, we are ready to receive more data
              sensorsClient.init();
              if(!config_.singleClient)
              {
                controlClient.init();
              }
              mc_rtc::log::info("[{}] Clients initialized, start receiving data", name_);
            }
            return controllerInit_;
          });
          clientsInitialized = true;
        }
      }
      if(!controllerInit_)
      {
        continue;
      }
      if(prev_id + 1 != sensors_.id)
      {
        mc_rtc::log::warning("[MCUDPControl] Missed one or more sensors reading (previous id: {}, current id: {})",
                             prev_id, sensors_.id);
      }
      std::unique_lock<std::mutex> lock(sendControlMutex_);
      sendControlCV_.wait(lock, [this]() -> bool { return sendControl_; });
      {
        std::lock_guard<std::mutex> controlLock(controlMutex_);
        controlData_.id = sensors_.id;
        controlClient.control().messages[robotModuleName_] = controlData_;
      }
      controlClient.send();
      prev_id = sensors_.id;
      sendControl_ = false;
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
  if(!gotSensors_) return;
  gotSensors_ = false;

  auto & ctl = controller_.controller();
  auto & robot = ctl.robot(robotName_);

  auto sensors = mc_udp::RobotSensors();
  {
    std::lock_guard<std::mutex> sensorsLock(sensorsMutex_);
    sensors = sensors_;
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

  // Floating base sensor
  if(robot.hasBodySensor("FloatingBase"))
  {
    controller_.setSensorPositions(
        robot.name(),
        {{"FloatingBase", {sensors.floatingBasePos[0], sensors.floatingBasePos[1], sensors.floatingBasePos[2]}}});
    Eigen::Vector3d fbRPY;
    controller_.setSensorOrientations(
        robot.name(),
        {{"FloatingBase", Eigen::Quaterniond(mc_rbdyn::rpyToMat(
                              {sensors.floatingBaseRPY[0], sensors.floatingBaseRPY[1], sensors.floatingBaseRPY[2]}))}});
    controller_.setSensorAngularVelocities(
        robot.name(),
        {{"FloatingBase", {sensors.floatingBaseVel[0], sensors.floatingBaseVel[1], sensors.floatingBaseVel[2]}}});
    controller_.setSensorLinearVelocities(
        robot.name(),
        {{"FloatingBase", {sensors.floatingBaseVel[3], sensors.floatingBaseVel[4], sensors.floatingBaseVel[5]}}});
    controller_.setSensorLinearAccelerations(
        robot.name(),
        {{"FloatingBase", {sensors.floatingBaseAcc[0], sensors.floatingBaseAcc[1], sensors.floatingBaseAcc[2]}}});
  }
  std::unordered_map<std::string, std::string> fsensors;
  fsensors["rfsensor"] = "RightFootForceSensor";
  fsensors["lfsensor"] = "LeftFootForceSensor";
  fsensors["rhsensor"] = "RightHandForceSensor";
  fsensors["lhsensor"] = "LeftHandForceSensor";
  std::map<std::string, std::map<std::string, sva::ForceVecd>> robot_wrenches;
  auto & wrenches = robot_wrenches[robotName_];
  for(const auto & fs : sensors.fsensors)
  {
    Eigen::Vector6d reading;
    reading << fs.reading[3], fs.reading[4], fs.reading[5], fs.reading[0], fs.reading[1], fs.reading[2];
    wrenches[fsensors.at(fs.name)] = sva::ForceVecd(reading);
  }
  controller_.setWrenches(robot.name(), wrenches);

  if(!controllerInit_)
  {
    mc_rtc::log::info("[{}] Initializing controller", name_);
    auto & robot = ctl.robot(robotName_);
    auto & q = robot.mbc().q;
    for(size_t i = 0; i < qIn.size(); i++)
    {
      auto jIdx = robot.jointIndexInMBC(i);
      if(jIdx != -1 && q[jIdx].size() == 1)
      {
        q[jIdx][0] = qIn[i];
      }
    }
    mc_rtc::log::warning("[{}] Resetting robot to initial position {}", name_, mc_rtc::io::to_string(qIn));
    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();
    /**
     * As we have no guarantee that the robot is available before the controller has already been initialized, we
     * provide a way to call a user-defined reset function that is called now that the robot is fully initialized.
     * The user is expected to handle resetting the tasks according to the current robot state.
     *
     * Note that in case the robot starts in its halfsitting stance, this is not needed as the controller's robot state
     * and the real robot match.
     */
    if(ctl.datastore().has("UDPPlugin::" + robotName_ + "::reset"))
    {
      ctl.datastore().call<void>("UDPPlugin::" + robotName_ + "::reset");
    }
    const auto & rjo = robot.module().ref_joint_order();
    auto & cc = controlData_; // no need for lock here as we won't be trying to send control data yet
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
    // mc_rtc::log::info("[MCUDPControl] Init duration {}", init_dt.count());
    controllerInit_ = true;
    udpInitCV_.notify_all();
  }
}

void UDPRobotControl::updateControl()
{
  if(!controllerInit_) return;
  // TODO update control
  //
  // if(prev_id + 1 != sc.id)
  // {
  //   mc_rtc::log::warning("[MCUDPControl] Missed one or more sensors reading (previous id: {}, current id: {})",
  //                        prev_id, sc.id);
  // }
  auto & ctl = controller_.controller();
  auto & robot = ctl.outputRobot(robotName_);
  const auto & rjo = robot.module().ref_joint_order();
  const auto & mbc = robot.mbc();
  std::lock_guard<std::mutex> lock(controlMutex_);
  auto & cc = controlData_;
  auto & qOut = cc.encoders;
  auto & alphaOut = cc.encoderVelocities;
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    const auto & jN = rjo[i];
    if(robot.hasJoint(jN))
    {
      auto jIndex = robot.jointIndexByName(jN);
      if(mbc.q[jIndex].size() == 1)
      {
        auto jIdx = robot.jointIndexByName(jN);
        qOut[i] = mbc.q[jIdx][0];
        if(config_.withEncoderVelocities)
        {
          alphaOut[i] = mbc.alpha[jIdx][0];
        }
      }
    }
  }

  // Ignore QP output for ignored joints
  for(const auto & j : ignoredJoints)
  {
    qOut[j.first] = j.second;
  }
  if(config_.withEncoderVelocities)
  {
    for(const auto & j : ignoredVelocities)
    {
      alphaOut[j.first] = j.second;
    }
  }
  // cc.id = sc.id;
  sendControl_ = true;
  sendControlCV_.notify_all();
}

} // namespace mc_plugin
