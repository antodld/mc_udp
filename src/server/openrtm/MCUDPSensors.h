#pragma once

#include <mc_udp/server/Server.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <chrono>
#include <memory>

class MCUDPSensors  : public RTC::DataFlowComponentBase
{
public:
  MCUDPSensors(RTC::Manager* manager);
  ~MCUDPSensors();

  virtual RTC::ReturnCode_t onInitialize();

  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
  // Configuration/Run
  double m_timeStep;
  bool m_enabled;
  int port;
  int timeout;

  bool was_enabled;

  // Inputs
  RTC::TimedDoubleSeq m_qIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_qInIn;
#ifdef MC_UDP_OPENRTM_LEGACY
  RTC::TimedDoubleSeq m_rpyIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_rpyInIn;
  RTC::TimedDoubleSeq m_rateIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_rateInIn;
  RTC::TimedDoubleSeq m_accIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_accInIn;
#else
  RTC::TimedOrientation3D m_rpyIn;
  RTC::InPort<RTC::TimedOrientation3D> m_rpyInIn;
  RTC::TimedAngularVelocity3D m_rateIn;
  RTC::InPort<RTC::TimedAngularVelocity3D> m_rateInIn;
  RTC::TimedAcceleration3D m_accIn;
  RTC::InPort<RTC::TimedAcceleration3D> m_accInIn;
#endif
  RTC::TimedDoubleSeq m_taucIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_taucInIn;
  RTC::TimedDoubleSeq rfsensor;
  RTC::InPort<RTC::TimedDoubleSeq> rfsensorIn;
  RTC::TimedDoubleSeq lfsensor;
  RTC::InPort<RTC::TimedDoubleSeq> lfsensorIn;
  RTC::TimedDoubleSeq rhsensor;
  RTC::InPort<RTC::TimedDoubleSeq> rhsensorIn;
  RTC::TimedDoubleSeq lhsensor;
  RTC::InPort<RTC::TimedDoubleSeq> lhsensorIn;
private:
  /* Measure execution time */
  std::chrono::time_point<std::chrono::system_clock> compute_start;
  std::chrono::time_point<std::chrono::system_clock> compute_end;
  std::chrono::duration<double> compute_time;
  /** Data server */
  mc_udp::Server server_;
};


extern "C"
{
  DLL_EXPORT void MCUDPSensorsInit(RTC::Manager* manager);
}
