// -*- C++ -*-
/*!
 * @file  MCNNGControl.cpp * @brief Core component for MC control * $Date$
 *
 * $Id$
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wdelete-incomplete"
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#include "MCNNGControl.h"

#include <mc_nng/logging.h>

#include <fstream>
#include <iomanip>

// Module specification
// <rtc-template block="module_spec">
static const char* mccontrol_spec[] =
  {
    "implementation_id", "MCNNGControl",
    "type_name",         "MCNNGControl",
    "description",       "Core component for MC control",
    "version",           "0.1",
    "vendor",            "CNRS",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.timeStep", "0.005",
    "conf.default.is_enabled", "0",
    "conf.default.uri", "tcp://*:4444",
    "conf.default.timeout", "5",
    ""
  };
// </rtc-template>

MCNNGControl::MCNNGControl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_timeStep(0.005),
    m_enabled(false),
    uri("tcp://*:4444"),
    timeout(5),
    was_enabled(false),
    m_qInIn("qIn", m_qIn),
    m_rpyInIn("rpyIn", m_rpyIn),
    m_rateInIn("rateIn", m_rateIn),
    m_accInIn("accIn", m_accIn),
    m_taucInIn("taucIn", m_taucIn),
    rfsensorIn("rfsensor", rfsensor),
    lfsensorIn("lfsensor", lfsensor),
    rhsensorIn("rhsensor", rhsensor),
    lhsensorIn("lhsensor", lhsensor),
    m_qOutOut("qOut", m_qOut),
    server_(),
    got_control_(false),
    control_lost_(false),
    control_lost_iter_(0)
    // </rtc-template>
{
}

MCNNGControl::~MCNNGControl()
{
}


RTC::ReturnCode_t MCNNGControl::onInitialize()
{
  MC_NNG_INFO("MCNNGControl::onInitialize() starting")
  // Set InPort buffers
  addInPort("qIn", m_qInIn);
  addInPort("rpyIn", m_rpyInIn);
  addInPort("rateIn", m_rateInIn);
  addInPort("accIn", m_accInIn);
  addInPort("taucIn", m_taucInIn);
  addInPort("rfsensor", rfsensorIn);
  addInPort("lfsensor", lfsensorIn);
  addInPort("rhsensor", rhsensorIn);
  addInPort("lhsensor", lhsensorIn);

  // Set OutPort buffer
  addOutPort("qOut", m_qOutOut);

  // Bind variables and configuration variable
  bindParameter("timeStep", m_timeStep, "0.005");
  bindParameter("is_enabled", m_enabled, "0");
  bindParameter("uri", uri, "tcp://*:4444");
  bindParameter("timeout", timeout, "5");

  MC_NNG_INFO("MCNNGControl::onInitialize() finished")
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCNNGControl::onActivated(RTC::UniqueId ec_id)
{
  MC_NNG_INFO("MCNNGControl::onActivated")
  server_.restart(uri, timeout);
  MC_NNG_SUCCESS("MCNNGControl started on " << uri << " (timeout: " << timeout << ")")
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MCNNGControl::onDeactivated(RTC::UniqueId ec_id)
{
  MC_NNG_INFO("MCNNGControl::onDeactivated")
  m_enabled = false;
  server_.stop();
  server_.sensors().id += 1;
  return RTC::RTC_OK;
}

namespace
{

void read_fsensor(const std::string & name, RTC::InPort<RTC::TimedDoubleSeq> & port, RTC::TimedDoubleSeq & data, mc_nng::Server & server_)
{
  if(port.isNew())
  {
    port.read();
    if(data.data.length() == 6)
    {
      server_.sensors().fsensor(name, data.data.NP_data());
    }
  }
}

}

RTC::ReturnCode_t MCNNGControl::onExecute(RTC::UniqueId ec_id)
{
  read_fsensor("rfsensor", rfsensorIn, rfsensor, server_);
  read_fsensor("lfsensor", lfsensorIn, lfsensor, server_);
  read_fsensor("rhsensor", rhsensorIn, rhsensor, server_);
  read_fsensor("lhsensor", lhsensorIn, lhsensor, server_);
  if(m_rpyInIn.isNew())
  {
    m_rpyInIn.read();
    server_.sensors().orientation[0] = m_rpyIn.data.r;
    server_.sensors().orientation[1] = m_rpyIn.data.p;
    server_.sensors().orientation[2] = m_rpyIn.data.y;
  }
  if(m_rateInIn.isNew())
  {
    m_rateInIn.read();
    server_.sensors().angularVelocity[0] = m_rateIn.data.avx;
    server_.sensors().angularVelocity[1] = m_rateIn.data.avy;
    server_.sensors().angularVelocity[2] = m_rateIn.data.avz;
  }
  if(m_accInIn.isNew())
  {
    m_accInIn.read();
    server_.sensors().angularAcceleration[0] = m_accIn.data.ax;
    server_.sensors().angularAcceleration[1] = m_accIn.data.ay;
    server_.sensors().angularAcceleration[2] = m_accIn.data.az;
  }
  if(m_taucInIn.isNew())
  {
    m_taucInIn.read();
    if(server_.sensors().torques.size() != m_taucIn.data.length())
    {
      server_.sensors().torques.resize(m_taucIn.data.length());
    }
    for(unsigned int i = 0; i < static_cast<unsigned int>(m_taucIn.data.length()); ++i)
    {
      server_.sensors().torques[i] = m_taucIn.data[i];
    }
  }
  if(m_qInIn.isNew())
  {
    m_qInIn.read();
    m_qOut.data.length(m_qIn.data.length());
    if(server_.sensors().encoders.size() != m_qIn.data.length())
    {
      server_.sensors().encoders.resize(m_qIn.data.length());
    }
    for(unsigned int i = 0; i < m_qIn.data.length(); ++i)
    {
      server_.sensors().encoders[i] = m_qIn.data[i];
    }
    coil::TimeValue coiltm(coil::gettimeofday());
    RTC::Time tm;
    tm.sec  = static_cast<CORBA::ULong>(coiltm.sec());
    tm.nsec = static_cast<CORBA::ULong>(coiltm.usec())*1000;
    if(m_enabled)
    {
      compute_start = std::chrono::system_clock::now();
      server_.send();
      if(server_.recv())
      {
        if(server_.control().encoders.size() != m_qIn.data.length())
        {
          MC_NNG_WARNING("[MCNNGControl] Command provided by control has the wrong size (expected: " << m_qIn.data.length() << ", got: " << server_.control().encoders.size() << ")")
          return RTC::RTC_OK;
        }
        got_control_ = true;
        if(control_lost_)
        {
          MC_NNG_WARNING("Control was lost for " << control_lost_iter_ << " iteration(s)")
          control_lost_iter_ = 0;
        }
        control_lost_ = false;
        for(unsigned int i = 0; i < m_qOut.data.length(); ++i)
        {
          m_qOut.data[i] = server_.control().encoders[i];
        }
        if(server_.control().id != server_.sensors().id)
        {
          MC_NNG_WARNING("[MCNNGControl] Using control from iteration " << server_.control().id << " when last sensors sent is from iteration " << server_.sensors().id)
          if(server_.recv())
          {
            for(unsigned int i = 0; i < m_qOut.data.length(); ++i)
            {
              m_qOut.data[i] = server_.control().encoders[i];
            }
          }
        }
      }
      else
      {
        if(got_control_ && !control_lost_)
        {
          MC_NNG_ERROR("[MCNNGControl] Didn't receive an answer before timeout, writing previous qOut as control")
          control_lost_ = true;
          got_control_ = false;
        }
        if(control_lost_)
        {
          control_lost_iter_++;
        }
      }
      compute_end = std::chrono::system_clock::now();
      compute_time = compute_end - compute_start;
      double elapsed = compute_time.count() * 1000;
      if(elapsed > 5.1)
      {
        MC_NNG_WARNING("Total time spent in MCNNGControl::onExecute (" << elapsed << ") exceeded 5.1ms")
      }
      m_qOut.tm = tm;
      m_qOutOut.write();
      server_.sensors().id += 1;
    }
    else
    {
      m_qOut = m_qIn;
    }
  }
  return RTC::RTC_OK;
}

extern "C"
{

  void MCNNGControlInit(RTC::Manager* manager)
  {
    coil::Properties profile(mccontrol_spec);
    manager->registerFactory(profile,
                             RTC::Create<MCNNGControl>,
                             RTC::Delete<MCNNGControl>);
  }

};


#pragma GCC diagnostic pop

