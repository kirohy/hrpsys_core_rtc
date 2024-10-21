#ifndef DATA_LOGGER2_H
#define DATA_LOGGER2_H

#include <deque>
#include <iomanip>
#include <mutex>

#include <rtm/CorbaPort.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/Manager.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/ExtendedDataTypesSkel.h>

#include "DataLogger2Service_impl.h"

#define DEFAULT_MAX_LOG_LENGTH (200 * 20)

#define RTC_INFO_STREAM(var) std::cout << "[" << m_profile.instance_name << "] " << var << std::endl;
#define RTC_WARN_STREAM(var)                                                                                           \
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << var << "\x1b[39m" << std::endl;

class LoggerPortBase {
  public:
    LoggerPortBase() : m_maxLength(DEFAULT_MAX_LOG_LENGTH) {}
    virtual const char *name()                                         = 0;
    virtual void clear()                                               = 0;
    virtual void dumpLog(std::ostream &os, unsigned int precision = 0) = 0;
    virtual void log()                                                 = 0;
    void maxLength(unsigned int len) { m_maxLength = len; }

  protected:
    unsigned int m_maxLength;
};

class DataLogger2 : public RTC::DataFlowComponentBase {
  public:
    DataLogger2(RTC::Manager *manager);

    virtual ~DataLogger2();

    virtual RTC::ReturnCode_t onInitialize();

    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    bool add(const char *i_type, const char *i_name);
    bool save(const char *i_basename);
    bool clear();
    void suspendLogging();
    void resumeLogging();
    void maxLength(unsigned int len);

    std::vector<LoggerPortBase *> m_ports;

  protected:
    RTC::TimedLong m_emergencySignal;
    RTC::InPort<RTC::TimedLong> m_emergencySignalIn;

    RTC::CorbaPort m_DataLogger2ServicePort;

    DataLogger2Service_impl m_service0;

  private:
    bool m_suspendFlag;
    std::mutex m_suspendFlagMutex;
    unsigned int m_log_precision;
    int dummy;
};

extern "C" {
void DataLogger2Init(RTC::Manager *manager);
};

#endif // DATA_LOGGER2_H
