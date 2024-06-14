#ifndef ROBOT_HARDWARE2_H
#define ROBOT_HARDWARE2_H

#include <cnoid/Body>
#include <hrpsys/idl/HRPDataTypes.hh>
#include <memory>
#include <robot_hardware/idl/RobotHardware2Service.hh>
#include <rtm/CorbaPort.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/Manager.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <time.h>

#include "RobotHardware2Service_impl.h"

#define RTC_INFO_STREAM(var) std::cout << "[" << m_profile.instance_name << "] " << var << std::endl;
#define RTC_WARN_STREAM(var) std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << var << "\x1b[39m" << std::endl;

class robot;

class RobotHardware2 : public RTC::DataFlowComponentBase {
  public:
    RobotHardware2(RTC::Manager *manager);
    virtual ~RobotHardware2();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    virtual inline void getTimeNow(RTC::Time &tm) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        tm.sec  = ts.tv_sec;
        tm.nsec = ts.tv_nsec;
    };

  protected:
    int m_isDemoMode;

    RTC::TimedDoubleSeq m_qRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
    RTC::TimedDoubleSeq m_dqRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqRefIn;
    RTC::TimedDoubleSeq m_ddqRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_ddqRefIn;
    RTC::TimedDoubleSeq m_tauRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn;

    RTC::TimedDoubleSeq m_q;
    RTC::TimedDoubleSeq m_dq;
    RTC::TimedDoubleSeq m_tau;
    RTC::TimedDoubleSeq m_ctau;
    RTC::TimedDoubleSeq m_pdtau;
    std::vector<RTC::TimedAcceleration3D> m_acc;
    std::vector<RTC::TimedAngularVelocity3D> m_rate;
    std::vector<RTC::TimedDoubleSeq> m_force;
    OpenHRP::TimedLongSeqSeq m_servoState;
    RTC::TimedLong m_emergencySignal;
    OpenHRP::RobotHardware2Service::TimedRobotState2 m_rstate2;

    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_dqOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_ctauOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_pdtauOut;
    std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedAcceleration3D>>> m_accOut;
    std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedAngularVelocity3D>>> m_rateOut;
    std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedDoubleSeq>>> m_forceOut;
    RTC::OutPort<OpenHRP::TimedLongSeqSeq> m_servoStateOut;
    RTC::OutPort<RTC::TimedLong> m_emergencySignalOut;
    RTC::OutPort<OpenHRP::RobotHardware2Service::TimedRobotState2> m_rstate2Out;

    RTC::CorbaPort m_RobotHardware2ServicePort;
    RobotHardware2Service_impl m_service0;

    robot *robot_ptr(void) { return m_robot.get(); };

  private:
    void getStatus2(OpenHRP::RobotHardware2Service::RobotState2 &rstate2);

    int dummy;
    std::shared_ptr<robot> m_robot;
};


extern "C" {
void RobotHardware2Init(RTC::Manager *manager);
};

#endif // ROBOT_HARDWARE2_H
