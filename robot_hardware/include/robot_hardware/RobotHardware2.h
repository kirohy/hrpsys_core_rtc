// -*- C++ -*-
/*!
 * @file  RobotHardware.h
 * @brief robot hardware component
 * @date  $Date$
 *
 * $Id$
 */

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

/**
   \brief RT component that do nothing and don't have ports. This component is used to create an execution context
 */
class RobotHardware2 : public RTC::DataFlowComponentBase {
  public:
    /**
       \brief Constructor
       \param manager pointer to the Manager
    */
    RobotHardware2(RTC::Manager *manager);
    /**
       \brief Destructor
    */
    virtual ~RobotHardware2();

    // The initialize action (on CREATED->ALIVE transition)
    // formaer rtc_init_entry()
    virtual RTC::ReturnCode_t onInitialize();

    // The finalize action (on ALIVE->END transition)
    // formaer rtc_exiting_entry()
    // virtual RTC::ReturnCode_t onFinalize();

    // The startup action when ExecutionContext startup
    // former rtc_starting_entry()
    // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

    // The shutdown action when ExecutionContext stop
    // former rtc_stopping_entry()
    // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

    // The activated action (Active state entry action)
    // former rtc_active_entry()
    // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

    // The deactivated action (Active state exit action)
    // former rtc_active_exit()
    // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

    // The execution action that is invoked periodically
    // former rtc_active_do()
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    // The aborting action when main logic error occurred.
    // former rtc_aborting_entry()
    // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

    // The error action in ERROR state
    // former rtc_error_do()
    // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

    // The reset action that is invoked resetting
    // This is same but different the former rtc_init_entry()
    // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

    // The state update action that is invoked after onExecute() action
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

    // The action that is invoked when execution context's rate is changed
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);

    virtual inline void getTimeNow(RTC::Time &tm) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        tm.sec  = ts.tv_sec;
        tm.nsec = ts.tv_nsec;
    };

    bool getProperty(const std::string &, std::string &);

  protected:
    // Configuration variable declaration
    // <rtc-template block="config_declare">
    int m_isDemoMode;

    // </rtc-template>

    // DataInPort declaration
    // <rtc-template block="inport_declare">

    /**
       \brief array of reference angles of joint with jointId
    */
    RTC::TimedDoubleSeq m_qRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
    /**
       \brief array of reference velocities of joint with jointId
    */
    RTC::TimedDoubleSeq m_dqRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqRefIn;
    /**
       \brief array of reference accelerations of joint with jointId
    */
    RTC::TimedDoubleSeq m_ddqRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_ddqRefIn;
    /**
       \brief array of reference torques of joint with jointId
    */
    RTC::TimedDoubleSeq m_tauRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn;

    // </rtc-template>

    /**
       \brief array of actual angles of joint with jointId
    */
    RTC::TimedDoubleSeq m_q;
    /**
       \brief array of actual velocities of joint with jointId
    */
    RTC::TimedDoubleSeq m_dq;
    /**
       \brief array of actual torques of joint with jointId
    */
    RTC::TimedDoubleSeq m_tau;
    /**
       \brief array of commanded torques of joint with jointId
    */
    RTC::TimedDoubleSeq m_ctau;
    /**
       \brief array of PD controller torques of joint with jointId
    */
    RTC::TimedDoubleSeq m_pdtau;
    /**
       \brief vector of actual acceleration (vector length = number of acceleration sensors)
    */
    std::vector<RTC::TimedAcceleration3D> m_acc;
    /**
       \brief vector of actual angular velocity (vector length = number of rate sensors)
    */
    std::vector<RTC::TimedAngularVelocity3D> m_rate;
    /**
       \brief vector of actual 6D wrench (vector length = number of F/T sensors)
              6D wrench vector = 3D force + 3D moment = fx, fy, fz, nx, ny, nz
    */
    std::vector<RTC::TimedDoubleSeq> m_force;
    OpenHRP::TimedLongSeqSeq m_servoState;
    RTC::TimedLong m_emergencySignal;
    OpenHRP::RobotHardware2Service::TimedRobotState2 m_rstate2;

    // DataOutPort declaration
    // <rtc-template block="outport_declare">
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

    // </rtc-template>

    // CORBA Port declaration
    // <rtc-template block="corbaport_declare">
    RTC::CorbaPort m_RobotHardware2ServicePort;

    // </rtc-template>

    // Service declaration
    // <rtc-template block="service_declare">
    RobotHardware2Service_impl m_service0;

    // </rtc-template>

    // Consumer declaration
    // <rtc-template block="consumer_declare">

    // </rtc-template>

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
