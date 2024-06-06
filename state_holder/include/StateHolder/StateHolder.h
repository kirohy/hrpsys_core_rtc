#ifndef STATEHOLDER_H
#define STATEHOLDER_H

#include <rtm/CorbaPort.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/Manager.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <semaphore.h>

#include "StateHolderService_impl.h"
#include "TimeKeeperService_impl.h"

#define RTC_INFO_STREAM(var) std::cout << "[" << m_profile.instance_name << "] " << var << std::endl;
#define RTC_WARN_STREAM(var) std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << var << "\x1b[39m" << std::endl;

class StateHolder : public RTC::DataFlowComponentBase {
  public:
    StateHolder(RTC::Manager *manager);

    virtual ~StateHolder();

    virtual RTC::ReturnCode_t onInitialize();

    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    void goActual();
    void getCommand(StateHolderService::Command &com);
    bool getProperty(const std::string &, std::string &);
    void wait(CORBA::Double tm);

  protected:
    RTC::TimedDoubleSeq m_currentQ;
    RTC::InPort<RTC::TimedDoubleSeq> m_currentQIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_qIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_tqIn;
    RTC::InPort<RTC::TimedPoint3D> m_basePosIn;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;
    RTC::InPort<RTC::TimedPoint3D> m_zmpIn;
    std::vector<RTC::InPort<RTC::TimedDoubleSeq> *> m_wrenchesIn;
    RTC::TimedDoubleSeq m_optionalData;
    RTC::InPort<RTC::TimedDoubleSeq> m_optionalDataIn;

    RTC::TimedDoubleSeq m_q;
    RTC::TimedDoubleSeq m_tq;
    RTC::TimedPoint3D m_basePos;
    RTC::TimedOrientation3D m_baseRpy;
    RTC::TimedDoubleSeq m_baseTform;
    RTC::TimedPose3D m_basePose;
    RTC::TimedPoint3D m_zmp;
    std::vector<RTC::TimedDoubleSeq> m_wrenches;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tqOut;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosOut;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_baseTformOut;
    RTC::OutPort<RTC::TimedPose3D> m_basePoseOut;
    RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
    std::vector<RTC::OutPort<RTC::TimedDoubleSeq> *> m_wrenchesOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_optionalDataOut;

    RTC::CorbaPort m_StateHolderServicePort;
    RTC::CorbaPort m_TimeKeeperServicePort;

    StateHolderService_impl m_service0;
    TimeKeeperService_impl m_service1;

  private:
    int m_timeCount;
    sem_t m_waitSem, m_timeSem;
    bool m_requestGoActual;
    double m_dt;
    int dummy;
};


extern "C" {
void StateHolderInit(RTC::Manager *manager);
};

#endif // STATEHOLDER_H
