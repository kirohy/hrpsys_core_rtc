#ifndef KALMANFILTER2_H
#define KALMANFILTER2_H

#include <cnoid/AccelerationSensor>
#include <cnoid/Body>
#include <rtm/CorbaPort.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/Manager.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/ExtendedDataTypesSkel.h>

#include "EKFilter.h"
#include "RPYKalmanFilter.h"

#include "KalmanFilter2Service_impl.h"

#define RTC_INFO_STREAM(var) std::cout << "[" << m_profile.instance_name << "] " << var << std::endl;
#define RTC_WARN_STREAM(var)                                                                                           \
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << var << "\x1b[39m" << std::endl;

class KalmanFilter2 : public RTC::DataFlowComponentBase {
  public:
    KalmanFilter2(RTC::Manager *manager);
    virtual ~KalmanFilter2();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    bool setKalmanFilterParam(const OpenHRP::KalmanFilter2Service::KalmanFilterParam &i_param);
    bool getKalmanFilterParam(OpenHRP::KalmanFilter2Service::KalmanFilterParam &i_param);
    bool resetKalmanFilterState();

  protected:
    RTC::TimedAngularVelocity3D m_rate;
    RTC::TimedAcceleration3D m_acc;
    RTC::TimedAcceleration3D m_accRef;
    RTC::TimedOrientation3D m_rpy;
    RTC::TimedOrientation3D m_rpyRaw;
    RTC::TimedOrientation3D m_rpy_prev;
    RTC::TimedOrientation3D m_rpyRaw_prev;

    RTC::InPort<RTC::TimedAngularVelocity3D> m_rateIn;
    RTC::InPort<RTC::TimedAcceleration3D> m_accIn;
    RTC::InPort<RTC::TimedAcceleration3D> m_accRefIn;
    RTC::InPort<RTC::TimedAngularVelocity3D> m_rpyIn; // for dummy usage

    RTC::OutPort<RTC::TimedOrientation3D> m_rpyOut;
    RTC::OutPort<RTC::TimedOrientation3D> m_rpyRawOut;
    RTC::TimedDoubleSeq m_qCurrent;
    RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
    RTC::TimedOrientation3D m_baseRpyCurrent;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyCurrentOut;

    RTC::CorbaPort m_KalmanFilterServicePort;
    KalmanFilter2Service_impl m_service0;

  private:
    double m_dt;
    RPYKalmanFilter rpy_kf;
    EKFilter ekf_filter;
    cnoid::BodyPtr m_robot;
    cnoid::AccelerationSensorPtr m_acc_sensor;
    cnoid::Matrix3 m_sensorR, sensorR_offset;
    cnoid::Vector3 acc_offset;
    unsigned int m_debugLevel;
    int dummy, loop;
    OpenHRP::KalmanFilter2Service::KFAlgorithm kf_algorithm;
};


extern "C" {
void KalmanFilter2Init(RTC::Manager *manager);
};

#endif // KALMANFILTER2_H
