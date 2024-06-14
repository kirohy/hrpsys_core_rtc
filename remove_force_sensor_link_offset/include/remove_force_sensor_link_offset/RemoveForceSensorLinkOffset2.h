#ifndef REMOVEFORCESENSORLINKOFFSET2_H
#define REMOVEFORCESENSORLINKOFFSET2_H

#include <rtm/CorbaPort.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/Manager.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <cnoid/Body>
#include <cnoid/EigenUtil>
#include <cnoid/JointPath>
#include <cnoid/Link>

#include <memory>
#include <mutex>
#include <semaphore.h>

#include "RemoveForceSensorLinkOffset2Service_impl.h"

#define RTC_INFO_STREAM(var) std::cout << "[" << m_profile.instance_name << "] " << var << std::endl;
#define RTC_WARN_STREAM(var) std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << var << "\x1b[39m" << std::endl;

class RemoveForceSensorLinkOffset2 : public RTC::DataFlowComponentBase {
  public:
    RemoveForceSensorLinkOffset2(RTC::Manager *manager);

    virtual ~RemoveForceSensorLinkOffset2();

    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    bool setForceMomentOffsetParam(const std::string &i_name_, const OpenHRP::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam &i_param_);
    bool getForceMomentOffsetParam(const std::string &i_name_, OpenHRP::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam &i_param_);
    bool loadForceMomentOffsetParams(const std::string &filename);
    bool dumpForceMomentOffsetParams(const std::string &filename);
    bool removeForceSensorOffset(const ::OpenHRP::RemoveForceSensorLinkOffset2Service::StrSequence &names, const double tm);

  protected:
    RTC::TimedDoubleSeq m_qCurrent;
    RTC::TimedOrientation3D m_rpy;

    RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
    RTC::InPort<RTC::TimedOrientation3D> m_rpyIn;

    std::vector<RTC::TimedDoubleSeq> m_force;
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq>>> m_forceIn;
    std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedDoubleSeq>>> m_forceOut;

    RTC::CorbaPort m_RemoveForceSensorLinkOffsetServicePort;

    RemoveForceSensorLinkOffset2Service_impl m_service0;

  private:
    struct ForceMomentOffsetParam {
        cnoid::Vector3 force_offset, moment_offset, link_offset_centroid;
        cnoid::Vector3 off_force, off_moment;
        double link_offset_mass;
        cnoid::Vector3 force_offset_sum, moment_offset_sum;
        int sensor_offset_calib_counter;
        sem_t wait_sem;

        ForceMomentOffsetParam()
            : force_offset(cnoid::Vector3::Zero()), moment_offset(cnoid::Vector3::Zero()), off_force(cnoid::Vector3::Zero()),
              off_moment(cnoid::Vector3::Zero()), link_offset_centroid(cnoid::Vector3::Zero()), link_offset_mass(0), force_offset_sum(cnoid::Vector3::Zero()),
              moment_offset_sum(cnoid::Vector3::Zero()), sensor_offset_calib_counter(0), wait_sem() {
            sem_init(&wait_sem, 0, 0);
        };
    };
    void updateRootLinkPosRot(const cnoid::Vector3 &rpy);
    void printForceMomentOffsetParam(const std::string &i_name_);

    std::map<std::string, ForceMomentOffsetParam> m_forcemoment_offset_param;

    static constexpr double grav = 9.80665;
    double m_dt;
    cnoid::BodyPtr m_robot;
    unsigned int m_debugLevel;
    int max_sensor_offset_calib_counter;
    std::mutex m_mutex;
};

extern "C" {
void RemoveForceSensorLinkOffset2Init(RTC::Manager *manager);
};

#endif // REMOVEFORCESENSORLINKOFFSET2_H
