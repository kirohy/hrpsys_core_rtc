#ifndef SEQUENCEPLAYER2_H
#define SEQUENCEPLAYER2_H

#include "SequencePlayer2Service_impl.h"
#include "seqplay.h"
#include <cnoid/Body>
#include <cnoid/EigenUtil>
#include <memory>
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
#include <semaphore.h>

#define RTC_INFO_STREAM(var) std::cout << "[" << m_profile.instance_name << "] " << var << std::endl;
#define RTC_WARN_STREAM(var)                                                                                           \
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << var << "\x1b[39m" << std::endl;

class SequencePlayer2 : public RTC::DataFlowComponentBase {
  public:
    SequencePlayer2(RTC::Manager *manager);
    virtual ~SequencePlayer2();

    virtual RTC::ReturnCode_t onInitialize();

    virtual RTC::ReturnCode_t onFinalize();

    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    double m_dt;
    std::shared_ptr<seqplay> player() { return m_seq; }
    cnoid::BodyPtr robot() { return m_robot; }
    void setClearFlag();
    void waitInterpolation();
    bool waitInterpolationOfGroup(const char *gname);
    bool setJointAngle(short id, double angle, double tm);
    bool setJointAngles(const double *angles, double tm);
    bool setJointAngles(const double *angles, const bool *mask, double tm);
    bool setJointAnglesSequence(const sequence_player::dSequenceSequence angless,
                                const sequence_player::bSequence &mask, const sequence_player::dSequence &times);
    bool setJointAnglesSequenceFull(
        const sequence_player::dSequenceSequence i_jvss, const sequence_player::dSequenceSequence i_vels,
        const sequence_player::dSequenceSequence i_torques, const sequence_player::dSequenceSequence i_poss,
        const sequence_player::dSequenceSequence i_rpys, const sequence_player::dSequenceSequence i_accs,
        const sequence_player::dSequenceSequence i_zmps, const sequence_player::dSequenceSequence i_wrenches,
        const sequence_player::dSequenceSequence i_optionals, const sequence_player::dSequence i_tms);
    bool clearJointAngles();
    bool setBasePos(const double *pos, double tm);
    bool setBaseRpy(const double *rpy, double tm);
    bool setZmp(const double *zmp, double tm);
    bool setTargetPose(const char *gname, const double *xyz, const double *rpy, double tm, const char *frame_name);
    bool setWrenches(const double *wrenches, double tm);
    void loadPattern(const char *basename, double time);
    void playPattern(const sequence_player::dSequenceSequence &pos, const sequence_player::dSequenceSequence &rpy,
                     const sequence_player::dSequenceSequence &zmp, const sequence_player::dSequence &tm);
    bool setInterpolationMode(sequence_player::SequencePlayer2Service::interpolationMode i_mode_);
    bool setInitialState(double tm = 0.0);
    bool addJointGroup(const char *gname, const sequence_player::SequencePlayer2Service::StrSequence &jnames);
    bool removeJointGroup(const char *gname);
    bool setJointAnglesOfGroup(const char *gname, const sequence_player::dSequence &jvs, double tm);
    bool setJointAnglesSequenceOfGroup(const char *gname, const sequence_player::dSequenceSequence angless,
                                       const sequence_player::dSequence &times);
    bool clearJointAnglesOfGroup(const char *gname);
    bool playPatternOfGroup(const char *gname, const sequence_player::dSequenceSequence &pos,
                            const sequence_player::dSequence &tm);

    void setMaxIKError(double pos, double rot);
    void setMaxIKIteration(short iter);

  protected:
    RTC::TimedDoubleSeq m_qInit;
    RTC::InPort<RTC::TimedDoubleSeq> m_qInitIn;
    RTC::TimedPoint3D m_basePosInit;
    RTC::InPort<RTC::TimedPoint3D> m_basePosInitIn;
    RTC::TimedOrientation3D m_baseRpyInit;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyInitIn;
    RTC::TimedPoint3D m_zmpRefInit;
    RTC::InPort<RTC::TimedPoint3D> m_zmpRefInitIn;

    RTC::TimedDoubleSeq m_qRef;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qRefOut;
    RTC::TimedDoubleSeq m_tqRef;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tqRefOut;
    RTC::TimedPoint3D m_zmpRef;
    RTC::OutPort<RTC::TimedPoint3D> m_zmpRefOut;
    RTC::TimedAcceleration3D m_accRef;
    RTC::OutPort<RTC::TimedAcceleration3D> m_accRefOut;
    RTC::TimedPoint3D m_basePos;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosOut;
    RTC::TimedOrientation3D m_baseRpy;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyOut;
    std::vector<RTC::TimedDoubleSeq> m_wrenches;
    std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedDoubleSeq>>> m_wrenchesOut;
    RTC::TimedDoubleSeq m_optionalData;
    RTC::OutPort<RTC::TimedDoubleSeq> m_optionalDataOut;

    RTC::CorbaPort m_SequencePlayer2ServicePort;

    SequencePlayer2Service_impl m_service0;

  private:
    std::shared_ptr<seqplay> m_seq;
    bool m_clearFlag, m_waitFlag;
    sem_t m_waitSem;
    cnoid::BodyPtr m_robot;
    std::string m_gname;
    unsigned int m_debugLevel;
    size_t m_optional_data_dim;
    std::mutex m_mutex;
    double m_error_pos, m_error_rot;
    short m_iteration;
    std::string m_fixedLink;
    cnoid::Vector3 m_offsetP, m_fixedP;
    cnoid::Matrix3 m_offsetR, m_fixedR;
    double m_timeToStartPlaying;
    int dummy;
};


extern "C" {
void SequencePlayer2Init(RTC::Manager *manager);
};

#endif // SEQUENCEPLAYER2_H
