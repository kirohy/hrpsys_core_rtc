#ifndef SEQPLAYSERVICESVC_IMPL_H
#define SEQPLAYSERVICESVC_IMPL_H

#include <sequence_player/idl/SequencePlayer2Service.hh>

using namespace OpenHRP;

class SequencePlayer2;

class SequencePlayer2Service_impl : public virtual POA_OpenHRP::SequencePlayer2Service, public virtual PortableServer::RefCountServantBase {
  public:
    SequencePlayer2Service_impl();
    virtual ~SequencePlayer2Service_impl();
    //
    void waitInterpolation();
    CORBA::Boolean waitInterpolationOfGroup(const char *gname);
    CORBA::Boolean setJointAnglesSequence(const dSequenceSequence &jvs, const dSequence &tms);
    CORBA::Boolean setJointAnglesSequenceWithMask(const dSequenceSequence &jvs, const bSequence &mask, const dSequence &tms);
    CORBA::Boolean setJointAnglesSequenceFull(const dSequenceSequence &jvss, const dSequenceSequence &vels, const dSequenceSequence &torques,
                                              const dSequenceSequence &poss, const dSequenceSequence &rpys, const dSequenceSequence &accs,
                                              const dSequenceSequence &zmps, const dSequenceSequence &wrenches, const dSequenceSequence &optionals,
                                              const dSequence &tms);
    CORBA::Boolean clearJointAngles();
    CORBA::Boolean setJointAngles(const dSequence &jvs, CORBA::Double tm);
    CORBA::Boolean setJointAnglesWithMask(const dSequence &jvs, const bSequence &mask, CORBA::Double tm);
    CORBA::Boolean setJointAngle(const char *jname, CORBA::Double jv, CORBA::Double tm);
    CORBA::Boolean setBasePos(const dSequence &pos, CORBA::Double tm);
    CORBA::Boolean setBaseRpy(const dSequence &rpy, CORBA::Double tm);
    CORBA::Boolean setZmp(const dSequence &zmp, CORBA::Double tm);
    CORBA::Boolean setWrenches(const dSequence &wrenches, CORBA::Double tm);
    CORBA::Boolean setTargetPose(const char *gname, const dSequence &xyz, const dSequence &rpy, CORBA::Double tm);
    CORBA::Boolean isEmpty();
    void loadPattern(const char *basename, CORBA::Double tm);
    void playPattern(const dSequenceSequence &pos, const dSequenceSequence &rpy, const dSequenceSequence &zmp, const dSequence &tm);
    void clear();
    void clearNoWait();
    CORBA::Boolean setInterpolationMode(OpenHRP::SequencePlayer2Service::interpolationMode i_mode_);
    CORBA::Boolean setInitialState();
    CORBA::Boolean addJointGroup(const char *gname, const OpenHRP::SequencePlayer2Service::StrSequence &jnames);
    CORBA::Boolean removeJointGroup(const char *gname);
    CORBA::Boolean setJointAnglesOfGroup(const char *gname, const dSequence &jvs, CORBA::Double tm);
    CORBA::Boolean setJointAnglesSequenceOfGroup(const char *gname, const dSequenceSequence &jvs, const dSequence &tms);
    CORBA::Boolean clearJointAnglesOfGroup(const char *gname);
    CORBA::Boolean clearOfGroup(const char *gname, CORBA::Double i_timelimit);
    CORBA::Boolean playPatternOfGroup(const char *gname, const dSequenceSequence &pos, const dSequence &tm);
    void setMaxIKError(CORBA::Double pos, CORBA::Double rot);
    void setMaxIKIteration(CORBA::Short iter);
    //
    void player(SequencePlayer2 *i_player);
    SequencePlayer2 *m_player;
};

#endif
