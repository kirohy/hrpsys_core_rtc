#ifndef SEQPLAYSERVICESVC_IMPL_H
#define SEQPLAYSERVICESVC_IMPL_H

#include <sequence_player/idl/SequencePlayer2Service.hh>

class SequencePlayer2;

class SequencePlayer2Service_impl : public virtual POA_sequence_player::SequencePlayer2Service,
                                    public virtual PortableServer::RefCountServantBase {
  public:
    SequencePlayer2Service_impl();
    virtual ~SequencePlayer2Service_impl();
    //
    void waitInterpolation();
    CORBA::Boolean waitInterpolationOfGroup(const char *gname);
    CORBA::Boolean setJointAnglesSequence(const sequence_player::dSequenceSequence &jvs,
                                          const sequence_player::dSequence &tms);
    CORBA::Boolean setJointAnglesSequenceWithMask(const sequence_player::dSequenceSequence &jvs,
                                                  const sequence_player::bSequence &mask,
                                                  const sequence_player::dSequence &tms);
    CORBA::Boolean setJointAnglesSequenceFull(
        const sequence_player::dSequenceSequence &jvss, const sequence_player::dSequenceSequence &vels,
        const sequence_player::dSequenceSequence &torques, const sequence_player::dSequenceSequence &poss,
        const sequence_player::dSequenceSequence &rpys, const sequence_player::dSequenceSequence &accs,
        const sequence_player::dSequenceSequence &zmps, const sequence_player::dSequenceSequence &wrenches,
        const sequence_player::dSequenceSequence &optionals, const sequence_player::dSequence &tms);
    CORBA::Boolean clearJointAngles();
    CORBA::Boolean setJointAngles(const sequence_player::dSequence &jvs, CORBA::Double tm);
    CORBA::Boolean setJointAnglesWithMask(const sequence_player::dSequence &jvs, const sequence_player::bSequence &mask,
                                          CORBA::Double tm);
    CORBA::Boolean setJointAngle(const char *jname, CORBA::Double jv, CORBA::Double tm);
    CORBA::Boolean setBasePos(const sequence_player::dSequence &pos, CORBA::Double tm);
    CORBA::Boolean setBaseRpy(const sequence_player::dSequence &rpy, CORBA::Double tm);
    CORBA::Boolean setZmp(const sequence_player::dSequence &zmp, CORBA::Double tm);
    CORBA::Boolean setWrenches(const sequence_player::dSequence &wrenches, CORBA::Double tm);
    CORBA::Boolean setTargetPose(const char *gname, const sequence_player::dSequence &xyz,
                                 const sequence_player::dSequence &rpy, CORBA::Double tm);
    CORBA::Boolean isEmpty();
    void loadPattern(const char *basename, CORBA::Double tm);
    void playPattern(const sequence_player::dSequenceSequence &pos, const sequence_player::dSequenceSequence &rpy,
                     const sequence_player::dSequenceSequence &zmp, const sequence_player::dSequence &tm);
    void clear();
    void clearNoWait();
    CORBA::Boolean setInterpolationMode(sequence_player::SequencePlayer2Service::interpolationMode i_mode_);
    CORBA::Boolean setInitialState();
    CORBA::Boolean addJointGroup(const char *gname, const sequence_player::SequencePlayer2Service::StrSequence &jnames);
    CORBA::Boolean removeJointGroup(const char *gname);
    CORBA::Boolean setJointAnglesOfGroup(const char *gname, const sequence_player::dSequence &jvs, CORBA::Double tm);
    CORBA::Boolean setJointAnglesSequenceOfGroup(const char *gname, const sequence_player::dSequenceSequence &jvs,
                                                 const sequence_player::dSequence &tms);
    CORBA::Boolean clearJointAnglesOfGroup(const char *gname);
    CORBA::Boolean clearOfGroup(const char *gname, CORBA::Double i_timelimit);
    CORBA::Boolean playPatternOfGroup(const char *gname, const sequence_player::dSequenceSequence &pos,
                                      const sequence_player::dSequence &tm);
    void setMaxIKError(CORBA::Double pos, CORBA::Double rot);
    void setMaxIKIteration(CORBA::Short iter);
    //
    void player(SequencePlayer2 *i_player);
    SequencePlayer2 *m_player;
};

#endif
