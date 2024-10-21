#ifndef REMOVEFORCESENSORLINKOFFSET2SERVICE_IMPL_H
#define REMOVEFORCESENSORLINKOFFSET2SERVICE_IMPL_H

#include "remove_force_sensor_link_offset/idl/RemoveForceSensorLinkOffset2Service.hh"

// using namespace OpenHRP;

class RemoveForceSensorLinkOffset2;

class RemoveForceSensorLinkOffset2Service_impl
    : public virtual POA_remove_force_sensor_link_offset::RemoveForceSensorLinkOffset2Service,
      public virtual PortableServer::RefCountServantBase {
  public:
    RemoveForceSensorLinkOffset2Service_impl();
    virtual ~RemoveForceSensorLinkOffset2Service_impl();
    //
    CORBA::Boolean setForceMomentOffsetParam(
        const char *i_name_,
        const remove_force_sensor_link_offset::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam &i_param_);
    CORBA::Boolean getForceMomentOffsetParam(
        const char *i_name_,
        remove_force_sensor_link_offset::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam_out i_param_);
    CORBA::Boolean loadForceMomentOffsetParams(const char *fiename);
    CORBA::Boolean dumpForceMomentOffsetParams(const char *fiename);
    CORBA::Boolean removeForceSensorOffset(
        const ::remove_force_sensor_link_offset::RemoveForceSensorLinkOffset2Service::StrSequence &names,
        CORBA::Double tm);
    //
    void rmfsoff(RemoveForceSensorLinkOffset2 *i_rmfsoff);

  private:
    RemoveForceSensorLinkOffset2 *m_rmfsoff;
};

#endif
