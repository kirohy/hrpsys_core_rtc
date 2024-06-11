#ifndef TIMEKEEPER2SERVICE_IMPL_H
#define TIMEKEEPER2SERVICE_IMPL_H

#include <state_holder/idl/TimeKeeper2Service.hh>

using namespace OpenHRP;

class StateHolder2;

class TimeKeeper2Service_impl : public virtual POA_OpenHRP::TimeKeeper2Service, public virtual PortableServer::RefCountServantBase {
  public:
    TimeKeeper2Service_impl();
    virtual ~TimeKeeper2Service_impl();
    void setComponent(StateHolder2 *i_comp) { m_comp = i_comp; }
    void sleep(CORBA::Double tm);

  private:
    StateHolder2 *m_comp;
};

#endif
