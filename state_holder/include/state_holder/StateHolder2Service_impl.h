#ifndef STATEHOLDER2SERVICE_IMPL_H
#define STATEHOLDER2SERVICE_IMPL_H

#include <state_holder/idl/StateHolder2Service.hh>

class StateHolder2;

class StateHolder2Service_impl : public virtual POA_state_holder::StateHolder2Service,
                                 public virtual PortableServer::RefCountServantBase {
  public:
    StateHolder2Service_impl();
    virtual ~StateHolder2Service_impl();
    void setComponent(StateHolder2 *i_comp) { m_comp = i_comp; }
    void goActual();
    void getCommand(state_holder::StateHolder2Service::Command_out com);

  private:
    StateHolder2 *m_comp;
};

#endif
