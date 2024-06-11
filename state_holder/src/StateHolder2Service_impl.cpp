#include <state_holder/StateHolder2.h>
#include <state_holder/StateHolder2Service_impl.h>

StateHolder2Service_impl::StateHolder2Service_impl() : m_comp(NULL) {}

StateHolder2Service_impl::~StateHolder2Service_impl() {}

void StateHolder2Service_impl::goActual() {
    if (!m_comp) return;

    m_comp->goActual();
}

void StateHolder2Service_impl::getCommand(OpenHRP::StateHolder2Service::Command_out com) {
    com = new OpenHRP::StateHolder2Service::Command;
    m_comp->getCommand(*com);
}
