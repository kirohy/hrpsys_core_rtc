#include <state_holder/StateHolder2.h>
#include <state_holder/TimeKeeper2Service_impl.h>

TimeKeeper2Service_impl::TimeKeeper2Service_impl() : m_comp(NULL) {}

TimeKeeper2Service_impl::~TimeKeeper2Service_impl() {}

void TimeKeeper2Service_impl::sleep(CORBA::Double tm) { m_comp->wait(tm); }
