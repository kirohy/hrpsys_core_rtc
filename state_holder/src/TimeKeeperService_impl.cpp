#include <state_holder/StateHolder.h>
#include <state_holder/TimeKeeperService_impl.h>

TimeKeeperService_impl::TimeKeeperService_impl() : m_comp(NULL) {}

TimeKeeperService_impl::~TimeKeeperService_impl() {}

void TimeKeeperService_impl::sleep(CORBA::Double tm) { m_comp->wait(tm); }
