#include <iostream>
#include <remove_force_sensor_link_offset/RemoveForceSensorLinkOffset2.h>
#include <remove_force_sensor_link_offset/RemoveForceSensorLinkOffset2Service_impl.h>

RemoveForceSensorLinkOffset2Service_impl::RemoveForceSensorLinkOffset2Service_impl() : m_rmfsoff(NULL) {}

RemoveForceSensorLinkOffset2Service_impl::~RemoveForceSensorLinkOffset2Service_impl() {}

CORBA::Boolean
RemoveForceSensorLinkOffset2Service_impl::setForceMomentOffsetParam(const char *i_name_,
                                                                    const OpenHRP::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam &i_param_) {
    return m_rmfsoff->setForceMomentOffsetParam(std::string(i_name_), i_param_);
}

CORBA::Boolean
RemoveForceSensorLinkOffset2Service_impl::getForceMomentOffsetParam(const char *i_name_,
                                                                    OpenHRP::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam_out i_param_) {
    i_param_ = new OpenHRP::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam();
    i_param_->force_offset.length(3);
    i_param_->moment_offset.length(3);
    i_param_->link_offset_centroid.length(3);
    return m_rmfsoff->getForceMomentOffsetParam(std::string(i_name_), *i_param_);
}

CORBA::Boolean RemoveForceSensorLinkOffset2Service_impl::loadForceMomentOffsetParams(const char *filename) {
    return m_rmfsoff->loadForceMomentOffsetParams(std::string(filename));
};

CORBA::Boolean RemoveForceSensorLinkOffset2Service_impl::dumpForceMomentOffsetParams(const char *filename) {
    return m_rmfsoff->dumpForceMomentOffsetParams(std::string(filename));
};

CORBA::Boolean RemoveForceSensorLinkOffset2Service_impl::removeForceSensorOffset(const ::OpenHRP::RemoveForceSensorLinkOffset2Service::StrSequence &names,
                                                                                 CORBA::Double tm) {
    return m_rmfsoff->removeForceSensorOffset(names, tm);
}

void RemoveForceSensorLinkOffset2Service_impl::rmfsoff(RemoveForceSensorLinkOffset2 *i_rmfsoff) { m_rmfsoff = i_rmfsoff; }
