#include <data_logger/DataLogger2.h>
#include <data_logger/DataLogger2Service_impl.h>

DataLogger2Service_impl::DataLogger2Service_impl() : m_logger(NULL) {}

DataLogger2Service_impl::~DataLogger2Service_impl() {}

CORBA::Boolean DataLogger2Service_impl::add(const char *type, const char *name) { return m_logger->add(type, name); }

CORBA::Boolean DataLogger2Service_impl::save(const char *basename) { return m_logger->save(basename); }

CORBA::Boolean DataLogger2Service_impl::clear() { return m_logger->clear(); }

void DataLogger2Service_impl::maxLength(CORBA::ULong len) { m_logger->maxLength(len); }
