#ifndef __DATA_LOGGER2_SERVICE_IMPL_H__
#define __DATA_LOGGER2_SERVICE_IMPL_H__

#include <data_logger/idl/DataLogger2Service.hh>

class DataLogger2;
class DataLogger2Service_impl : public virtual POA_data_logger::DataLogger2Service,
                                public virtual PortableServer::RefCountServantBase {
  public:
    DataLogger2Service_impl();
    virtual ~DataLogger2Service_impl();

    void setLogger(DataLogger2 *i_logger) { m_logger = i_logger; }

    CORBA::Boolean add(const char *type, const char *name);
    CORBA::Boolean save(const char *basename);
    CORBA::Boolean clear();
    void maxLength(CORBA::ULong len);

  private:
    DataLogger2 *m_logger;
};

#endif
