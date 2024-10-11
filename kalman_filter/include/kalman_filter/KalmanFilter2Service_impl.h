#ifndef __KALMANFILTER2_SERVICE_H__
#define __KALMANFILTER2_SERVICE_H__

#include <kalman_filter/idl/KalmanFilter2Service.hh>

class KalmanFilter2;

class KalmanFilter2Service_impl : public virtual POA_OpenHRP::KalmanFilter2Service,
                                  public virtual PortableServer::RefCountServantBase {
  public:
    KalmanFilter2Service_impl();
    virtual ~KalmanFilter2Service_impl();

    bool setKalmanFilterParam(const OpenHRP::KalmanFilter2Service::KalmanFilterParam &i_param);
    bool getKalmanFilterParam(OpenHRP::KalmanFilter2Service::KalmanFilterParam &i_param);
    bool resetKalmanFilterState();

    void kalman(KalmanFilter2 *i_kalman);

  private:
    KalmanFilter2 *m_kalman;
};

#endif
