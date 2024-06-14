#include <iostream>
#include <kalman_filter/KalmanFilter2.h>
#include <kalman_filter/KalmanFilter2Service_impl.h>

KalmanFilter2Service_impl::KalmanFilter2Service_impl() {}

KalmanFilter2Service_impl::~KalmanFilter2Service_impl() {}

bool KalmanFilter2Service_impl::setKalmanFilterParam(const OpenHRP::KalmanFilter2Service::KalmanFilterParam &i_param) {
    return m_kalman->setKalmanFilterParam(i_param);
}

bool KalmanFilter2Service_impl::getKalmanFilterParam(OpenHRP::KalmanFilter2Service::KalmanFilterParam &i_param) {
    i_param = OpenHRP::KalmanFilter2Service::KalmanFilterParam();
    return m_kalman->getKalmanFilterParam(i_param);
}

bool KalmanFilter2Service_impl::resetKalmanFilterState() { return m_kalman->resetKalmanFilterState(); }

void KalmanFilter2Service_impl::kalman(KalmanFilter2 *i_kalman) { m_kalman = i_kalman; }
