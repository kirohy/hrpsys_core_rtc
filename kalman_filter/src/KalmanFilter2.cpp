#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/Link>
#include <kalman_filter/KalmanFilter2.h>
#include <math.h>
#include <rtm/CorbaNaming.h>

//#define USE_EKF

static const char *kalmanfilter2_spec[] = {"implementation_id",
                                           "KalmanFilter2",
                                           "type_name",
                                           "KalmanFilter2",
                                           "description",
                                           "kalman filter2",
                                           "version",
                                           "0.0.0",
                                           "vendor",
                                           "JSK",
                                           "category",
                                           "example",
                                           "activity_type",
                                           "DataFlowComponent",
                                           "max_instance",
                                           "10",
                                           "language",
                                           "C++",
                                           "lang_type",
                                           "compile",
                                           "conf.default.debugLevel",
                                           "0",
                                           ""};

KalmanFilter2::KalmanFilter2(RTC::Manager *manager)
    : RTC::DataFlowComponentBase(manager), m_rateIn("rate", m_rate), m_accIn("acc", m_acc), m_accRefIn("accRef", m_accRef), m_rpyIn("rpyIn", m_rate),
      m_qCurrentIn("qCurrent", m_qCurrent), m_rpyOut("rpy", m_rpy), m_rpyRawOut("rpy_raw", m_rpyRaw), m_baseRpyCurrentOut("baseRpyCurrent", m_baseRpyCurrent),
      m_KalmanFilterServicePort("KalmanFilter2Service"), m_robot(cnoid::BodyPtr()), m_debugLevel(0), dummy(0), loop(0) {
    m_service0.kalman(this);
}

KalmanFilter2::~KalmanFilter2() {}

#define DEBUGP ((m_debugLevel == 1 && loop % 200 == 0) || m_debugLevel > 1)
RTC::ReturnCode_t KalmanFilter2::onInitialize() {
    RTC_INFO_STREAM("onInitialize()");
    // Bind variables and configuration variable
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // Set InPort buffers
    addInPort("rate", m_rateIn);
    addInPort("acc", m_accIn);
    addInPort("accRef", m_accRefIn);
    addInPort("rpyIn", m_rpyIn);
    addInPort("qCurrent", m_qCurrentIn);

    // Set OutPort buffer
    addOutPort("rpy", m_rpyOut);
    addOutPort("rpy_raw", m_rpyRawOut);
    addOutPort("baseRpyCurrent", m_baseRpyCurrentOut);

    // Set service provider to Ports
    m_KalmanFilterServicePort.registerProvider("service0", "KalmanFilter2Service", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_KalmanFilterServicePort);

    // Setup robot model
    RTC::Properties &prop = getProperties();

    if (prop.hasKey("dt")) {
        m_dt = std::stod(std::string(prop["dt"]));
    } else {
        double rate = std::stod(std::string(this->m_pManager->getConfig()["exec_cxt.periodic.rate"]));
        if (rate > 0.0) {
            m_dt = 1.0 / rate;
        } else {
            RTC_WARN_STREAM("dt is invalid");
            return RTC::RTC_ERROR;
        }
    }
    RTC_INFO_STREAM("dt = " << m_dt);

    cnoid::BodyLoader body_loader;
    std::string body_filename;
    if (prop.hasKey("model")) {
        body_filename = std::string(prop["model"]);
    } else {
        body_filename = std::string(this->m_pManager->getConfig()["model"]);
    }
    if (body_filename.find("file://") == 0) { body_filename.erase(0, strlen("file://")); }
    m_robot = body_loader.load(body_filename);
    if (!m_robot) {
        RTC_WARN_STREAM("failed to load model [" << body_filename << "]");
        return RTC::RTC_ERROR;
    } else {
        RTC_INFO_STREAM("successed to load model [" << body_filename << "]");
    }

    m_rpy.data.r = 0;
    m_rpy.data.p = 0;
    m_rpy.data.y = 0;

    cnoid::DeviceList<cnoid::AccelerationSensor> acc_sensors(m_robot->devices());
    if (acc_sensors.size() > 0) {
        m_acc_sensor = acc_sensors[0];
        m_sensorR    = m_acc_sensor->link()->R() * m_acc_sensor->R_local();
    } else {
        m_sensorR = cnoid::Matrix3::Identity();
    }
    rpy_kf.setParam(m_dt, 0.001, 0.003, 1000, std::string(m_profile.instance_name));
    rpy_kf.setSensorR(m_sensorR);
    ekf_filter.setdt(m_dt);
    kf_algorithm = OpenHRP::KalmanFilter2Service::RPYKalmanFilter;
    m_qCurrent.data.length(m_robot->numJoints());
    acc_offset     = cnoid::Vector3::Zero();
    sensorR_offset = cnoid::Matrix3::Identity();

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t KalmanFilter::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t KalmanFilter2::onActivated(RTC::UniqueId ec_id) {
    RTC_INFO_STREAM("onActivated(" << ec_id << ")");
    return RTC::RTC_OK;
}

RTC::ReturnCode_t KalmanFilter2::onDeactivated(RTC::UniqueId ec_id) {
    RTC_INFO_STREAM("onDeactivated(" << ec_id << ")");
    return RTC::RTC_OK;
}

RTC::ReturnCode_t KalmanFilter2::onExecute(RTC::UniqueId ec_id) {
    loop++;
    static int initialize = 0;
    // RTC_INFO_STREAM("onExecute(" << ec_id << ") ");
    if (m_rpyIn.isNew()) {
        m_rpyIn.read();
        m_rpy.data.r = m_rate.data.avx;
        m_rpy.data.p = m_rate.data.avy;
        m_rpy.data.y = m_rate.data.avz;
        m_rpy.tm     = m_rate.tm;
        m_rpyOut.write();
        return RTC::RTC_OK;
    }
    if (m_rateIn.isNew()) { m_rateIn.read(); }
    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
        for (int i = 0; i < m_robot->numJoints(); i++) {
            m_robot->joint(i)->q() = m_qCurrent.data[i];
        }
    }
    double sx_ref = 0.0, sy_ref = 0.0, sz_ref = 0.0;
    if (m_accRefIn.isNew()) {
        m_accRefIn.read();
        sx_ref = m_accRef.data.ax, sy_ref = m_accRef.data.ay, sz_ref = m_accRef.data.az;
    }
    if (m_accIn.isNew()) {
        m_accIn.read();

        cnoid::Vector3 acc  = m_sensorR * cnoid::Vector3(m_acc.data.ax - sx_ref + acc_offset(0), m_acc.data.ay - sy_ref + acc_offset(1),
                                                        m_acc.data.az - sz_ref + acc_offset(2)); // transform to imaginary acc data
        acc                 = sensorR_offset * acc;
        cnoid::Vector3 gyro = m_sensorR * cnoid::Vector3(m_rate.data.avx, m_rate.data.avy, m_rate.data.avz); // transform to imaginary rate data
        gyro                = sensorR_offset * gyro;
        if (DEBUGP) {
            RTC_INFO_STREAM("raw data acc : " << std::endl << acc);
            RTC_INFO_STREAM("raw data gyro : " << std::endl << gyro);
        }
        cnoid::Vector3 rpy, rpyRaw, baseRpyCurrent;
        if (kf_algorithm == OpenHRP::KalmanFilter2Service::QuaternionExtendedKalmanFilter) {
            ekf_filter.main_one(rpy, rpyRaw, acc, gyro);
        } else if (kf_algorithm == OpenHRP::KalmanFilter2Service::RPYKalmanFilter) {
            double sl_y;
            cnoid::Matrix3 BtoS;
            m_robot->calcForwardKinematics();
            if (m_acc_sensor) {
                sl_y = cnoid::rpyFromRot(m_acc_sensor->link()->R())[2];
                BtoS = (m_robot->rootLink()->R()).transpose() * (m_acc_sensor->link()->R() * m_acc_sensor->R_local());
            } else {
                sl_y = 0.0;
                BtoS = (m_robot->rootLink()->R()).transpose();
            }
            rpy_kf.main_one(rpy, rpyRaw, baseRpyCurrent, acc, gyro, sl_y, BtoS);
        }
        m_rpyRaw.data.r         = rpyRaw(0);
        m_rpyRaw.data.p         = rpyRaw(1);
        m_rpyRaw.data.y         = rpyRaw(2);
        m_rpy.data.r            = rpy(0);
        m_rpy.data.p            = rpy(1);
        m_rpy.data.y            = rpy(2);
        m_baseRpyCurrent.data.r = baseRpyCurrent(0);
        m_baseRpyCurrent.data.p = baseRpyCurrent(1);
        m_baseRpyCurrent.data.y = baseRpyCurrent(2);
        // add time stamp
        m_rpyRaw.tm         = m_acc.tm;
        m_rpy.tm            = m_acc.tm;
        m_baseRpyCurrent.tm = m_acc.tm;

        m_rpyOut.write();
        m_rpyRawOut.write();
        m_baseRpyCurrentOut.write();
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t KalmanFilter::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

bool KalmanFilter2::setKalmanFilterParam(const OpenHRP::KalmanFilter2Service::KalmanFilterParam &i_param) {
    RTC_INFO_STREAM("setKalmanFilterParam");
    rpy_kf.setParam(m_dt, i_param.Q_angle, i_param.Q_rate, i_param.R_angle, std::string(m_profile.instance_name));
    kf_algorithm = i_param.kf_algorithm;
    for (size_t i = 0; i < 3; i++) {
        acc_offset(i) = i_param.acc_offset[i];
    }
    cnoid::Vector3 rpyoff;
    for (size_t i = 0; i < 3; i++) {
        rpyoff(i) = i_param.sensorRPY_offset[i];
    }
    sensorR_offset = cnoid::rotFromRpy(rpyoff);
    RTC_INFO_STREAM(
        "  kf_algorithm=" << (kf_algorithm == OpenHRP::KalmanFilter2Service::RPYKalmanFilter ? "RPYKalmanFilter" : "QuaternionExtendedKalmanFilter"));
    RTC_INFO_STREAM("  acc_offset = " << acc_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")));
    RTC_INFO_STREAM("  sensorRPY_offset = " << rpyoff.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")));
    return true;
}

bool KalmanFilter2::resetKalmanFilterState() {
    rpy_kf.resetKalmanFilterState();
    ekf_filter.resetKalmanFilterState();
    return true;
};

bool KalmanFilter2::getKalmanFilterParam(OpenHRP::KalmanFilter2Service::KalmanFilterParam &i_param) {
    i_param.Q_angle      = rpy_kf.getQangle();
    i_param.Q_rate       = rpy_kf.getQrate();
    i_param.R_angle      = rpy_kf.getRangle();
    i_param.kf_algorithm = kf_algorithm;
    for (size_t i = 0; i < 3; i++) {
        i_param.acc_offset[i] = acc_offset(i);
    }
    cnoid::Vector3 rpyoff = cnoid::rpyFromRot(sensorR_offset);
    for (size_t i = 0; i < 3; i++) {
        i_param.sensorRPY_offset[i] = rpyoff(i);
    }
    return true;
}

extern "C" {
void KalmanFilter2Init(RTC::Manager *manager) {
    RTC::Properties profile(kalmanfilter2_spec);
    manager->registerFactory(profile, RTC::Create<KalmanFilter2>, RTC::Delete<KalmanFilter2>);
}
};
