#include <remove_force_sensor_link_offset/RemoveForceSensorLinkOffset2.h>
#include <rtm/CorbaNaming.h>
#include <cnoid/AccelerationSensor>
#include <cnoid/BodyLoader>
#include <cnoid/DeviceList>
#include <cnoid/ForceSensor>

static const char *removeforcesensorlinkoffset2_spec[] = {"implementation_id", "RemoveForceSensorLinkOffset2", "type_name", "RemoveForceSensorLinkOffset2",
                                                          "description", "null component", "version", "0.0.0", "vendor", "JSK", "category", "example",
                                                          "activity_type", "DataFlowComponent", "max_instance", "10", "language", "C++", "lang_type", "compile",
                                                          // Configuration variables
                                                          "conf.default.debugLevel", "0", ""};

RemoveForceSensorLinkOffset2::RemoveForceSensorLinkOffset2(RTC::Manager *manager)
    : RTC::DataFlowComponentBase(manager),
      m_qCurrentIn("qCurrent", m_qCurrent), m_rpyIn("rpy", m_rpy), m_RemoveForceSensorLinkOffsetServicePort("RemoveForceSensorLinkOffset2Service"),
      m_debugLevel(0), max_sensor_offset_calib_counter(0) {
    m_service0.rmfsoff(this);
}

RemoveForceSensorLinkOffset2::~RemoveForceSensorLinkOffset2() {}


RTC::ReturnCode_t RemoveForceSensorLinkOffset2::onInitialize() {
    RTC_INFO_STREAM("onInitialize()");

    // Bind variables and configuration variable
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // Set InPort buffers
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("rpy", m_rpyIn);

    // Set OutPort buffer

    // Set service provider to Ports
    m_RemoveForceSensorLinkOffsetServicePort.registerProvider("service0", "RemoveForceSensorLinkOffset2Service", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_RemoveForceSensorLinkOffsetServicePort);

    RTC::Properties &prop = this->getProperties();

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

    cnoid::DeviceList<cnoid::ForceSensor> force_sensors(m_robot->devices());
    unsigned int nforce = force_sensors.size();

    m_force.resize(nforce);
    m_forceOut.resize(nforce);
    m_forceIn.resize(nforce);
    for (unsigned int i = 0; i < nforce; i++) {
        cnoid::ForceSensorPtr s = force_sensors[i];
        m_forceOut[i]           = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq>>(std::string("off_" + s->name()).c_str(), m_force[i]);
        m_forceIn[i]            = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq>>(s->name().c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(s->name().c_str(), *m_forceIn[i]);
        registerOutPort(std::string("off_" + s->name()).c_str(), *m_forceOut[i]);
        m_forcemoment_offset_param.insert(std::pair<std::string, ForceMomentOffsetParam>(s->name(), ForceMomentOffsetParam()));
    }
    max_sensor_offset_calib_counter = static_cast<int>(8.0 / m_dt); // 8.0[s] by default
    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RemoveForceSensorLinkOffset2::onActivated(RTC::UniqueId ec_id) {
    RTC_INFO_STREAM("onActivated(" << ec_id << ")");
    return RTC::RTC_OK;
}

RTC::ReturnCode_t RemoveForceSensorLinkOffset2::onDeactivated(RTC::UniqueId ec_id) {
    RTC_INFO_STREAM("onDeactivated(" << ec_id << ")");
    return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel == 1 && loop % 200 == 0) || m_debugLevel > 1)
RTC::ReturnCode_t RemoveForceSensorLinkOffset2::onExecute(RTC::UniqueId ec_id) {
    // RTC_INFO_STREAM("onExecute(" << ec_id << ")");
    static int loop = 0;
    loop++;
    for (unsigned int i = 0; i < m_forceIn.size(); i++) {
        if (m_forceIn[i]->isNew()) { m_forceIn[i]->read(); }
    }
    cnoid::Vector3 rpy;
    if (m_rpyIn.isNew()) {
        m_rpyIn.read();
        rpy = cnoid::Vector3(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y);
    } else {
        rpy = cnoid::Vector3::Zero();
    }
    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
        for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
            m_robot->joint(i)->q() = m_qCurrent.data[i];
        }
        //
        this->updateRootLinkPosRot(rpy);
        m_robot->calcForwardKinematics();
        std::lock_guard<std::mutex> guard(m_mutex);
        for (unsigned int i = 0; i < m_forceIn.size(); i++) {
            if (m_force[i].data.length() == 6) {
                std::string sensor_name      = m_forceIn[i]->name();
                cnoid::ForceSensorPtr sensor = m_robot->findDevice<cnoid::ForceSensor>(sensor_name);
                cnoid::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
                cnoid::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
                if (DEBUGP) {
                    RTC_INFO_STREAM("wrench [" << m_forceIn[i]->name() << "]");
                    RTC_INFO_STREAM("  raw force = " << data_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")));
                    RTC_INFO_STREAM("  raw moment = " << data_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")));
                }
                if (sensor) {
                    // real force sensor
                    cnoid::Matrix3 sensorR      = sensor->link()->R() * sensor->R_local();
                    ForceMomentOffsetParam &fmp = m_forcemoment_offset_param[sensor_name];
                    cnoid::Vector3 mg           = cnoid::Vector3(0, 0, fmp.link_offset_mass * grav * -1);
                    cnoid::Vector3 cxmg         = cnoid::Vector3(sensorR * fmp.link_offset_centroid).cross(mg);
                    // Sensor offset calib
                    if (fmp.sensor_offset_calib_counter > 0) { // while calibrating
                        fmp.force_offset_sum += (data_p - sensorR.transpose() * mg);
                        fmp.moment_offset_sum += (data_r - sensorR.transpose() * cxmg);
                        fmp.sensor_offset_calib_counter--;
                        if (fmp.sensor_offset_calib_counter == 0) {
                            fmp.force_offset  = fmp.force_offset_sum / max_sensor_offset_calib_counter;
                            fmp.moment_offset = fmp.moment_offset_sum / max_sensor_offset_calib_counter;
                            sem_post(&(fmp.wait_sem));
                        }
                    }
                    // force and moments which do not include offsets
                    fmp.off_force  = sensorR * (data_p - fmp.force_offset) - mg;
                    fmp.off_moment = sensorR * (data_r - fmp.moment_offset) - cxmg;
                    // convert absolute force -> sensor local force
                    fmp.off_force  = cnoid::Vector3(sensorR.transpose() * fmp.off_force);
                    fmp.off_moment = cnoid::Vector3(sensorR.transpose() * fmp.off_moment);
                    for (size_t j = 0; j < 3; j++) {
                        m_force[i].data[j]     = fmp.off_force(j);
                        m_force[i].data[3 + j] = fmp.off_moment(j);
                    }
                    if (DEBUGP) {
                        RTC_INFO_STREAM("  off force = " << fmp.off_force.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")));
                        RTC_INFO_STREAM("  off moment = " << fmp.off_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")));
                    }
                } else {
                    RTC_INFO_STREAM("unknwon force param " << sensor_name);
                }
            }
        }
    }
    for (unsigned int i = 0; i < m_forceOut.size(); i++) {
        m_forceOut[i]->write();
    }
    return RTC::RTC_OK;
}

void RemoveForceSensorLinkOffset2::updateRootLinkPosRot(const cnoid::Vector3 &rpy) {
    cnoid::DeviceList<cnoid::AccelerationSensor> acc_sensors(m_robot->devices());
    if (acc_sensors.size() > 0) {
        cnoid::AccelerationSensorPtr sensor = acc_sensors[0];
        cnoid::Quaternion q1(cnoid::Matrix3(sensor->link()->R() * sensor->R_local()).transpose());
        cnoid::Quaternion q2(m_robot->rootLink()->R());
        cnoid::Quaternion q3 = q1 * q2;
        cnoid::Quaternion q4(cnoid::rotFromRpy(rpy(0), rpy(1), rpy(2)));
        cnoid::Quaternion q5 = q4 * q3.normalized();
        m_robot->rootLink()->setRotation(q5.normalized().toRotationMatrix());
    }
}

void RemoveForceSensorLinkOffset2::printForceMomentOffsetParam(const std::string &i_name_) {
    RTC_INFO_STREAM(
        "  force_offset = " << m_forcemoment_offset_param[i_name_].force_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                            << "[N]");
    RTC_INFO_STREAM("  moment_offset = " << m_forcemoment_offset_param[i_name_].moment_offset.format(
                                                Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                                         << "[Nm]");
    RTC_INFO_STREAM("  link_offset_centroid = " << m_forcemoment_offset_param[i_name_].link_offset_centroid.format(
                                                       Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                                                << "[m]");
    RTC_INFO_STREAM("  link_offset_mass = " << m_forcemoment_offset_param[i_name_].link_offset_mass << "[kg]");
};

bool RemoveForceSensorLinkOffset2::setForceMomentOffsetParam(const std::string &i_name_,
                                                             const OpenHRP::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam &i_param_) {
    RTC_INFO_STREAM("setForceMomentOffsetParam [" << i_name_ << "]");
    if (m_forcemoment_offset_param.find(i_name_) != m_forcemoment_offset_param.end()) {
        memcpy(m_forcemoment_offset_param[i_name_].force_offset.data(), i_param_.force_offset.get_buffer(), sizeof(double) * 3);
        memcpy(m_forcemoment_offset_param[i_name_].moment_offset.data(), i_param_.moment_offset.get_buffer(), sizeof(double) * 3);
        memcpy(m_forcemoment_offset_param[i_name_].link_offset_centroid.data(), i_param_.link_offset_centroid.get_buffer(), sizeof(double) * 3);
        m_forcemoment_offset_param[i_name_].link_offset_mass = i_param_.link_offset_mass;
        this->printForceMomentOffsetParam(i_name_);
        return true;
    } else {
        RTC_INFO_STREAM("No such limb: " << i_name_);
        return false;
    }
}

bool RemoveForceSensorLinkOffset2::getForceMomentOffsetParam(const std::string &i_name_,
                                                             OpenHRP::RemoveForceSensorLinkOffset2Service::forcemomentOffsetParam &i_param_) {
    if (m_forcemoment_offset_param.find(i_name_) != m_forcemoment_offset_param.end()) {
        // std::cerr << "OK " << i_name_ << " in getForceMomentOffsetParam" << std::endl;
        memcpy(i_param_.force_offset.get_buffer(), m_forcemoment_offset_param[i_name_].force_offset.data(), sizeof(double) * 3);
        memcpy(i_param_.moment_offset.get_buffer(), m_forcemoment_offset_param[i_name_].moment_offset.data(), sizeof(double) * 3);
        memcpy(i_param_.link_offset_centroid.get_buffer(), m_forcemoment_offset_param[i_name_].link_offset_centroid.data(), sizeof(double) * 3);
        i_param_.link_offset_mass = m_forcemoment_offset_param[i_name_].link_offset_mass;
        return true;
    } else {
        RTC_INFO_STREAM("No such limb " << i_name_ << " in getForceMomentOffsetParam");
        return false;
    }
}

bool RemoveForceSensorLinkOffset2::loadForceMomentOffsetParams(const std::string &filename) {
    RTC_INFO_STREAM("loadForceMomentOffsetParams");
    std::ifstream ifs(filename.c_str());
    if (ifs.is_open()) {
        while (ifs.eof() == 0) {
            std::string tmps;
            ForceMomentOffsetParam tmpp;
            if (ifs >> tmps) {
                if (m_forcemoment_offset_param.find(tmps) != m_forcemoment_offset_param.end()) {
                    for (size_t i = 0; i < 3; i++)
                        ifs >> tmpp.force_offset(i);
                    for (size_t i = 0; i < 3; i++)
                        ifs >> tmpp.moment_offset(i);
                    for (size_t i = 0; i < 3; i++)
                        ifs >> tmpp.link_offset_centroid(i);
                    ifs >> tmpp.link_offset_mass;
                    m_forcemoment_offset_param[tmps] = tmpp;
                    RTC_INFO_STREAM("  " << tmps << "");
                    this->printForceMomentOffsetParam(tmps);
                } else {
                    RTC_INFO_STREAM("] no such (" << tmps << ")");
                    return false;
                }
            }
        }
    } else {
        RTC_INFO_STREAM("failed to open(" << filename << ")");
        return false;
    }
    return true;
};

bool RemoveForceSensorLinkOffset2::dumpForceMomentOffsetParams(const std::string &filename) {
    RTC_INFO_STREAM("dumpForceMomentOffsetParams");
    std::ofstream ofs(filename.c_str());
    if (ofs.is_open()) {
        for (std::map<std::string, ForceMomentOffsetParam>::iterator it = m_forcemoment_offset_param.begin(); it != m_forcemoment_offset_param.end(); it++) {
            ofs << it->first << " ";
            ofs << it->second.force_offset[0] << " " << it->second.force_offset[1] << " " << it->second.force_offset[2] << " ";
            ofs << it->second.moment_offset[0] << " " << it->second.moment_offset[1] << " " << it->second.moment_offset[2] << " ";
            ofs << it->second.link_offset_centroid[0] << " " << it->second.link_offset_centroid[1] << " " << it->second.link_offset_centroid[2] << " ";
            ofs << it->second.link_offset_mass << std::endl;
        }
    } else {
        RTC_INFO_STREAM("failed to open(" << filename << ")");
        return false;
    }
    return true;
};

bool RemoveForceSensorLinkOffset2::removeForceSensorOffset(const ::OpenHRP::RemoveForceSensorLinkOffset2Service::StrSequence &names, const double tm) {
    RTC_INFO_STREAM("removeForceSensorOffset...");

    // Check argument validity
    std::vector<std::string> valid_names, invalid_names, calibrating_names;
    bool is_valid_argument = true;
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        if (names.length() == 0) { // If no sensor names are specified, calibrate all sensors.
            std::cerr << "[" << m_profile.instance_name << "]   No sensor names are specified, calibrate all sensors = [";
            for (auto it = m_forcemoment_offset_param.begin(); it != m_forcemoment_offset_param.end(); it++) {
                valid_names.push_back(it->first);
                std::cerr << it->first << " ";
            }
            std::cerr << "]" << std::endl;
        } else {
            for (size_t i = 0; i < names.length(); i++) {
                std::string name(names[i]);
                if (m_forcemoment_offset_param.find(name) != m_forcemoment_offset_param.end()) {
                    if (m_forcemoment_offset_param[name].sensor_offset_calib_counter == 0) {
                        valid_names.push_back(name);
                    } else {
                        calibrating_names.push_back(name);
                        is_valid_argument = false;
                    }
                } else {
                    invalid_names.push_back(name);
                    is_valid_argument = false;
                }
            }
        }
    }
    // Return if invalid or calibrating
    if (!is_valid_argument) {
        std::cerr << "[" << m_profile.instance_name << "]   Cannot start removeForceSensorOffset, invalid = [";
        for (size_t i = 0; i < invalid_names.size(); i++)
            std::cerr << invalid_names[i] << " ";
        std::cerr << "], calibrating = [";
        for (size_t i = 0; i < calibrating_names.size(); i++)
            std::cerr << calibrating_names[i] << " ";
        std::cerr << "]" << std::endl;
        return false;
    }

    // Start calibration
    //   Print output force before calib
    std::cerr << "[" << m_profile.instance_name << "]   Calibrate sensor names = [";
    for (size_t i = 0; i < valid_names.size(); i++)
        std::cerr << valid_names[i] << " ";
    std::cerr << "]" << std::endl;
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        for (size_t i = 0; i < valid_names.size(); i++) {
            std::cerr << "[" << m_profile.instance_name << "]     Offset-removed force before calib [" << valid_names[i] << "], ";
            std::cerr << "force = "
                      << m_forcemoment_offset_param[valid_names[i]].off_force.format(
                             Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][N]"))
                      << ", ";
            std::cerr << "moment = "
                      << m_forcemoment_offset_param[valid_names[i]].off_moment.format(
                             Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][Nm]"));
            std::cerr << std::endl;
        }
        max_sensor_offset_calib_counter = static_cast<int>(tm / m_dt);
        for (size_t i = 0; i < valid_names.size(); i++) {
            m_forcemoment_offset_param[valid_names[i]].force_offset_sum            = cnoid::Vector3::Zero();
            m_forcemoment_offset_param[valid_names[i]].moment_offset_sum           = cnoid::Vector3::Zero();
            m_forcemoment_offset_param[valid_names[i]].sensor_offset_calib_counter = max_sensor_offset_calib_counter;
        }
    }
    //   Wait
    for (size_t i = 0; i < valid_names.size(); i++) {
        sem_wait(&(m_forcemoment_offset_param[valid_names[i]].wait_sem));
    }
    //   Print output force and offset after calib
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        std::cerr << "[" << m_profile.instance_name << "]   Calibrate done (calib time = " << tm << "[s])" << std::endl;
        for (size_t i = 0; i < valid_names.size(); i++) {
            std::cerr << "[" << m_profile.instance_name << "]     Calibrated offset [" << valid_names[i] << "], ";
            std::cerr << "force_offset = "
                      << m_forcemoment_offset_param[valid_names[i]].force_offset.format(
                             Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][N]"))
                      << ", ";
            std::cerr << "moment_offset = "
                      << m_forcemoment_offset_param[valid_names[i]].moment_offset.format(
                             Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][Nm]"))
                      << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]     Offset-removed force after calib [" << valid_names[i] << "], ";
            std::cerr << "force = "
                      << m_forcemoment_offset_param[valid_names[i]].off_force.format(
                             Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][N]"))
                      << ", ";
            std::cerr << "moment = "
                      << m_forcemoment_offset_param[valid_names[i]].off_moment.format(
                             Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][Nm]"));
            std::cerr << std::endl;
        }
    }
    std::cerr << "[" << m_profile.instance_name << "] removeForceSensorOffset...done" << std::endl;
    return true;
}

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C" {
void RemoveForceSensorLinkOffset2Init(RTC::Manager *manager) {
    RTC::Properties profile(removeforcesensorlinkoffset2_spec);
    manager->registerFactory(profile, RTC::Create<RemoveForceSensorLinkOffset2>, RTC::Delete<RemoveForceSensorLinkOffset2>);
}
};
