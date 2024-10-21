#include <data_logger/DataLogger2.h>
#include <hrpsys/idl/pointcloud.hh>
#include <robot_hardware/idl/RobotHardware2Service.hh>


static const char *nullcomponent_spec[] = {"implementation_id", "DataLogger2", "type_name", "DataLogger2",
                                           "description", "data logger component", "version", "1.0", "vendor", "AIST",
                                           "category", "example", "activity_type", "DataFlowComponent", "max_instance",
                                           "10", "language", "C++", "lang_type", "compile",
                                           // Configuration variables
                                           "conf.default.log_precision", "0", ""};

#define LOG_SET_PRECISION(strm)                                                                                        \
    int prc;                                                                                                           \
    if (precision != 0) {                                                                                              \
        prc = os.precision();                                                                                          \
        os << std::scientific << std::setprecision(precision);                                                         \
    }

#define LOG_UNSET_PRECISION(strm)                                                                                      \
    if (precision != 0) os << std::fixed << std::setprecision(prc);

void printData(std::ostream &os, const RTC::Acceleration3D &data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    os << data.ax << " " << data.ay << " " << data.az << " ";
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, const RTC::Velocity2D &data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    os << data.vx << " " << data.vy << " " << data.va << " ";
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, const RTC::Pose3D &data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    os << data.position.x << " " << data.position.y << " " << data.position.z << " " << data.orientation.r << " "
       << data.orientation.p << " " << data.orientation.y << " ";
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, const RTC::AngularVelocity3D &data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    os << data.avx << " " << data.avy << " " << data.avz << " ";
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, const RTC::Point3D &data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    os << data.x << " " << data.y << " " << data.z << " ";
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, const RTC::Vector3D &data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    os << data.x << " " << data.y << " " << data.z << " ";
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, const RTC::Orientation3D &data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    os << data.r << " " << data.p << " " << data.y << " ";
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, const PointCloudTypes::PointCloud &data, unsigned int precision = 0) {
    uint npoint = data.data.length() / data.point_step;
    os << data.width << " " << data.height << " " << data.type << " " << npoint;
    float *ptr = (float *)data.data.get_buffer();
    std::string type(data.type);
    if (type != "xyz" && type != "xyzrgb") {
        std::cerr << "point cloud type(" << type << ") is not supported" << std::endl;
        return;
    }
    for (uint i = 0; i < npoint; i++) {
        os << " " << *ptr++ << " " << *ptr++ << " " << *ptr++;
        if (type == "xyzrgb") {
            unsigned char *rgb = (unsigned char *)ptr;
            os << " " << (int)rgb[0] << " " << (int)rgb[1] << " " << (int)rgb[2];
            ptr++;
        }
    }
}

template <class T> std::ostream &operator<<(std::ostream &os, const _CORBA_Unbounded_Sequence<T> &data) {
    for (unsigned int j = 0; j < data.length(); j++) {
        os << data[j] << " ";
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OpenHRP::RobotHardware2Service::DblSequence6 &data) {
    for (unsigned int j = 0; j < data.length(); j++) {
        os << data[j] << " ";
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OpenHRP::RobotHardware2Service::DblSequence3 &data) {
    for (unsigned int j = 0; j < data.length(); j++) {
        os << data[j] << " ";
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const OpenHRP::RobotHardware2Service::BatteryState &data) {
    os << data.voltage << " " << data.current << " " << data.soc << " ";
    return os;
}

template <class T> void printData(std::ostream &os, const T &data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    for (unsigned int j = 0; j < data.length(); j++) {
        os << data[j] << " ";
    }
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, double data, unsigned int precision = 0) {
    LOG_SET_PRECISION(os);
    os << data << " ";
    LOG_UNSET_PRECISION(os);
}

void printData(std::ostream &os, const OpenHRP::RobotHardware2Service::RobotState2 &data, unsigned int precision = 0) {
    printData(os, data.angle, precision);
    printData(os, data.command, precision);
    printData(os, data.torque, precision);
    printData(os, data.servoState, precision);
    printData(os, data.force, precision);
    printData(os, data.rateGyro, precision);
    printData(os, data.accel, precision);
    printData(os, data.batteries, precision);
    printData(os, data.voltage, precision);
    printData(os, data.current, precision);
    printData(os, data.temperature, precision);
}

template <class T> class LoggerPort : public LoggerPortBase {
  public:
    LoggerPort(const char *name) : m_port(name, m_data) {}
    const char *name() { return m_port.name(); }
    virtual void dumpLog(std::ostream &os, unsigned int precision = 0) {
        os.setf(std::ios::fixed, std::ios::floatfield);
        for (unsigned int i = 0; i < m_log.size(); i++) {
            printLog(os, m_log[i], precision);
        }
    }
    void printLog(std::ostream &os, T &data, unsigned int precision = 0) {
        os << std::setprecision(6) << (data.tm.sec + data.tm.nsec / 1e9) << " ";
        // data
        printData(os, data.data, precision);
        os << std::endl;
    }
    RTC::InPort<T> &port() { return m_port; }
    void log() {
        if (m_port.isNew()) {
            m_port.read();
            m_log.push_back(m_data);
            while (m_log.size() > m_maxLength) {
                m_log.pop_front();
            }
        }
    }
    void clear() { m_log.clear(); }

  protected:
    RTC::InPort<T> m_port;
    T m_data;
    std::deque<T> m_log;
};

class LoggerPortForPointCloud : public LoggerPort<PointCloudTypes::PointCloud> {
  public:
    LoggerPortForPointCloud(const char *name) : LoggerPort<PointCloudTypes::PointCloud>(name) {}
    void dumpLog(std::ostream &os, unsigned int precision = 0) {
        os.setf(std::ios::fixed, std::ios::floatfield);
        for (unsigned int i = 0; i < m_log.size(); i++) {
            // time
            os << std::setprecision(6) << (m_log[i].tm.sec + m_log[i].tm.nsec / 1e9) << " ";
            // data
            printData(os, m_log[i], precision);
            os << std::endl;
        }
    }
};

DataLogger2::DataLogger2(RTC::Manager *manager)
    : RTC::DataFlowComponentBase(manager), m_emergencySignalIn("emergencySignal", m_emergencySignal),
      m_DataLogger2ServicePort("DataLogger2Service"), m_suspendFlag(false), m_log_precision(0), dummy(0) {
    m_service0.setLogger(this);
}

DataLogger2::~DataLogger2() {}


RTC::ReturnCode_t DataLogger2::onInitialize() {
    RTC_INFO_STREAM("onInitialize()");
    bindParameter("log_precision", m_log_precision, "0");

    // Set InPort buffers
    addInPort("emergencySignal", m_emergencySignalIn);

    // Set OutPort buffer

    // Set service provider to Ports
    m_DataLogger2ServicePort.registerProvider("service0", "DataLogger2Service", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_DataLogger2ServicePort);

    // Bind variables and configuration variable

    return RTC::RTC_OK;
}

RTC::ReturnCode_t DataLogger2::onActivated(RTC::UniqueId ec_id) { return RTC::RTC_OK; }

RTC::ReturnCode_t DataLogger2::onDeactivated(RTC::UniqueId ec_id) { return RTC::RTC_OK; }

RTC::ReturnCode_t DataLogger2::onExecute(RTC::UniqueId ec_id) {
    if (ec_id == 0) {
        if (m_emergencySignalIn.isNew()) {
            m_emergencySignalIn.read();
            time_t sec     = time(NULL);
            struct tm *tm_ = localtime(&sec);
            char date[20];
            strftime(date, 20, "%Y-%m-%d", tm_);
            char basename[32];
            sprintf(basename, "emglog-%s-%02d%02d", date, tm_->tm_hour, tm_->tm_min);
            RTC_INFO_STREAM("received emergency signal. saving log files(" << basename << ")");
            save(basename);
            while (m_emergencySignalIn.isNew()) {
                m_emergencySignalIn.read();
            }
        }
    } else {
        std::lock_guard<std::mutex> guard(m_suspendFlagMutex);

        if (m_suspendFlag) return RTC::RTC_OK;

        for (unsigned int i = 0; i < m_ports.size(); i++) {
            m_ports[i]->log();
        }
    }
    return RTC::RTC_OK;
}

bool DataLogger2::add(const char *i_type, const char *i_name) {
    suspendLogging();
    for (unsigned int i = 0; i < m_ports.size(); i++) {
        if (strcmp(m_ports[i]->name(), i_name) == 0) {
            RTC_WARN_STREAM("Logger port named \"" << i_name << "\" already exists");
            resumeLogging();
            return false;
        }
    }

    LoggerPortBase *new_port = NULL;
    if (strcmp(i_type, "TimedDoubleSeq") == 0) {
        LoggerPort<RTC::TimedDoubleSeq> *lp = new LoggerPort<RTC::TimedDoubleSeq>(i_name);
        new_port                            = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedLongSeq") == 0) {
        LoggerPort<RTC::TimedLongSeq> *lp = new LoggerPort<RTC::TimedLongSeq>(i_name);
        new_port                          = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedBooleanSeq") == 0) {
        LoggerPort<RTC::TimedBooleanSeq> *lp = new LoggerPort<RTC::TimedBooleanSeq>(i_name);
        new_port                             = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedLongSeqSeq") == 0) {
        LoggerPort<OpenHRP::TimedLongSeqSeq> *lp = new LoggerPort<OpenHRP::TimedLongSeqSeq>(i_name);
        new_port                                 = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedPoint3D") == 0) {
        LoggerPort<RTC::TimedPoint3D> *lp = new LoggerPort<RTC::TimedPoint3D>(i_name);
        new_port                          = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedVector3D") == 0) {
        LoggerPort<RTC::TimedVector3D> *lp = new LoggerPort<RTC::TimedVector3D>(i_name);
        new_port                           = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedOrientation3D") == 0) {
        LoggerPort<RTC::TimedOrientation3D> *lp = new LoggerPort<RTC::TimedOrientation3D>(i_name);
        new_port                                = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedAcceleration3D") == 0) {
        LoggerPort<RTC::TimedAcceleration3D> *lp = new LoggerPort<RTC::TimedAcceleration3D>(i_name);
        new_port                                 = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedAngularVelocity3D") == 0) {
        LoggerPort<RTC::TimedAngularVelocity3D> *lp = new LoggerPort<RTC::TimedAngularVelocity3D>(i_name);
        new_port                                    = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedVelocity2D") == 0) {
        LoggerPort<RTC::TimedVelocity2D> *lp = new LoggerPort<RTC::TimedVelocity2D>(i_name);
        new_port                             = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedPose3D") == 0) {
        LoggerPort<RTC::TimedPose3D> *lp = new LoggerPort<RTC::TimedPose3D>(i_name);
        new_port                         = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "PointCloud") == 0) {
        LoggerPort<PointCloudTypes::PointCloud> *lp = new LoggerPortForPointCloud(i_name);
        new_port                                    = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else if (strcmp(i_type, "TimedRobotState2") == 0) {
        LoggerPort<OpenHRP::RobotHardware2Service::TimedRobotState2> *lp =
            new LoggerPort<OpenHRP::RobotHardware2Service::TimedRobotState2>(i_name);
        new_port = lp;
        if (!addInPort(i_name, lp->port())) {
            resumeLogging();
            return false;
        }
    } else {
        RTC_WARN_STREAM("DataLogger: unsupported data type(" << i_type << ")");
        resumeLogging();
        return false;
    }
    m_ports.push_back(new_port);
    resumeLogging();
    return true;
}

bool DataLogger2::save(const char *i_basename) {
    suspendLogging();
    bool ret = true;
    for (unsigned int i = 0; i < m_ports.size(); i++) {
        std::string fname = i_basename;
        fname.append(".");
        fname.append(m_ports[i]->name());
        std::ofstream ofs(fname.c_str());
        if (ofs.is_open()) {
            m_ports[i]->dumpLog(ofs, m_log_precision);
        } else {
            RTC_WARN_STREAM("failed to open(" << fname << ")");
            ret = false;
        }
    }
    if (ret) RTC_INFO_STREAM("Save log to " << i_basename << ".*");
    resumeLogging();
    return ret;
}

bool DataLogger2::clear() {
    suspendLogging();
    for (unsigned int i = 0; i < m_ports.size(); i++) {
        m_ports[i]->clear();
    }
    RTC_INFO_STREAM("Log cleared");
    resumeLogging();
    return true;
}

void DataLogger2::suspendLogging() {
    std::lock_guard<std::mutex> guard(m_suspendFlagMutex);
    m_suspendFlag = true;
}

void DataLogger2::resumeLogging() {
    std::lock_guard<std::mutex> guard(m_suspendFlagMutex);
    m_suspendFlag = false;
}

void DataLogger2::maxLength(unsigned int len) {
    suspendLogging();
    for (unsigned int i = 0; i < m_ports.size(); i++) {
        m_ports[i]->maxLength(len);
    }
    RTC_INFO_STREAM("Log max length is set to " << len);
    resumeLogging();
}

extern "C" {
void DataLogger2Init(RTC::Manager *manager) {
    RTC::Properties profile(nullcomponent_spec);
    manager->registerFactory(profile, RTC::Create<DataLogger2>, RTC::Delete<DataLogger2>);
}
};
