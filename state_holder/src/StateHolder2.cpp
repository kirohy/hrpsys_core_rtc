#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>
#include <cnoid/ForceSensor>
#include <rtm/CorbaNaming.h>
#include <state_holder/StateHolder2.h>
#include <time.h>

// clang-format off
static const char *stateholder2_spec[] = {
    "implementation_id", "StateHolder2",
    "type_name",         "StateHolder2",
    "description",       "state holder",
    "version",           "0.0.0",
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""};
// clang-format on

StateHolder2::StateHolder2(RTC::Manager *manager)
    : RTC::DataFlowComponentBase(manager), m_currentQIn("currentQIn", m_currentQ), m_qIn("qIn", m_q),
      m_tqIn("tqIn", m_tq), m_basePosIn("basePosIn", m_basePos), m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_zmpIn("zmpIn", m_zmp), m_optionalDataIn("optionalDataIn", m_optionalData), m_qOut("qOut", m_q),
      m_tqOut("tqOut", m_tq), m_basePosOut("basePosOut", m_basePos), m_baseRpyOut("baseRpyOut", m_baseRpy),
      m_baseTformOut("baseTformOut", m_baseTform), m_basePoseOut("basePoseOut", m_basePose), m_zmpOut("zmpOut", m_zmp),
      m_optionalDataOut("optionalDataOut", m_optionalData), m_StateHolder2ServicePort("StateHolder2Service"),
      m_TimeKeeper2ServicePort("TimeKeeper2Service"), m_timeCount(0), dummy(0) {

    m_service0.setComponent(this);
    m_service1.setComponent(this);
    m_requestGoActual = false;

    sem_init(&m_waitSem, 0, 0);
    sem_init(&m_timeSem, 0, 0);
}

StateHolder2::~StateHolder2() {}


RTC::ReturnCode_t StateHolder2::onInitialize() {
    RTC_INFO_STREAM("onInitialize()");
    // Set InPort buffers
    addInPort("currentQIn", m_currentQIn);
    addInPort("qIn", m_qIn);
    addInPort("tqIn", m_tqIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("zmpIn", m_zmpIn);
    addInPort("optionalDataIn", m_optionalDataIn);

    // Set OutPort buffer
    addOutPort("qOut", m_qOut);
    addOutPort("tqOut", m_tqOut);
    addOutPort("basePosOut", m_basePosOut);
    addOutPort("baseRpyOut", m_baseRpyOut);
    addOutPort("baseTformOut", m_baseTformOut);
    addOutPort("basePoseOut", m_basePoseOut);
    addOutPort("zmpOut", m_zmpOut);
    addOutPort("optionalDataOut", m_optionalDataOut);

    // Set service provider to Ports
    m_StateHolder2ServicePort.registerProvider("service0", "StateHolder2Service", m_service0);
    m_TimeKeeper2ServicePort.registerProvider("service1", "TimeKeeper2Service", m_service1);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_StateHolder2ServicePort);
    addPort(m_TimeKeeper2ServicePort);

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
    cnoid::BodyPtr robot = body_loader.load(body_filename);
    if (!robot) {
        RTC_WARN_STREAM("failed to load model [" << body_filename << "]");
        return RTC::RTC_ERROR;
    } else {
        RTC_INFO_STREAM("successed to load model [" << body_filename << "]");
    }

    cnoid::LinkPtr li  = robot->rootLink();
    cnoid::Vector3 p   = li->translation();
    cnoid::Matrix3 R   = li->rotation().matrix();
    cnoid::Vector3 rpy = cnoid::rpyFromRot(R);

    m_baseTform.data.length(12);
    double *T = m_baseTform.data.get_buffer();
    T[0]      = R(0, 0);
    T[1]      = R(0, 1);
    T[2]      = R(0, 2);
    T[3]      = p[0];
    T[4]      = R(0, 0);
    T[5]      = R(0, 1);
    T[6]      = R(0, 2);
    T[7]      = p[1];
    T[8]      = R(0, 0);
    T[9]      = R(0, 1);
    T[10]     = R(0, 2);
    T[11]     = p[2];

    m_basePos.data.x = m_basePose.data.position.x = p[0];
    m_basePos.data.y = m_basePose.data.position.y = p[1];
    m_basePos.data.z = m_basePose.data.position.z = p[2];
    m_baseRpy.data.r = m_basePose.data.orientation.r = rpy[0];
    m_baseRpy.data.p = m_basePose.data.orientation.p = rpy[1];
    m_baseRpy.data.y = m_basePose.data.orientation.y = rpy[2];
    m_zmp.data.x = m_zmp.data.y = m_zmp.data.z = 0.0;

    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    cnoid::DeviceList<cnoid::ForceSensor> sensors(robot->devices());
    for (size_t i = 0; i < sensors.size(); i++) {
        if (std::string(sensors[i]->typeName()) == "ForceSensor") { fsensor_names.push_back(sensors[i]->name()); }
    }

    int npforce = fsensor_names.size();

    // TODO
    //   find names for virtual force sensors
    // coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    // unsigned int nvforce = virtual_force_sensor.size()/10;
    // for (unsigned int i=0; i<nvforce; i++){
    //   fsensor_names.push_back(virtual_force_sensor[i*10+0]);
    // }
    //   add ports for all force sensors
    // unsigned int nforce  = npforce + nvforce;

    unsigned int nforce = npforce;
    m_wrenches.resize(nforce);
    m_wrenchesIn.resize(nforce);
    m_wrenchesOut.resize(nforce);
    for (unsigned int i = 0; i < nforce; i++) {
        m_wrenchesIn[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq>>(
            std::string(fsensor_names[i] + "In").c_str(), m_wrenches[i]);
        m_wrenchesOut[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq>>(
            std::string(fsensor_names[i] + "Out").c_str(), m_wrenches[i]);
        m_wrenches[i].data.length(6);
        m_wrenches[i].data[0] = m_wrenches[i].data[1] = m_wrenches[i].data[2] = 0.0;
        m_wrenches[i].data[3] = m_wrenches[i].data[4] = m_wrenches[i].data[5] = 0.0;
        registerInPort(std::string(fsensor_names[i] + "In").c_str(), *m_wrenchesIn[i]);
        registerOutPort(std::string(fsensor_names[i] + "Out").c_str(), *m_wrenchesOut[i]);
    }

    return RTC::RTC_OK;
}


RTC::ReturnCode_t StateHolder2::onExecute(RTC::UniqueId ec_id) {
    // RTC_INFO_STREAM("onExecute(" << ec_id << ")");
    RTC::Time tm;

    if (m_currentQIn.isNew()) {
        m_currentQIn.read();
        tm = m_currentQ.tm;
    } else {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        tm.sec  = ts.tv_sec;
        tm.nsec = ts.tv_nsec;
    }

    if (m_qIn.isNew()) { m_qIn.read(); }
    if (m_tqIn.isNew()) { m_tqIn.read(); }
    if (m_requestGoActual || (m_q.data.length() == 0 && m_currentQ.data.length() > 0)) {
        m_q = m_currentQ;
        if (m_q.data.length() != m_tq.data.length()) {
            m_tq.data.length(m_q.data.length());
            for (size_t i = 0; i < m_tq.data.length(); i++) {
                m_tq.data[i] = 0;
            }
        }
        // Reset reference wrenches to zero
        for (unsigned int i = 0; i < m_wrenchesIn.size(); i++) {
            m_wrenches[i].data[0] = m_wrenches[i].data[1] = m_wrenches[i].data[2] = 0.0;
            m_wrenches[i].data[3] = m_wrenches[i].data[4] = m_wrenches[i].data[5] = 0.0;
        }
    }

    if (m_requestGoActual) {
        m_requestGoActual = false;
        sem_post(&m_waitSem);
    }

    if (m_basePosIn.isNew()) { m_basePosIn.read(); }

    if (m_baseRpyIn.isNew()) { m_baseRpyIn.read(); }

    if (m_zmpIn.isNew()) { m_zmpIn.read(); }

    if (m_optionalDataIn.isNew()) { m_optionalDataIn.read(); }

    for (size_t i = 0; i < m_wrenchesIn.size(); i++) {
        if (m_wrenchesIn[i]->isNew()) { m_wrenchesIn[i]->read(); }
    }

    double *a        = m_baseTform.data.get_buffer();
    a[0]             = m_basePos.data.x;
    a[1]             = m_basePos.data.y;
    a[2]             = m_basePos.data.z;
    cnoid::Matrix3 R = cnoid::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    for (int i = 0; i < 3; i++) { // set row major
        for (int j = 0; j < 3; j++) {
            a[3 + i * 3 + j] = R(i, j);
        }
    }

    m_basePose.data.position    = m_basePos.data;
    m_basePose.data.orientation = m_baseRpy.data;

    // put timestamps
    m_q.tm         = tm;
    m_tq.tm        = tm;
    m_baseTform.tm = tm;
    m_basePos.tm   = tm;
    m_baseRpy.tm   = tm;
    m_zmp.tm       = tm;
    m_basePose.tm  = tm;
    for (size_t i = 0; i < m_wrenches.size(); i++) {
        m_wrenches[i].tm = tm;
    }

    // write
    if (m_q.data.length() > 0) { m_qOut.write(); }
    if (m_tq.data.length() > 0) { m_tqOut.write(); }
    m_baseTformOut.write();
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_zmpOut.write();
    m_basePoseOut.write();
    m_optionalDataOut.write();
    for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
        m_wrenchesOut[i]->write();
    }

    if (m_timeCount > 0) {
        m_timeCount--;
        if (m_timeCount == 0) sem_post(&m_timeSem);
    }

    return RTC::RTC_OK;
}

void StateHolder2::goActual() {
    RTC_INFO_STREAM("StateHolder::goActual()");
    m_requestGoActual = true;
    sem_wait(&m_waitSem);
}

void StateHolder2::getCommand(state_holder::StateHolder2Service::Command &com) {
    com.jointRefs.length(m_q.data.length());
    memcpy(com.jointRefs.get_buffer(), m_q.data.get_buffer(), sizeof(double) * m_q.data.length());
    com.baseTransform.length(12);
    com.baseTransform[0] = m_basePos.data.x;
    com.baseTransform[1] = m_basePos.data.y;
    com.baseTransform[2] = m_basePos.data.z;
    cnoid::Matrix3 R     = cnoid::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    double *a            = com.baseTransform.get_buffer();
    for (int i = 0; i < 3; i++) { // set row major
        for (int j = 0; j < 3; j++) {
            a[3 + i * 3 + j] = R(i, j);
        }
    }
    com.zmp.length(3);
    com.zmp[0] = m_zmp.data.x;
    com.zmp[1] = m_zmp.data.y;
    com.zmp[2] = m_zmp.data.z;
}

void StateHolder2::wait(CORBA::Double tm) {
    m_timeCount = tm / m_dt;
    sem_wait(&m_timeSem);
}

extern "C" {
void StateHolder2Init(RTC::Manager *manager) {
    RTC::Properties profile(stateholder2_spec);
    manager->registerFactory(profile, RTC::Create<StateHolder2>, RTC::Delete<StateHolder2>);
}
};
