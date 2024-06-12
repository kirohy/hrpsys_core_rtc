#include <cnoid/BodyLoader>
#include <robot_hardware/RobotHardware2.h>
#include <robot_hardware/robot.h>
#include <rtm/CorbaNaming.h>

static const char *robothardware2_spec[] = {"implementation_id", "RobotHardware2", "type_name", "RobotHardware2", "description", "RobotHardware2", "version",
                                            "1.0.0", "vendor", "JSK", "category", "example", "activity_type", "DataFlowComponent", "max_instance", "1",
                                            "language", "C++", "lang_type", "compile",
                                            // Configuration variables
                                            "conf.default.isDemoMode", "0", "conf.default.fzLimitRatio", "2.0", "conf.default.servoErrorLimit", ",",
                                            "conf.default.jointAccelerationLimit", "0", "conf.default.servoOnDelay", "0",

                                            ""};
// </rtc-template>

RobotHardware2::RobotHardware2(RTC::Manager *manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_isDemoMode(0), m_qRefIn("qRef", m_qRef), m_dqRefIn("dqRef", m_dqRef), m_ddqRefIn("ddqRef", m_ddqRef), m_tauRefIn("tauRef", m_tauRef), m_qOut("q", m_q),
      m_dqOut("dq", m_dq), m_tauOut("tau", m_tau), m_ctauOut("ctau", m_ctau), m_pdtauOut("pdtau", m_pdtau), m_servoStateOut("servoState", m_servoState),
      m_emergencySignalOut("emergencySignal", m_emergencySignal), m_rstate2Out("rstate2", m_rstate2), m_RobotHardware2ServicePort("RobotHardware2Service"),
      // </rtc-template>
      dummy(0) {}

RobotHardware2::~RobotHardware2() {}


RTC::ReturnCode_t RobotHardware2::onInitialize() {
    RTC_INFO_STREAM("onInitialize()");
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">

    addInPort("qRef", m_qRefIn);
    addInPort("dqRef", m_dqRefIn);
    addInPort("ddqRef", m_ddqRefIn);
    addInPort("tauRef", m_tauRefIn);

    addOutPort("q", m_qOut);
    addOutPort("dq", m_dqOut);
    addOutPort("tau", m_tauOut);
    addOutPort("ctau", m_ctauOut);
    addOutPort("pdtau", m_pdtauOut);
    addOutPort("servoState", m_servoStateOut);
    addOutPort("emergencySignal", m_emergencySignalOut);
    addOutPort("rstate2", m_rstate2Out);

    // Set service provider to Ports
    m_RobotHardware2ServicePort.registerProvider("service0", "RobotHardware2Service", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_RobotHardware2ServicePort);

    // </rtc-template>

    RTC::Properties &prop = this->getProperties();

    std::string buf;
    this->getProperty("dt", buf);
    double dt = std::stod(buf);
    if (dt <= 0.0) {
        this->getProperty("exec_cxt.periodic.rate", buf);
        double rate = std::stod(buf);
        if (rate > 0.0) {
            dt = 1.0 / rate;
        } else {
            RTC_WARN_STREAM("dt is invalid");
            return RTC::RTC_ERROR;
        }
    }
    m_robot = std::make_shared<robot>(dt);


    // RTC::Manager& rtcManager = RTC::Manager::instance();
    // std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    // int comPos = nameServer.find(",");
    // if (comPos < 0){
    //     comPos = nameServer.length();
    // }
    // nameServer = nameServer.substr(0, comPos);
    // RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    cnoid::BodyLoader body_loader;
    std::string body_filename;
    this->getProperty("model", body_filename);
    if (body_filename.find("file://") == 0) { body_filename.erase(0, strlen("file://")); }
    if (!body_loader.load(m_robot.get(), body_filename)) {
        RTC_WARN_STREAM("failed to load model [" << body_filename << "]");
        return RTC::RTC_ERROR;
    } else {
        RTC_INFO_STREAM("successed to load model [" << body_filename << "]");
    }

    std::vector<std::string> keys = prop.propertyNames();
    for (unsigned int i = 0; i < keys.size(); i++) {
        m_robot->setProperty(keys[i].c_str(), prop[keys[i]].c_str());
    }
    RTC_INFO_STREAM("dof = " << m_robot->numJoints());
    if (!m_robot->init()) return RTC::RTC_ERROR;

    m_service0.setRobot(m_robot);

    m_q.data.length(m_robot->numJoints());
    m_dq.data.length(m_robot->numJoints());
    m_tau.data.length(m_robot->numJoints());
    m_ctau.data.length(m_robot->numJoints());
    m_pdtau.data.length(m_robot->numJoints());
    m_servoState.data.length(m_robot->numJoints());
    m_qRef.data.length(m_robot->numJoints());
    m_dqRef.data.length(m_robot->numJoints());
    m_ddqRef.data.length(m_robot->numJoints());
    m_tauRef.data.length(m_robot->numJoints());

    m_rate.resize(m_robot->numGyroSensors());
    m_rateOut.resize(m_robot->numGyroSensors());
    for (int i = 0; i < m_robot->numGyroSensors(); i++) {
        cnoid::RateGyroSensorPtr s = m_robot->findGyroSensor(i);
        RTC_INFO_STREAM("Find RateGyroSensor: " << s->name());
        m_rateOut[i] = std::make_unique<RTC::OutPort<RTC::TimedAngularVelocity3D>>(s->name().c_str(), m_rate[i]);
        registerOutPort(s->name().c_str(), *m_rateOut[i]);
    }

    m_acc.resize(m_robot->numAccSensors());
    m_accOut.resize(m_robot->numAccSensors());
    for (int i = 0; i < m_robot->numAccSensors(); i++) {
        cnoid::AccelerationSensorPtr s = m_robot->findAccSensor(i);
        RTC_INFO_STREAM("Find AccelerationSensor: " << s->name());
        m_accOut[i] = std::make_unique<RTC::OutPort<RTC::TimedAcceleration3D>>(s->name().c_str(), m_acc[i]);
        registerOutPort(s->name().c_str(), *m_accOut[i]);
    }

    cnoid::DeviceList<cnoid::ForceSensor> force_sensors(m_robot->devices());
    m_force.resize(force_sensors.size());
    m_forceOut.resize(force_sensors.size());
    for (int i = 0; i < force_sensors.size(); i++) {
        cnoid::ForceSensorPtr s = m_robot->findForceSensor(i);
        RTC_INFO_STREAM("Find ForceSensor: " << s->name());
        m_forceOut[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq>>(s->name().c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerOutPort(s->name().c_str(), *m_forceOut[i]);
    }


    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("isDemoMode", m_isDemoMode, "0");
    // bindParameter("servoErrorLimit", m_robot->m_servoErrorLimit, ",");
    bindParameter("fzLimitRatio", m_robot->m_fzLimitRatio, "2");
    bindParameter("jointAccelerationLimit", m_robot->m_accLimit, "0");
    bindParameter("servoOnDelay", m_robot->m_servoOnDelay, "0");

    // </rtc-template>

    return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t RobotHardware::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RobotHardware2::onExecute(RTC::UniqueId ec_id) {
    // std::cout << "RobotHardware:onExecute(" << ec_id << ")" << std::endl;
    RTC::Time tm;
    this->getTimeNow(tm);

    if (!m_isDemoMode) {
        robot::emg_reason reason;
        int id;
        if (m_robot->checkEmergency(reason, id)) {
            if (reason == robot::EMG_SERVO_ERROR || reason == robot::EMG_POWER_OFF) {
                m_robot->servo("all", false);
                m_emergencySignal.data = reason;
                m_emergencySignalOut.write();
            } else if (reason == robot::EMG_SERVO_ALARM) {
                m_emergencySignal.data = reason;
                m_emergencySignalOut.write();
            }
        }
    }

    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
        // std::cout << "RobotHardware: qRef[21] = " << m_qRef.data[21] << std::endl;
        if (!m_isDemoMode && m_robot->checkJointCommands(m_qRef.data.get_buffer())) {
            m_robot->servo("all", false);
            m_emergencySignal.data = robot::EMG_SERVO_ERROR;
            m_emergencySignalOut.write();
        } else {
            // output to iob
            m_robot->writeJointCommands(m_qRef.data.get_buffer());
        }
    }
    if (m_dqRefIn.isNew()) {
        m_dqRefIn.read();
        // std::cout << "RobotHardware: dqRef[21] = " << m_dqRef.data[21] << std::endl;
        // output to iob
        m_robot->writeVelocityCommands(m_dqRef.data.get_buffer());
    }
    if (m_ddqRefIn.isNew()) {
        m_ddqRefIn.read();
        // std::cout << "RobotHardware: dqRef[21] = " << m_dqRef.data[21] << std::endl;
        // output to iob
        m_robot->writeAccelerationCommands(m_ddqRef.data.get_buffer());
    }
    if (m_tauRefIn.isNew()) {
        m_tauRefIn.read();
        // std::cout << "RobotHardware: tauRef[21] = " << m_tauRef.data[21] << std::endl;
        // output to iob
        m_robot->writeTorqueCommands(m_tauRef.data.get_buffer());
    }

    // read from iob
    m_robot->readJointAngles(m_q.data.get_buffer());
    m_q.tm = tm;
    m_robot->readJointVelocities(m_dq.data.get_buffer());
    m_dq.tm = tm;
    m_robot->readJointTorques(m_tau.data.get_buffer());
    m_tau.tm = tm;
    m_robot->readJointCommandTorques(m_ctau.data.get_buffer());
    m_ctau.tm = tm;
    m_robot->readPDControllerTorques(m_pdtau.data.get_buffer());
    m_pdtau.tm = tm;
    for (unsigned int i = 0; i < m_rate.size(); i++) {
        double rate[3];
        m_robot->readGyroSensor(i, rate);
        m_rate[i].data.avx = rate[0];
        m_rate[i].data.avy = rate[1];
        m_rate[i].data.avz = rate[2];
        m_rate[i].tm       = tm;
    }

    for (unsigned int i = 0; i < m_acc.size(); i++) {
        double acc[3];
        m_robot->readAccelerometer(i, acc);
        m_acc[i].data.ax = acc[0];
        m_acc[i].data.ay = acc[1];
        m_acc[i].data.az = acc[2];
        m_acc[i].tm      = tm;
    }

    for (unsigned int i = 0; i < m_force.size(); i++) {
        m_robot->readForceSensor(i, m_force[i].data.get_buffer());
        m_force[i].tm = tm;
    }

    for (unsigned int i = 0; i < m_servoState.data.length(); i++) {
        size_t len = m_robot->lengthOfExtraServoState(i) + 1;
        m_servoState.data[i].length(len);
        int status = 0, v;
        v          = m_robot->readCalibState(i);
        status |= v << OpenHRP::RobotHardware2Service::CALIB_STATE_SHIFT;
        v = m_robot->readPowerState(i);
        status |= v << OpenHRP::RobotHardware2Service::POWER_STATE_SHIFT;
        v = m_robot->readServoState(i);
        status |= v << OpenHRP::RobotHardware2Service::SERVO_STATE_SHIFT;
        v = m_robot->readServoAlarm(i);
        status |= v << OpenHRP::RobotHardware2Service::SERVO_ALARM_SHIFT;
        v = m_robot->readDriverTemperature(i);
        status |= v << OpenHRP::RobotHardware2Service::DRIVER_TEMP_SHIFT;
        m_servoState.data[i][0] = status;
        m_robot->readExtraServoState(i, (int *)(m_servoState.data[i].get_buffer() + 1));
    }
    m_servoState.tm = tm;

    getStatus2(m_rstate2.data);
    m_rstate2.tm = tm;

    m_robot->oneStep();

    m_qOut.write();
    m_dqOut.write();
    m_tauOut.write();
    m_ctauOut.write();
    m_pdtauOut.write();
    m_servoStateOut.write();
    for (unsigned int i = 0; i < m_rateOut.size(); i++) {
        m_rateOut[i]->write();
    }
    for (unsigned int i = 0; i < m_accOut.size(); i++) {
        m_accOut[i]->write();
    }
    for (unsigned int i = 0; i < m_forceOut.size(); i++) {
        m_forceOut[i]->write();
    }
    m_rstate2Out.write();

    return RTC::RTC_OK;
}

template <class T> void getStatus(std::shared_ptr<robot> robot, T &rstate) {
    rstate.angle.length(robot->numJoints());
    robot->readJointAngles(rstate.angle.get_buffer());

    rstate.command.length(robot->numJoints());
    robot->readJointCommands(rstate.command.get_buffer());

    rstate.torque.length(robot->numJoints());
    if (!robot->readJointTorques(rstate.torque.get_buffer())) {
        for (unsigned int i = 0; i < rstate.torque.length(); i++) {
            rstate.torque[i] = 0.0;
        }
    }

    rstate.servoState.length(robot->numJoints());
    int v, status;
    for (unsigned int i = 0; i < rstate.servoState.length(); ++i) {
        size_t len = robot->lengthOfExtraServoState(i) + 1;
        rstate.servoState[i].length(len);
        status = 0;
        v      = robot->readCalibState(i);
        status |= v << OpenHRP::RobotHardware2Service::CALIB_STATE_SHIFT;
        v = robot->readPowerState(i);
        status |= v << OpenHRP::RobotHardware2Service::POWER_STATE_SHIFT;
        v = robot->readServoState(i);
        status |= v << OpenHRP::RobotHardware2Service::SERVO_STATE_SHIFT;
        v = robot->readServoAlarm(i);
        status |= v << OpenHRP::RobotHardware2Service::SERVO_ALARM_SHIFT;
        v = robot->readDriverTemperature(i);
        status |= v << OpenHRP::RobotHardware2Service::DRIVER_TEMP_SHIFT;
        rstate.servoState[i][0] = status;
        robot->readExtraServoState(i, (int *)(rstate.servoState[i].get_buffer() + 1));
    }

    rstate.rateGyro.length(robot->numGyroSensors());
    for (unsigned int i = 0; i < rstate.rateGyro.length(); i++) {
        rstate.rateGyro[i].length(3);
        robot->readGyroSensor(i, rstate.rateGyro[i].get_buffer());
    }

    rstate.accel.length(robot->numAccSensors());
    for (unsigned int i = 0; i < rstate.accel.length(); i++) {
        rstate.accel[i].length(3);
        robot->readAccelerometer(i, rstate.accel[i].get_buffer());
    }

    rstate.force.length(robot->numForceSensors());
    for (unsigned int i = 0; i < rstate.force.length(); i++) {
        rstate.force[i].length(6);
        robot->readForceSensor(i, rstate.force[i].get_buffer());
    }

    robot->readPowerStatus(rstate.voltage, rstate.current);
}

void RobotHardware2::getStatus2(OpenHRP::RobotHardware2Service::RobotState2 &rstate2) {
    getStatus(m_robot, rstate2);
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
    rstate2.batteries.length(m_robot->numBatteries());
    for (unsigned int i = 0; i < rstate2.batteries.length(); i++) {
        m_robot->readBatteryState(i, rstate2.batteries[i].voltage, rstate2.batteries[i].current, rstate2.batteries[i].soc);
    }
    rstate2.temperature.length(m_robot->numThermometers());
    for (unsigned int i = 0; i < rstate2.temperature.length(); i++) {
        m_robot->readThermometer(i, rstate2.temperature[i]);
    }
#endif
}

/*
RTC::ReturnCode_t RobotHardware::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

bool RobotHardware2::getProperty(const std::string &key, std::string &ret) {
    if (this->getProperties().hasKey(key.c_str())) {
        ret = std::string(this->getProperties()[key.c_str()]);
    } else if (this->m_pManager->getConfig().hasKey(key.c_str())) {
        ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
    } else {
        return false;
    }
    RTC_INFO_STREAM(key << ": " << ret);
    return true;
}


extern "C" {

void RobotHardware2Init(RTC::Manager *manager) {
    RTC::Properties profile(robothardware2_spec);
    manager->registerFactory(profile, RTC::Create<RobotHardware2>, RTC::Delete<RobotHardware2>);
}
};
