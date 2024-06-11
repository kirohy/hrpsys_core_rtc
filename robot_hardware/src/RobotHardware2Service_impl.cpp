#include <robot_hardware/RobotHardware2Service_impl.h>
#include <robot_hardware/robot.h>

RobotHardware2Service_impl::RobotHardware2Service_impl() : m_robot(std::shared_ptr<robot>()) {}

RobotHardware2Service_impl::~RobotHardware2Service_impl() {}

#define GetStatus                                                                                                                                              \
                                                                                                                                                               \
    rs->angle.length(m_robot->numJoints());                                                                                                                    \
    m_robot->readJointAngles(rs->angle.get_buffer());                                                                                                          \
                                                                                                                                                               \
    rs->command.length(m_robot->numJoints());                                                                                                                  \
    m_robot->readJointCommands(rs->command.get_buffer());                                                                                                      \
                                                                                                                                                               \
    rs->torque.length(m_robot->numJoints());                                                                                                                   \
    if (!m_robot->readJointTorques(rs->torque.get_buffer())) {                                                                                                 \
        for (unsigned int i = 0; i < rs->torque.length(); i++) {                                                                                               \
            rs->torque[i] = 0.0;                                                                                                                               \
        }                                                                                                                                                      \
    }                                                                                                                                                          \
                                                                                                                                                               \
    rs->servoState.length(m_robot->numJoints());                                                                                                               \
    int v, status;                                                                                                                                             \
    for (unsigned int i = 0; i < rs->servoState.length(); ++i) {                                                                                               \
        size_t len = m_robot->lengthOfExtraServoState(i) + 1;                                                                                                  \
        rs->servoState[i].length(len);                                                                                                                         \
        status = 0;                                                                                                                                            \
        v      = m_robot->readCalibState(i);                                                                                                                   \
        status |= v << OpenHRP::RobotHardware2Service::CALIB_STATE_SHIFT;                                                                                      \
        v = m_robot->readPowerState(i);                                                                                                                        \
        status |= v << OpenHRP::RobotHardware2Service::POWER_STATE_SHIFT;                                                                                      \
        v = m_robot->readServoState(i);                                                                                                                        \
        status |= v << OpenHRP::RobotHardware2Service::SERVO_STATE_SHIFT;                                                                                      \
        v = m_robot->readServoAlarm(i);                                                                                                                        \
        status |= v << OpenHRP::RobotHardware2Service::SERVO_ALARM_SHIFT;                                                                                      \
        v = m_robot->readDriverTemperature(i);                                                                                                                 \
        status |= v << OpenHRP::RobotHardware2Service::DRIVER_TEMP_SHIFT;                                                                                      \
        rs->servoState[i][0] = status;                                                                                                                         \
        m_robot->readExtraServoState(i, (int *)(rs->servoState[i].get_buffer() + 1));                                                                          \
    }                                                                                                                                                          \
                                                                                                                                                               \
    rs->rateGyro.length(m_robot->numGyroSensors());                                                                                                            \
    for (unsigned int i = 0; i < rs->rateGyro.length(); i++) {                                                                                                 \
        rs->rateGyro[i].length(3);                                                                                                                             \
        m_robot->readGyroSensor(i, rs->rateGyro[i].get_buffer());                                                                                              \
    }                                                                                                                                                          \
                                                                                                                                                               \
    rs->accel.length(m_robot->numAccSensors());                                                                                                                \
    for (unsigned int i = 0; i < rs->accel.length(); i++) {                                                                                                    \
        rs->accel[i].length(3);                                                                                                                                \
        m_robot->readAccelerometer(i, rs->accel[i].get_buffer());                                                                                              \
    }                                                                                                                                                          \
                                                                                                                                                               \
    rs->force.length(m_robot->numForceSensors());                                                                                                              \
    for (unsigned int i = 0; i < rs->force.length(); i++) {                                                                                                    \
        rs->force[i].length(6);                                                                                                                                \
        m_robot->readForceSensor(i, rs->force[i].get_buffer());                                                                                                \
    }                                                                                                                                                          \
                                                                                                                                                               \
    m_robot->readPowerStatus(rs->voltage, rs->current);

void RobotHardware2Service_impl::getStatus(OpenHRP::RobotHardware2Service::RobotState_out rs) {
    rs = new OpenHRP::RobotHardware2Service::RobotState();

    GetStatus;
}

void RobotHardware2Service_impl::getStatus2(OpenHRP::RobotHardware2Service::RobotState2_out rs) {
    rs = new OpenHRP::RobotHardware2Service::RobotState2();

    GetStatus;

#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
    rs->batteries.length(m_robot->numBatteries());
    for (unsigned int i = 0; i < rs->batteries.length(); i++) {
        m_robot->readBatteryState(i, rs->batteries[i].voltage, rs->batteries[i].current, rs->batteries[i].soc);
    }
    rs->temperature.length(m_robot->numThermometers());
    for (unsigned int i = 0; i < rs->temperature.length(); i++) {
        m_robot->readThermometer(i, rs->temperature[i]);
    }
#endif
}

CORBA::Boolean RobotHardware2Service_impl::power(const char *jname, OpenHRP::RobotHardware2Service::SwitchStatus ss) {
    return m_robot->power(jname, ss == OpenHRP::RobotHardware2Service::SWITCH_ON);
}

CORBA::Boolean RobotHardware2Service_impl::servo(const char *jname, OpenHRP::RobotHardware2Service::SwitchStatus ss) {
    return m_robot->servo(jname, ss == OpenHRP::RobotHardware2Service::SWITCH_ON);
}

void RobotHardware2Service_impl::calibrateInertiaSensor() { m_robot->startInertiaSensorCalibration(); }

void RobotHardware2Service_impl::removeForceSensorOffset() { m_robot->removeForceSensorOffset(); }

void RobotHardware2Service_impl::initializeJointAngle(const char *name, const char *option) { m_robot->initializeJointAngle(name, option); }

void RobotHardware2Service_impl::setServoGainPercentage(const char *jname, double percentage) { m_robot->setServoGainPercentage(jname, percentage, 0); }

void RobotHardware2Service_impl::setServoPGainPercentage(const char *jname, double percentage) { m_robot->setServoGainPercentage(jname, percentage, 1); }

void RobotHardware2Service_impl::setServoDGainPercentage(const char *jname, double percentage) { m_robot->setServoGainPercentage(jname, percentage, 2); }

void RobotHardware2Service_impl::setServoPGainPercentageWithTime(const char *jname, double percentage, double time) {
    m_robot->setServoGainPercentage(jname, percentage, 1, time);
}

void RobotHardware2Service_impl::setServoDGainPercentageWithTime(const char *jname, double percentage, double time) {
    m_robot->setServoGainPercentage(jname, percentage, 2, time);
}

void RobotHardware2Service_impl::setServoTorqueGainPercentage(const char *jname, double percentage) {
    m_robot->setServoTorqueGainPercentage(jname, percentage);
}

void RobotHardware2Service_impl::setServoErrorLimit(const char *jname, double limit) { m_robot->setServoErrorLimit(jname, limit); }

CORBA::Boolean RobotHardware2Service_impl::addJointGroup(const char *gname, const OpenHRP::RobotHardware2Service::StrSequence &jnames) {
    std::vector<std::string> joints;
    joints.resize(jnames.length());
    for (unsigned int i = 0; i < jnames.length(); i++) {
        joints[i] = jnames[i];
    }
    return m_robot->addJointGroup(gname, joints);
}

CORBA::Boolean RobotHardware2Service_impl::readDigitalInput(::OpenHRP::RobotHardware2Service::OctSequence_out din) {
    din = new ::OpenHRP::RobotHardware2Service::OctSequence();
    din->length(lengthDigitalInput());
    return m_robot->readDigitalInput((char *)(din->get_buffer()));
}

CORBA::Long RobotHardware2Service_impl::lengthDigitalInput() { return m_robot->lengthDigitalInput(); }

CORBA::Boolean RobotHardware2Service_impl::writeDigitalOutput(const ::OpenHRP::RobotHardware2Service::OctSequence &dout) {
    return m_robot->writeDigitalOutput((const char *)(dout.get_buffer()));
}

CORBA::Boolean RobotHardware2Service_impl::writeDigitalOutputWithMask(const ::OpenHRP::RobotHardware2Service::OctSequence &dout,
                                                                      const ::OpenHRP::RobotHardware2Service::OctSequence &mask) {
    return m_robot->writeDigitalOutputWithMask((const char *)(dout.get_buffer()), (const char *)(mask.get_buffer()));
}

CORBA::Long RobotHardware2Service_impl::lengthDigitalOutput() { return m_robot->lengthDigitalOutput(); }

CORBA::Boolean RobotHardware2Service_impl::readDigitalOutput(::OpenHRP::RobotHardware2Service::OctSequence_out dout) {
    dout = new ::OpenHRP::RobotHardware2Service::OctSequence();
    dout->length(lengthDigitalOutput());
    return m_robot->readDigitalOutput((char *)(dout->get_buffer()));
}

CORBA::Boolean RobotHardware2Service_impl::setJointInertia(const char *name, ::CORBA::Double mn) { return m_robot->setJointInertia(name, mn); }

void RobotHardware2Service_impl::setJointInertias(const ::OpenHRP::RobotHardware2Service::DblSequence &mns) { m_robot->setJointInertias(mns.get_buffer()); }


void RobotHardware2Service_impl::enableDisturbanceObserver() { m_robot->enableDisturbanceObserver(); }

void RobotHardware2Service_impl::disableDisturbanceObserver() { m_robot->disableDisturbanceObserver(); }

void RobotHardware2Service_impl::setDisturbanceObserverGain(::CORBA::Double gain) { m_robot->setDisturbanceObserverGain(gain); }

void RobotHardware2Service_impl::setJointControlMode(const char *jname, OpenHRP::RobotHardware2Service::JointControlMode jcm) {
    joint_control_mode mode;
    switch (jcm) {
    case OpenHRP::RobotHardware2Service::FREE:
        mode = JCM_FREE;
        break;
    case OpenHRP::RobotHardware2Service::POSITION:
        mode = JCM_POSITION;
        break;
    case OpenHRP::RobotHardware2Service::TORQUE:
        mode = JCM_TORQUE;
        break;
    case OpenHRP::RobotHardware2Service::VELOCITY:
        mode = JCM_VELOCITY;
        break;
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 4
    case OpenHRP::RobotHardware2Service::POSITION_TORQUE:
        mode = JCM_POSITION_TORQUE;
        break;
#endif
    default:
        return;
    }
    m_robot->setJointControlMode(jname, mode);
}
