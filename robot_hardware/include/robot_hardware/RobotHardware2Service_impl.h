// -*-C++-*-

#ifndef ROBOTHARDWARE2SERVICE_IMPL_H
#define ROBOTHARDWARE2SERVICE_IMPL_H

#include <boost/intrusive_ptr.hpp>
#include <robot_hardware/idl/RobotHardware2Service.hh>
#include <robot_hardware/robot.h>

class RobotHardware2Service_impl : public virtual POA_OpenHRP::RobotHardware2Service, public virtual PortableServer::RefCountServantBase {
  public:
    RobotHardware2Service_impl();
    virtual ~RobotHardware2Service_impl();

    void getStatus(OpenHRP::RobotHardware2Service::RobotState_out rs);
    void getStatus2(OpenHRP::RobotHardware2Service::RobotState2_out rs);

    CORBA::Boolean power(const char *jname, OpenHRP::RobotHardware2Service::SwitchStatus ss);
    CORBA::Boolean servo(const char *jname, OpenHRP::RobotHardware2Service::SwitchStatus ss);
    void setServoGainPercentage(const char *jname, double limit);
    void setServoPGainPercentage(const char *jname, double limit);
    void setServoDGainPercentage(const char *jname, double limit);
    void setServoPGainPercentageWithTime(const char *jname, double limit, double time);
    void setServoDGainPercentageWithTime(const char *jname, double limit, double time);
    void setServoTorqueGainPercentage(const char *jname, double limit);
    void setServoErrorLimit(const char *jname, double limit);
    void calibrateInertiaSensor();
    void removeForceSensorOffset();
    void initializeJointAngle(const char *name, const char *option);
    CORBA::Boolean addJointGroup(const char *gname, const OpenHRP::RobotHardware2Service::StrSequence &jnames);
    CORBA::Boolean readDigitalInput(::OpenHRP::RobotHardware2Service::OctSequence_out din);
    CORBA::Long lengthDigitalInput();
    CORBA::Boolean writeDigitalOutput(const ::OpenHRP::RobotHardware2Service::OctSequence &dout);
    CORBA::Boolean writeDigitalOutputWithMask(const ::OpenHRP::RobotHardware2Service::OctSequence &dout,
                                              const ::OpenHRP::RobotHardware2Service::OctSequence &mask);
    CORBA::Long lengthDigitalOutput();
    CORBA::Boolean readDigitalOutput(::OpenHRP::RobotHardware2Service::OctSequence_out dout);
    CORBA::Boolean setJointInertia(const char *name, ::CORBA::Double mn);
    void setJointInertias(const ::OpenHRP::RobotHardware2Service::DblSequence &mns);
    void enableDisturbanceObserver();
    void disableDisturbanceObserver();
    void setDisturbanceObserverGain(::CORBA::Double gain);
    void setJointControlMode(const char *jname, OpenHRP::RobotHardware2Service::JointControlMode jcm);
    //
    // void setRobot(boost::shared_ptr<robot>& i_robot) { m_robot = i_robot; }
    void setRobot(std::shared_ptr<robot> &i_robot) { m_robot = i_robot; }

  private:
    std::shared_ptr<robot> m_robot;
};
#endif
