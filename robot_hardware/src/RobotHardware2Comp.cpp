#include <iostream>
#include <robot_hardware/RobotHardware2.h>
#include <rtm/Manager.h>
#include <rtm/ModuleManager.h>
#include <string>


void MyModuleInit(RTC::Manager *manager) {
    RobotHardware2Init(manager);
    RTC::RtcBase *comp;

    // Create a component
    comp = manager->createComponent("RobotHardware2");


    // Example
    // The following procedure is examples how handle RT-Components.
    // These should not be in this function.

    // Get the component's object reference
    RTC::RTObject_var rtobj;
    rtobj = RTC::RTObject::_narrow(manager->getPOA()->servant_to_reference(comp));

    // Get the port list of the component
    RTC::PortServiceList *portlist;
    portlist = rtobj->get_ports();

    // getting port profiles
    std::cout << "Number of Ports: ";
    std::cout << portlist->length() << std::endl << std::endl;
    for (CORBA::ULong i(0), n(portlist->length()); i < n; ++i) {
        RTC::PortService_ptr port;
        port = (*portlist)[i];
        std::cout << "Port" << i << " (name): ";
        std::cout << port->get_port_profile()->name << std::endl;

        RTC::PortInterfaceProfileList iflist;
        iflist = port->get_port_profile()->interfaces;
        std::cout << "---interfaces---" << std::endl;
        for (CORBA::ULong i(0), n(iflist.length()); i < n; ++i) {
            std::cout << "I/F name: ";
            std::cout << iflist[i].instance_name << std::endl;
            std::cout << "I/F type: ";
            std::cout << iflist[i].type_name << std::endl;
            const char *pol;
            pol = iflist[i].polarity == 0 ? "PROVIDED" : "REQUIRED";
            std::cout << "Polarity: " << pol << std::endl;
        }
        std::cout << "---properties---" << std::endl;
        NVUtil::dump(port->get_port_profile()->properties);
        std::cout << "----------------" << std::endl << std::endl;
    }

    return;
}

int main(int argc, char **argv) {
    RTC::Manager *manager;
    manager = RTC::Manager::init(argc, argv);

    // Initialize manager
    manager->init(argc, argv);

#ifndef __APPLE__
    try {
        manager->load("libhrpEC.so", "hrpECInit");
    } catch (RTC::ModuleManager::NotFound &ex) {
        std::cerr << ex.name << std::endl;
    } catch (RTC::ModuleManager::Error &ex) {
        std::cerr << ex.reason << std::endl;
        std::cerr << dlerror() << std::endl;
    } catch (...) { std::cerr << "exception in loading libhrpEC.so" << std::endl; }
#endif

    // Set module initialization proceduer
    // This procedure will be invoked in activateManager() function.
    manager->setModuleInitProc(MyModuleInit);

    // Activate manager and register to naming service
    manager->activateManager();

    // run the manager in blocking mode
    // runManager(false) is the default.
    manager->runManager();

    // If you want to run the manager in non-blocking mode, do like this
    // manager->runManager(true);

    return 0;
}
