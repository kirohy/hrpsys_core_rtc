#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/JointPath>
#include <rtm/CorbaNaming.h>
#include <sequence_player/SequencePlayer2.h>

static const char *sequenceplayer2_spec[] = {"implementation_id", "SequencePlayer2", "type_name", "SequencePlayer2",
                                             "description", "sequence player component", "version", "1.0.0", "vendor",
                                             "JSK", "category", "example", "activity_type", "DataFlowComponent",
                                             "max_instance", "10", "language", "C++", "lang_type", "compile",
                                             // Configuration variables
                                             "conf.default.debugLevel", "0", "conf.default.fixedLink", "",

                                             ""};

SequencePlayer2::SequencePlayer2(RTC::Manager *manager)
    : RTC::DataFlowComponentBase(manager), m_qInitIn("qInit", m_qInit), m_basePosInitIn("basePosInit", m_basePosInit),
      m_baseRpyInitIn("baseRpyInit", m_baseRpyInit), m_zmpRefInitIn("zmpRefInit", m_zmpRefInit),
      m_qRefOut("qRef", m_qRef), m_tqRefOut("tqRef", m_tqRef), m_zmpRefOut("zmpRef", m_zmpRef),
      m_accRefOut("accRef", m_accRef), m_basePosOut("basePos", m_basePos), m_baseRpyOut("baseRpy", m_baseRpy),
      m_optionalDataOut("optionalData", m_optionalData), m_SequencePlayer2ServicePort("SequencePlayer2Service"),
      m_robot(cnoid::BodyPtr()), m_debugLevel(0), m_error_pos(0.0001), m_error_rot(0.001), m_iteration(50), dummy(0) {
    sem_init(&m_waitSem, 0, 0);
    m_service0.player(this);
    m_clearFlag = false;
    m_waitFlag  = false;
}

SequencePlayer2::~SequencePlayer2() {}


RTC::ReturnCode_t SequencePlayer2::onInitialize() {
    RTC_INFO_STREAM("onInitialize()");
    // Set InPort buffers
    addInPort("qInit", m_qInitIn);
    addInPort("basePosInit", m_basePosInitIn);
    addInPort("baseRpyInit", m_baseRpyInitIn);
    addInPort("zmpRefInit", m_zmpRefInitIn);

    // Set OutPort buffer
    addOutPort("qRef", m_qRefOut);
    addOutPort("tqRef", m_tqRefOut);
    addOutPort("zmpRef", m_zmpRefOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("basePos", m_basePosOut);
    addOutPort("baseRpy", m_baseRpyOut);
    addOutPort("optionalData", m_optionalDataOut);

    // Set service provider to Ports
    m_SequencePlayer2ServicePort.registerProvider("service0", "SequencePlayer2Service", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_SequencePlayer2ServicePort);

    // Bind variables and configuration variable
    bindParameter("debugLevel", m_debugLevel, "0");
    bindParameter("fixedLink", m_fixedLink, "");

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

    unsigned int dof = m_robot->numJoints();


    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    cnoid::DeviceList<cnoid::ForceSensor> sensors(m_robot->devices());
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
    m_wrenchesOut.resize(nforce);
    for (unsigned int i = 0; i < nforce; i++) {
        m_wrenchesOut[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq>>(
            std::string(fsensor_names[i] + "Ref").c_str(), m_wrenches[i]);
        m_wrenches[i].data.length(6);
        registerOutPort(std::string(fsensor_names[i] + "Ref").c_str(), *m_wrenchesOut[i]);
    }

    std::string optional_data_dim_str;
    if (prop.hasKey("seq_optional_data_dim")) {
        m_optional_data_dim = std::stoi(std::string(prop["seq_optional_data_dim"]));
    } else {
        m_optional_data_dim = 1;
    }
    RTC_INFO_STREAM("seq_optional_data_dim = " << m_optional_data_dim);

    m_seq = std::make_shared<seqplay>(dof, m_dt, nforce, m_optional_data_dim);

    m_qInit.data.length(dof);
    for (unsigned int i = 0; i < dof; i++)
        m_qInit.data[i] = 0.0;
    cnoid::LinkPtr root  = m_robot->rootLink();
    m_basePosInit.data.x = root->p()[0];
    m_basePosInit.data.y = root->p()[1];
    m_basePosInit.data.z = root->p()[2];
    cnoid::Vector3 rpy   = cnoid::rpyFromRot(root->R());
    m_baseRpyInit.data.r = rpy[0];
    m_baseRpyInit.data.p = rpy[1];
    m_baseRpyInit.data.y = rpy[2];
    m_zmpRefInit.data.x  = 0;
    m_zmpRefInit.data.y  = 0;
    m_zmpRefInit.data.z  = 0;

    // allocate memory for outPorts
    m_qRef.data.length(dof);
    m_tqRef.data.length(dof);
    m_optionalData.data.length(m_optional_data_dim);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t SequencePlayer2::onFinalize() {
    if (m_debugLevel > 0) { RTC_WARN_STREAM(__PRETTY_FUNCTION__); }
    return RTC::RTC_OK;
}

RTC::ReturnCode_t SequencePlayer2::onActivated(RTC::UniqueId ec_id) {
    RTC_INFO_STREAM("onActivated(" << ec_id << ")");

    return RTC::RTC_OK;
}

RTC::ReturnCode_t SequencePlayer2::onExecute(RTC::UniqueId ec_id) {
    // RTC_INFO_STREAM("onExecute(" << ec_id << ")");
    static int loop = 0;
    loop++;
    if (m_debugLevel > 0 && loop % 1000 == 0) { RTC_WARN_STREAM(__PRETTY_FUNCTION__ << "(" << ec_id << ")"); }
    if (m_qInitIn.isNew()) m_qInitIn.read();
    if (m_basePosInitIn.isNew()) m_basePosInitIn.read();
    if (m_baseRpyInitIn.isNew()) m_baseRpyInitIn.read();
    if (m_zmpRefInitIn.isNew()) m_zmpRefInitIn.read();

    if (m_gname != "" && m_seq->isEmpty(m_gname.c_str())) {
        if (m_waitFlag) {
            m_gname    = "";
            m_waitFlag = false;
            sem_post(&m_waitSem);
        }
    }
    if (m_seq->isEmpty()) {
        m_clearFlag = false;
        if (m_waitFlag) {
            m_waitFlag = false;
            sem_post(&m_waitSem);
        }
    } else {
        std::lock_guard<std::mutex> guard(m_mutex);

        double zmp[3], acc[3], pos[3], rpy[3], wrenches[6 * m_wrenches.size()];
        m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches,
                   m_optionalData.data.get_buffer());
        m_zmpRef.data.x  = zmp[0];
        m_zmpRef.data.y  = zmp[1];
        m_zmpRef.data.z  = zmp[2];
        m_accRef.data.ax = acc[0];
        m_accRef.data.ay = acc[1];
        m_accRef.data.az = acc[2];

        if (m_fixedLink != "") {
            for (int i = 0; i < m_robot->numJoints(); i++) {
                m_robot->joint(i)->q() = m_qRef.data[i];
            }
            for (int i = 0; i < 3; i++) {
                m_robot->rootLink()->p()[i] = pos[i];
            }
            m_robot->rootLink()->setRotation(cnoid::rotFromRpy(rpy[0], rpy[1], rpy[2]));
            m_robot->calcForwardKinematics();
            cnoid::LinkPtr root = m_robot->rootLink();
            cnoid::Vector3 rootP;
            cnoid::Matrix3 rootR;
            if (m_timeToStartPlaying > 0) {
                m_timeToStartPlaying -= m_dt;
                cnoid::LinkPtr fixed       = m_robot->link(m_fixedLink);
                cnoid::Matrix3 fixed2rootR = fixed->R().transpose() * root->R();
                cnoid::Vector3 fixed2rootP = fixed->R().transpose() * (root->p() - fixed->p());
                rootR                      = m_fixedR * fixed2rootR;
                rootP                      = m_fixedR * fixed2rootP + m_fixedP;
            } else {
                rootR = m_offsetR * m_robot->rootLink()->R();
                rootP = m_offsetR * m_robot->rootLink()->p() + m_offsetP;
            }
            cnoid::Vector3 rootRpy = cnoid::rpyFromRot(rootR);
            pos[0]                 = rootP[0];
            pos[1]                 = rootP[1];
            pos[2]                 = rootP[2];
            rpy[0]                 = rootRpy[0];
            rpy[1]                 = rootRpy[1];
            rpy[2]                 = rootRpy[2];
        }
        m_basePos.data.x = pos[0];
        m_basePos.data.y = pos[1];
        m_basePos.data.z = pos[2];
        m_baseRpy.data.r = rpy[0];
        m_baseRpy.data.p = rpy[1];
        m_baseRpy.data.y = rpy[2];
        size_t force_i   = 0;
        for (size_t i = 0; i < m_wrenches.size(); i++) {
            m_wrenches[i].data[0] = wrenches[force_i++];
            m_wrenches[i].data[1] = wrenches[force_i++];
            m_wrenches[i].data[2] = wrenches[force_i++];
            m_wrenches[i].data[3] = wrenches[force_i++];
            m_wrenches[i].data[4] = wrenches[force_i++];
            m_wrenches[i].data[5] = wrenches[force_i++];
        }
        m_qRef.tm = m_qInit.tm;
        m_qRefOut.write();
        m_tqRefOut.write();
        m_zmpRefOut.write();
        m_accRefOut.write();
        m_basePosOut.write();
        m_baseRpyOut.write();
        m_optionalDataOut.write();
        for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
            m_wrenchesOut[i]->write();
        }

        if (m_clearFlag) { m_seq->clear(0.001); }
    }
    return RTC::RTC_OK;
}

void SequencePlayer2::setClearFlag() {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    m_clearFlag = true;
}

void SequencePlayer2::waitInterpolation() {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    m_waitFlag = true;
    sem_wait(&m_waitSem);
}

bool SequencePlayer2::waitInterpolationOfGroup(const char *gname) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    m_gname    = gname;
    m_waitFlag = true;
    sem_wait(&m_waitSem);
    return true;
}


bool SequencePlayer2::setJointAngle(short id, double angle, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return false;
    if (id < 0 || id >= m_robot->numJoints()) {
        RTC_INFO_STREAM("[setJointAngle] Invalid jointId " << id);
        return false;
    }
    std::vector<double> q(m_robot->numJoints());
    m_seq->getJointAngles(q.data());
    q[id] = angle;
    for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
        cnoid::LinkPtr j = m_robot->joint(i);
        if (j) j->q() = q[i];
    }
    m_robot->calcForwardKinematics();
    cnoid::Vector3 absZmp = m_robot->calcCenterOfMass();
    absZmp[2]             = 0;
    cnoid::LinkPtr root   = m_robot->rootLink();
    cnoid::Vector3 relZmp = root->R().transpose() * (absZmp - root->p());
    m_seq->setJointAngles(q.data(), tm);
    m_seq->setZmp(relZmp.data(), tm);
    return true;
}

bool SequencePlayer2::setJointAngles(const double *angles, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return false;
    for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
        cnoid::LinkPtr j = m_robot->joint(i);
        if (j) j->q() = angles[i];
    }
    m_robot->calcForwardKinematics();
    cnoid::Vector3 absZmp = m_robot->calcCenterOfMass();
    absZmp[2]             = 0;
    cnoid::LinkPtr root   = m_robot->rootLink();
    cnoid::Vector3 relZmp = root->R().transpose() * (absZmp - root->p());
    std::vector<const double *> v_poss;
    std::vector<double> v_tms;
    v_poss.push_back(angles);
    v_tms.push_back(tm);
    m_seq->setJointAnglesSequence(v_poss, v_tms);
    m_seq->setZmp(relZmp.data(), tm);
    return true;
}

bool SequencePlayer2::setJointAngles(const double *angles, const bool *mask, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!setInitialState()) return false;

    double pose[m_robot->numJoints()];
    for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
        pose[i] = mask[i] ? angles[i] : m_qInit.data[i];
    }
    m_seq->setJointAngles(pose, tm);
    return true;
}

bool SequencePlayer2::setJointAnglesSequence(const OpenHRP::dSequenceSequence angless, const OpenHRP::bSequence &mask,
                                             const OpenHRP::dSequence &times) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!setInitialState()) return false;

    bool tmp_mask[robot()->numJoints()];
    if (mask.length() != robot()->numJoints()) {
        for (unsigned int i = 0; i < robot()->numJoints(); i++)
            tmp_mask[i] = true;
    } else {
        for (unsigned int i = 0; i < robot()->numJoints(); i++)
            tmp_mask[i] = mask.get_buffer()[i];
    }
    int len = angless.length();
    std::vector<const double *> v_poss;
    std::vector<double> v_tms;
    for (unsigned int i = 0; i < angless.length(); i++)
        v_poss.push_back(angless[i].get_buffer());
    for (unsigned int i = 0; i < times.length(); i++)
        v_tms.push_back(times[i]);
    return m_seq->setJointAnglesSequence(v_poss, v_tms);
}

bool SequencePlayer2::clearJointAngles() {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!setInitialState()) return false;

    return m_seq->clearJointAngles();
}

bool SequencePlayer2::setJointAnglesSequenceOfGroup(const char *gname, const OpenHRP::dSequenceSequence angless,
                                                    const OpenHRP::dSequence &times) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;

    std::vector<const double *> v_poss;
    std::vector<double> v_tms;
    for (unsigned int i = 0; i < angless.length(); i++)
        v_poss.push_back(angless[i].get_buffer());
    for (unsigned int i = 0; i < times.length(); i++)
        v_tms.push_back(times[i]);
    return m_seq->setJointAnglesSequenceOfGroup(gname, v_poss, v_tms, angless.length() > 0 ? angless[0].length() : 0);
}

bool SequencePlayer2::clearJointAnglesOfGroup(const char *gname) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;

    return m_seq->clearJointAnglesOfGroup(gname);
}

bool SequencePlayer2::setJointAnglesSequenceFull(
    const OpenHRP::dSequenceSequence i_jvss, const OpenHRP::dSequenceSequence i_vels,
    const OpenHRP::dSequenceSequence i_torques, const OpenHRP::dSequenceSequence i_poss,
    const OpenHRP::dSequenceSequence i_rpys, const OpenHRP::dSequenceSequence i_accs,
    const OpenHRP::dSequenceSequence i_zmps, const OpenHRP::dSequenceSequence i_wrenches,
    const OpenHRP::dSequenceSequence i_optionals, const dSequence i_tms) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!setInitialState()) return false;

    int len = i_jvss.length();
    std::vector<const double *> v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals;
    std::vector<double> v_tms;
    for (unsigned int i = 0; i < i_jvss.length(); i++)
        v_jvss.push_back(i_jvss[i].get_buffer());
    for (unsigned int i = 0; i < i_vels.length(); i++)
        v_vels.push_back(i_vels[i].get_buffer());
    for (unsigned int i = 0; i < i_torques.length(); i++)
        v_torques.push_back(i_torques[i].get_buffer());
    for (unsigned int i = 0; i < i_poss.length(); i++)
        v_poss.push_back(i_poss[i].get_buffer());
    for (unsigned int i = 0; i < i_rpys.length(); i++)
        v_rpys.push_back(i_rpys[i].get_buffer());
    for (unsigned int i = 0; i < i_accs.length(); i++)
        v_accs.push_back(i_accs[i].get_buffer());
    for (unsigned int i = 0; i < i_zmps.length(); i++)
        v_zmps.push_back(i_zmps[i].get_buffer());
    for (unsigned int i = 0; i < i_wrenches.length(); i++)
        v_wrenches.push_back(i_wrenches[i].get_buffer());
    for (unsigned int i = 0; i < i_optionals.length(); i++)
        v_optionals.push_back(i_optionals[i].get_buffer());
    for (unsigned int i = 0; i < i_tms.length(); i++)
        v_tms.push_back(i_tms[i]);
    return m_seq->setJointAnglesSequenceFull(v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches,
                                             v_optionals, v_tms);
}

bool SequencePlayer2::setBasePos(const double *pos, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return false;
    m_seq->setBasePos(pos, tm);
    return true;
}

bool SequencePlayer2::setBaseRpy(const double *rpy, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return false;
    m_seq->setBaseRpy(rpy, tm);
    return true;
}

bool SequencePlayer2::setZmp(const double *zmp, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return false;
    m_seq->setZmp(zmp, tm);
    return true;
}

bool SequencePlayer2::setWrenches(const double *wrenches, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return false;
    m_seq->setWrenches(wrenches, tm);
    return true;
}

bool SequencePlayer2::setTargetPose(const char *gname, const double *xyz, const double *rpy, double tm,
                                    const char *frame_name) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!setInitialState()) return false;
    // setup
    std::vector<int> indices;
    std::vector<double> start_av, end_av;
    std::vector<std::vector<double>> avs;
    if (!m_seq->getJointGroup(gname, indices)) {
        RTC_INFO_STREAM("[setTargetPose] Could not find joint group " << gname);
        return false;
    }
    start_av.resize(indices.size());
    end_av.resize(indices.size());

    if (m_robot->joint(indices[0])->isRoot()) {
        RTC_INFO_STREAM("[setTargetPose] " << m_robot->joint(indices[0])->name() << " does not have parent");
        return false;
    }
    std::string base_parent_name = m_robot->joint(indices[0])->parent()->name();
    std::string target_name      = m_robot->joint(indices[indices.size() - 1])->name();
    // prepare joint path
    std::shared_ptr<cnoid::JointPath> manip =
        cnoid::JointPath::getCustomPath(m_robot->link(base_parent_name), m_robot->link(target_name));

    // calc fk
    for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
        cnoid::LinkPtr j = m_robot->joint(i);
        if (j) j->q() = m_qRef.data.get_buffer()[i];
    }
    m_robot->calcForwardKinematics();
    for (unsigned int i = 0; i < manip->numJoints(); i++) {
        start_av[i] = manip->joint(i)->q();
    }

    // xyz and rpy are relateive to root link, where as pos and rotatoin of manip->calcInverseKinematics are relative to
    // base link

    // ik params
    cnoid::Vector3 start_p(m_robot->link(target_name)->p());
    cnoid::Matrix3 start_R(m_robot->link(target_name)->R());
    cnoid::Vector3 end_p(xyz[0], xyz[1], xyz[2]);
    cnoid::Matrix3 end_R = cnoid::rotFromRpy(rpy[0], rpy[1], rpy[2]) * m_robot->link(target_name)->R().transpose();

    // change start and end must be relative to the frame_name
    if ((frame_name != NULL) && (!m_robot->link(frame_name))) {
        RTC_INFO_STREAM("[setTargetPose] Could not find frame_name " << frame_name);
        return false;
    } else if (frame_name != NULL) {
        cnoid::Vector3 frame_p(m_robot->link(frame_name)->p());
        cnoid::Matrix3 frame_R(m_robot->link(frame_name)->R());
        end_p = frame_R * end_p + frame_p;
        end_R = frame_R * end_R;
    }
    manip->setNumericalIkMaxIkError(m_error_pos); // TODO
    manip->setNumericalIkMaxIterations(m_iteration);
    RTC_INFO_STREAM("[setTargetPose] Solveing IK with frame " << (frame_name ? frame_name : "world_frame") << ", Error "
                                                              << m_error_pos << m_error_rot << ", Iteration "
                                                              << m_iteration);
    RTC_INFO_STREAM("                Start\n" << start_p << "\n" << start_R << std::endl);
    RTC_INFO_STREAM("                End\n" << end_p << "\n" << end_R << std::endl);

    int len = std::max(((start_p - end_p).norm() / 0.02),                                    // 2cm
                       ((cnoid::omegaFromRot(start_R.transpose() * end_R).norm()) / 0.025)); // 2 deg
    len     = std::max(len, 1);

    std::vector<const double *> v_pos;
    std::vector<double> v_tm;
    v_pos.resize(len);
    v_tm.resize(len);

    // do loop
    for (int i = 0; i < len; i++) {
        double a             = (1 + i) / (double)len;
        cnoid::Vector3 p     = (1 - a) * start_p + a * end_p;
        cnoid::Vector3 omega = cnoid::omegaFromRot(start_R.transpose() * end_R);
        cnoid::Matrix3 R = start_R * Eigen::AngleAxisd(a * omega.norm(), omega.isZero() ? omega : omega.normalized());
        cnoid::Isometry3 target;
        target.translation() = p;
        target.linear()      = R;
        bool ret             = manip->calcInverseKinematics(target);

        if (m_debugLevel > 0) {
            RTC_INFO_STREAM("target pos/rot : " << i << "/" << a << " : " << p[0] << " " << p[1] << " " << p[2] << ","
                                                << omega[0] << " " << omega[1] << " " << omega[2]);
        }
        if (!ret) {
            RTC_INFO_STREAM("[setTargetPose] IK failed");
            return false;
        }
        v_pos[i] = (const double *)malloc(sizeof(double) * manip->numJoints());
        for (unsigned int j = 0; j < manip->numJoints(); j++) {
            ((double *)v_pos[i])[j] = manip->joint(j)->q();
        }
        v_tm[i] = tm / len;
    }

    if (m_debugLevel > 0) {
        for (int i = 0; i < len; i++) {
            std::cerr << "[" << m_profile.instance_name << "] " << v_tm[i] << ":";
            for (int j = 0; j < start_av.size(); j++) {
                std::cerr << v_pos[i][j] << " ";
            }
            std::cerr << std::endl;
        }
    }

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false; // reset sequencer
    bool ret =
        m_seq->playPatternOfGroup(gname, v_pos, v_tm, m_qInit.data.get_buffer(), v_pos.size() > 0 ? indices.size() : 0);

    // clean up memory, need to improve
    for (int i = 0; i < len; i++) {
        free((double *)v_pos[i]);
    }

    return ret;
}

void SequencePlayer2::loadPattern(const char *basename, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (setInitialState()) {
        if (m_fixedLink != "") {
            cnoid::LinkPtr l = m_robot->link(m_fixedLink);
            if (!l) {
                RTC_WARN_STREAM(__PRETTY_FUNCTION__ << "can't find a fixed link(" << m_fixedLink << ")");
                m_fixedLink = "";
                return;
            }
            m_robot->calcForwardKinematics(); // this is not called by setinitialstate()
            m_fixedP = l->p();
            m_fixedR = l->R();

            std::string pos = std::string(basename) + ".pos";
            std::string wst = std::string(basename) + ".waist";
            std::ifstream ifspos(pos.c_str());
            std::ifstream ifswst(wst.c_str());
            if (!ifspos.is_open() || !ifswst.is_open()) {
                RTC_WARN_STREAM(__PRETTY_FUNCTION__ << "can't open " << pos << " or " << wst << ")");
                m_fixedLink = "";
                return;
            }
            double time;
            ifspos >> time;
            for (int i = 0; i < m_robot->numJoints(); i++) {
                ifspos >> m_robot->joint(i)->q();
            }
            ifswst >> time;
            for (int i = 0; i < 3; i++)
                ifswst >> m_robot->rootLink()->p()[i];
            cnoid::Vector3 rpy;
            for (int i = 0; i < 3; i++)
                ifswst >> rpy[i];
            m_robot->rootLink()->setRotation(cnoid::rotFromRpy(rpy));
            m_robot->calcForwardKinematics();

            m_offsetR            = m_fixedR * l->R().transpose();
            m_offsetP            = m_fixedP - m_offsetR * l->p();
            m_timeToStartPlaying = tm;
        }
        m_seq->loadPattern(basename, tm);
    }
}

bool SequencePlayer2::setInitialState(double tm) {
    if (m_debugLevel > 0) {
        RTC_INFO_STREAM(__PRETTY_FUNCTION__ << "m_seq-isEmpty() " << m_seq->isEmpty() << ", m_Init.data.length() "
                                            << m_qInit.data.length());
    }
    if (!m_seq->isEmpty()) return true;

    if (m_qInit.data.length() == 0) {
        RTC_INFO_STREAM("can't determine initial posture");
        return false;
    } else {
        m_seq->setJointAngles(m_qInit.data.get_buffer(), tm);
        for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
            cnoid::LinkPtr l = m_robot->joint(i);
            l->q()           = m_qInit.data[i];
            m_qRef.data[i]   = m_qInit.data[i]; // update m_qRef for setTargetPose()
        }

        cnoid::LinkPtr root = m_robot->rootLink();

        root->setTranslation(cnoid::Vector3(m_basePosInit.data.x, m_basePosInit.data.y, m_basePosInit.data.z));
        m_seq->setBasePos(root->p().data(), tm);

        double rpy[] = {m_baseRpyInit.data.r, m_baseRpyInit.data.p, m_baseRpyInit.data.y};
        m_seq->setBaseRpy(rpy, tm);
        root->setRotation(cnoid::rotFromRpy(rpy[0], rpy[1], rpy[2]));

        double zmp[] = {m_zmpRefInit.data.x, m_zmpRefInit.data.y, m_zmpRefInit.data.z};
        m_seq->setZmp(zmp, tm);
        double zero[] = {0, 0, 0};
        m_seq->setBaseAcc(zero, tm);
        return true;
    }
}

void SequencePlayer2::playPattern(const dSequenceSequence &pos, const dSequenceSequence &rpy,
                                  const dSequenceSequence &zmp, const dSequence &tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return;

    std::vector<const double *> v_pos, v_rpy, v_zmp;
    std::vector<double> v_tm;
    for (unsigned int i = 0; i < pos.length(); i++)
        v_pos.push_back(pos[i].get_buffer());
    for (unsigned int i = 0; i < rpy.length(); i++)
        v_rpy.push_back(rpy[i].get_buffer());
    for (unsigned int i = 0; i < zmp.length(); i++)
        v_zmp.push_back(zmp[i].get_buffer());
    for (unsigned int i = 0; i < tm.length(); i++)
        v_tm.push_back(tm[i]);
    return m_seq->playPattern(v_pos, v_rpy, v_zmp, v_tm, m_qInit.data.get_buffer(),
                              pos.length() > 0 ? pos[0].length() : 0);
}

bool SequencePlayer2::setInterpolationMode(OpenHRP::SequencePlayer2Service::interpolationMode i_mode_) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    interpolator::interpolation_mode new_mode;
    if (i_mode_ == OpenHRP::SequencePlayer2Service::LINEAR) {
        new_mode = interpolator::LINEAR;
    } else if (i_mode_ == OpenHRP::SequencePlayer2Service::HOFFARBIB) {
        new_mode = interpolator::HOFFARBIB;
    } else {
        return false;
    }
    return m_seq->setInterpolationMode(new_mode);
}

bool SequencePlayer2::addJointGroup(const char *gname, const OpenHRP::SequencePlayer2Service::StrSequence &jnames) {
    RTC_INFO_STREAM("[addJointGroup] group name = " << gname);
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    if (!waitInterpolationOfGroup(gname)) return false;

    std::lock_guard<std::mutex> guard(m_mutex);
    std::vector<int> indices;
    for (size_t i = 0; i < jnames.length(); i++) {
        cnoid::LinkPtr l = m_robot->link(std::string(jnames[i]));
        if (l) {
            indices.push_back(l->jointId());
        } else {
            RTC_INFO_STREAM("[addJointGroup] link name " << jnames[i] << "is not found");
            return false;
        }
    }
    return m_seq->addJointGroup(gname, indices);
}

bool SequencePlayer2::removeJointGroup(const char *gname) {
    RTC_INFO_STREAM("[removeJointGroup] group name = " << gname);
    if (!waitInterpolationOfGroup(gname)) return false;
    bool ret;
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        if (!setInitialState()) return false;
        ret = m_seq->removeJointGroup(gname);
    }
    return ret;
}

bool SequencePlayer2::setJointAnglesOfGroup(const char *gname, const dSequence &jvs, double tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;
    return m_seq->setJointAnglesOfGroup(gname, jvs.get_buffer(), jvs.length(), tm);
}

bool SequencePlayer2::playPatternOfGroup(const char *gname, const dSequenceSequence &pos, const dSequence &tm) {
    if (m_debugLevel > 0) { RTC_INFO_STREAM(__PRETTY_FUNCTION__); }
    std::lock_guard<std::mutex> guard(m_mutex);
    if (!setInitialState()) return false;

    std::vector<const double *> v_pos;
    std::vector<double> v_tm;
    for (unsigned int i = 0; i < pos.length(); i++)
        v_pos.push_back(pos[i].get_buffer());
    for (unsigned int i = 0; i < tm.length(); i++)
        v_tm.push_back(tm[i]);
    return m_seq->playPatternOfGroup(gname, v_pos, v_tm, m_qInit.data.get_buffer(),
                                     pos.length() > 0 ? pos[0].length() : 0);
}

void SequencePlayer2::setMaxIKError(double pos, double rot) {
    m_error_pos = pos;
    m_error_rot = rot;
}

void SequencePlayer2::setMaxIKIteration(short iter) { m_iteration = iter; }

extern "C" {
void SequencePlayer2Init(RTC::Manager *manager) {
    RTC::Properties profile(sequenceplayer2_spec);
    manager->registerFactory(profile, RTC::Create<SequencePlayer2>, RTC::Delete<SequencePlayer2>);
}
};
