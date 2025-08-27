#include <neo_utils/rbdlWrapper.hpp>

neo_utils::RBDLWrapper::RBDLWrapper(bool floatingBase) {
    rbdl_check_api_version(RBDL_API_VERSION);

    // initalize rbdl model from robot_description
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 999999);
    node = std::make_shared<rclcpp::Node>("rbdl_wrapper_" +
                                          std::to_string(dist(gen)));
    const std::string &urdfStr = retrieveUrdf();
    model = std::make_shared<Model>();
    Addons::URDFReadFromString(urdfStr.c_str(), model.get(), floatingBase);

    // setup system members
    q = VectorNd::Zero(model->q_size);
    qdot = VectorNd::Zero(model->qdot_size);

    //      read joint limits
    if (setupJoints(urdfStr.c_str()) < 0) {
        RCLCPP_ERROR(node->get_logger(),
                     "JointLimits could not be parsed from robot_description.");
    }

    // setup publisher for /joint_states
    jointStatesPub = node->create_publisher<MsgJointState>("joint_states", 10);

    // launch executor for updating
    auto jointStateUpdate = [this](MsgJointState jointState) {
        this->readJointsFromTopic(jointState);
    };
    jointStateSub = node->create_subscription<MsgJointState>(
        "/joint_states", 10, jointStateUpdate);
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);

    updateMask({});  // create mask that does not remove any joints
    thread = std::thread([this]() { this->exec->spin(); });
}

std::shared_ptr<const Model> neo_utils::RBDLWrapper::get_model() {
    return model;
}

VectorNd neo_utils::RBDLWrapper::get_q() const { return maskVec(q); }

VectorNd neo_utils::RBDLWrapper::get_qdot() const { return maskVec(qdot); }

neo_utils::RBDLWrapper::JointLimit neo_utils::RBDLWrapper::get_jointLimit(
    const int maskedIdx) const {
    const int qIdx = mask.masked2qIdx(maskedIdx);
    return jointLimits[qIdx];
}

std::vector<std::string> neo_utils::RBDLWrapper::get_jointNames() const {
    return maskVec(jointNames);
}

int neo_utils::RBDLWrapper::jointName2qIdx(const std::string &jointName) const {
    auto iter = jointName2qIdxMap.find(jointName);
    if (iter == jointName2qIdxMap.end()) return -1;
    const int originalIdx = iter->second;
    return mask.qIdx2masked(originalIdx);
}

/**
 * This function creates new mask that rearranges vectors to only include the
 * not listed joint names.
 */
void neo_utils::RBDLWrapper::updateMask(
    const std::vector<std::string> &maskedOutJoints) {
    int lastIdx = 0;
    mask = Mask();  // clean mask
    for (const std::string &name : jointNames) {
        bool isMasked =
            std::find(maskedOutJoints.begin(), maskedOutJoints.end(), name) !=
            maskedOutJoints.end();
        if (isMasked) {
            continue;
        }

        auto iter = jointName2qIdxMap.find(name);
        const int qIdx = iter != jointName2qIdxMap.end() ? iter->second : -1;
        mask.add(qIdx, lastIdx++);
    }
}

/**
 * Publishes the given q values to /joint_states.
 * @returns A vector of all joint names that could not be changed (because of
 *          wrong name or limits)
 */
std::vector<std::string> neo_utils::RBDLWrapper::publishJoints(
    std::vector<double> q) {
    std::vector<std::string> errorJointNames = {};
    std::vector<double> unmasked_q = unmaskVec(q, 0.0);

    MsgJointState msg;
    msg.position = {};
    msg.name = {};
    for (int qIdx = 0; qIdx < (int)unmasked_q.size(); qIdx++) {
        const double curr_q = unmasked_q[qIdx];
        if (qIdx < 0) continue;
        const std::string name = jointNames[qIdx];
        const JointLimit limit = jointLimits[qIdx];
        if (limit.q_min > curr_q || curr_q > limit.q_max) {
            errorJointNames.push_back(name);
            continue;
        }
        msg.position.push_back(curr_q);
        msg.name.push_back(name);
    }

    msg.header.frame_id = "";
    msg.header.stamp = node->get_clock()->now();
    msg.velocity = {};
    msg.effort = {};
    jointStatesPub->publish(msg);

    return errorJointNames;
}

Vector3d neo_utils::RBDLWrapper::base2body(Vector3d pBase,
                                           const std::string &bodyName) const {
    return base2body(pBase, bodyName, this->q);
}

// CAUTION: takes unmasked q
Vector3d neo_utils::RBDLWrapper::base2body(Vector3d pBase,
                                           const std::string &bodyName, VectorNd qNew) const {
    uint bodyId = model->GetBodyId(bodyName.c_str());
    Vector3d pBody = CalcBaseToBodyCoordinates(*model, qNew, bodyId, pBase);
    return pBody;
}

Vector3d neo_utils::RBDLWrapper::body2base(Vector3d pBody,
                                           const std::string &bodyName) const {
    return body2base(pBody, bodyName, this->q);
}

// CAUTION: takes unmasked q
Vector3d neo_utils::RBDLWrapper::body2base(Vector3d pBody,
                                           const std::string &bodyName, VectorNd qNew) const {
    uint bodyId = model->GetBodyId(bodyName.c_str());
    Vector3d pBase = CalcBodyToBaseCoordinates(*model, qNew, bodyId, pBody);
    return pBase;
}

const std::string neo_utils::RBDLWrapper::retrieveUrdf() {
    auto paramClientNode = std::make_shared<rclcpp::SyncParametersClient>(
        node, "robot_state_publisher");
    while (!paramClientNode->wait_for_service(std::chrono::seconds(1))) {
    }

    return paramClientNode->get_parameter<std::string>("robot_description");
}

/**
 * Reads the current values of each joints from the joint_states topic.
 */
void neo_utils::RBDLWrapper::readJointsFromTopic(MsgJointState msg) {
    for (uint jointStateIdx = 0; jointStateIdx < msg.name.size();
         jointStateIdx++) {
        std::string name = msg.name[jointStateIdx];
        auto mapIter = jointName2qIdxMap.find(name);
        uint q_idx = -1;
        if (mapIter != jointName2qIdxMap.end()) {
            q_idx = mapIter->second;
        } else {
            continue;
        }
        q[q_idx] = msg.position[jointStateIdx];
        qdot[q_idx] =
            msg.velocity.size() > 0 ? msg.velocity[jointStateIdx] : 0.0;
    }
}

int neo_utils::RBDLWrapper::setupJoints(const char *xml) {
    tinyxml2::XMLDocument xmlDoc;
    if (xmlDoc.Parse(xml) != 0) {
        return -1;
    }
    jointLimits = std::vector<JointLimit>(model->q_size);
    jointNames = std::vector<std::string>(model->q_size);

    auto xmlRobot = xmlDoc.FirstChildElement("robot");
    for (auto xmlJoint = xmlRobot->FirstChildElement("joint");
         xmlJoint != nullptr;
         xmlJoint = xmlJoint->NextSiblingElement("joint")) {
        const std::string jointName = xmlJoint->Attribute("name");
        if (jointName.empty()) continue;
        const char *type = xmlJoint->Attribute("type");
        if (!type || std::string(type) == "fixed") continue;
        //          write to map
        std::string jointNameCopy = jointName;
        int wordPos = jointName.find("joint");
        if (wordPos >= 0) {
            jointNameCopy.replace(wordPos, std::string("joint").length(),
                                  "link");
        }
        // find body with same name, without "joint", to select correct
        // link and joint have same id in rbdl => bodyId = jointId
        uint bodyId = model->GetBodyId(jointNameCopy.c_str());
        if (bodyId > model->mBodies.size()) continue;
        int q_idx = model->mJoints[bodyId].q_index;

        jointName2qIdxMap.insert({jointName, q_idx});

        //          write limits
        auto xmlLimit = xmlJoint->FirstChildElement("limit");
        JointLimit limit = {};
        if (xmlLimit) {
            xmlLimit->QueryDoubleAttribute("lower", &limit.q_min);
            xmlLimit->QueryDoubleAttribute("upper", &limit.q_max);
            xmlLimit->QueryDoubleAttribute("effort", &limit.effort);
            xmlLimit->QueryDoubleAttribute("velocity", &limit.velocity);
        } else {
            limit.q_min = 0.0;
            limit.q_max = 2.0 * M_PI;
            limit.velocity = HUGE_VAL;
            limit.effort = HUGE_VAL;
        }
        jointLimits[q_idx] = limit;
        jointNames[q_idx] = jointName;
    }
    return 0;
}