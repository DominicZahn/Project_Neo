#include <neo_utils/rbdlWrapper.hpp>

neo_utils::RBDLWrapper::RBDLWrapper(bool floatingBase) {
    node = std::make_shared<rclcpp::Node>("rbdl_wrapper");
    const std::string &urdfStr = retrieveUrdf();
    model = std::make_shared<Model>();
    Addons::URDFReadFromString(urdfStr.c_str(), model.get(), floatingBase);

    auto jointStateUpdate = [this](MsgJointState jointState) {
        this->updateJointStates(jointState);
    };
    jointStateSub = node->create_subscription<MsgJointState>(
        "/joint_states", 10, jointStateUpdate);

    q = VectorNd::Zero(100);
    qdot = VectorNd::Zero(100);

    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);
    thread = std::thread([this]() { this->exec->spin(); });
}

neo_utils::RBDLWrapper::~RBDLWrapper() {}

std::shared_ptr<const Model> neo_utils::RBDLWrapper::get_model() {
    return model;
}

const VectorNd &neo_utils::RBDLWrapper::get_q() const { return q; }

const VectorNd &neo_utils::RBDLWrapper::get_qdot() const { return qdot; }

const std::string neo_utils::RBDLWrapper::retrieveUrdf() {
    auto paramClientNode = std::make_shared<rclcpp::SyncParametersClient>(
        node, "robot_state_publisher");
    while (!paramClientNode->wait_for_service(std::chrono::seconds(1))) {
    }

    return paramClientNode->get_parameter<std::string>("robot_description");
}

void neo_utils::RBDLWrapper::updateJointStates(MsgJointState msg) {
    for (uint jointStateIdx = 0; jointStateIdx < msg.name.size();
         jointStateIdx++) {
        std::string name = msg.name[jointStateIdx];
        auto mapIter = jointState2JointIdx.find(name);
        uint q_idx = -1;
        if (mapIter != jointState2JointIdx.end()) {
            // retrive from map
            q_idx = mapIter->second;
        } else {
            // retrive from mBodies and save to map
            int wordPos = name.find("joint");
            if (wordPos >= 0) {
                name.replace(wordPos, std::string("joint").length(), "link");
            }
            // find body with same name, without "joint", to select correct
            // link link and joint have the same id in rbdl => bodyId =
            // jointId
            uint bodyId = model->GetBodyId(name.c_str());
            if (bodyId > model->mBodies.size()) continue;
            q_idx = model->mJoints[bodyId].q_index;

            jointState2JointIdx.insert({msg.name[jointStateIdx], q_idx});
        }
        q[q_idx] = msg.position[jointStateIdx];
        qdot[q_idx] =
            msg.velocity.size() > 0 ? msg.velocity[jointStateIdx] : 0.0;
    }
}