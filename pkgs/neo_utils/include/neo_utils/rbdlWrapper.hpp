#pragma once

// ------------------- rbdl ------------------
#include <rbdl/Body.h>
#include <rbdl/Joint.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Model.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL Addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#include <tinyxml2.h>

// ------------------- ros2 -------------------
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
typedef sensor_msgs::msg::JointState MsgJointState;

// ----------------- general C++ --------------
#include <cmath>
#include <memory>
#include <thread>
#include <unordered_map>

using namespace std::chrono_literals;

namespace neo_utils {
/**
 * @class RBDLWrapper
 * A wrapper for rbdl that also stays consistent with the current ros2 state.
 * It therefore depends on the following processes already running:
 * - /joint_states (topic)
 * - /robot_state_publisher/robot_description (parameter)
 */
class RBDLWrapper {
   public:
    struct JointLimit {
        double q_min;
        double q_max;
        double velocity;
        double effort;
    };
    RBDLWrapper(bool floatingBase = false);
    ~RBDLWrapper();
    int jointName2qIdx(std::string &jointName) const;
    std::vector<std::string> publishJoints(std::vector<double>);
    // ------ getter for exposed members -------
    std::shared_ptr<const Model> get_model();
    const VectorNd &get_q() const;
    const VectorNd &get_qdot() const;
    const std::vector<JointLimit> &get_qLimits() const;
    const std::vector<std::string> &get_jointNames() const;
    // -------- forwarded rbdl funcs -----------

   private:
    // ---------- internal members -------------
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Subscription<MsgJointState>> jointStateSub;
    std::unordered_map<std::string, uint> jointName2qIdxMap;
    std::thread thread;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
    std::shared_ptr<rclcpp::Publisher<MsgJointState>> jointStatesPub;
    // ---------- exposed members --------------
    std::shared_ptr<Model> model;
    VectorNd q;
    VectorNd qdot;
    std::vector<JointLimit> jointLimits;
    std::vector<std::string> jointNames;
    // ---------- private functions ------------
    const std::string retrieveUrdf();
    void readJointsFromTopic(MsgJointState msg);
    /**
     * Iterates through urdf in xml format and builds
     * - jointLimits and
     * - jointName2qIdx-map
     */
    int setupJoints(const char* xml);
};
}  // namespace neo_utils