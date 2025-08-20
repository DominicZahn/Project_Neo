#pragma once

// ------------------- rbdl ------------------
#include <rbdl/Body.h>
#include <rbdl/Joint.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Model.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL Addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// ------------------- ros2 -------------------
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
typedef sensor_msgs::msg::JointState MsgJointState;

// ----------------- general C++ --------------
#include <cmath>
#include <memory>
#include <unordered_map>
#include <thread>

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
    RBDLWrapper(bool floatingBase = false);
    ~RBDLWrapper();
    // ------ getter for exposed members ------
    std::shared_ptr<const Model> get_model();
    const VectorNd &get_q() const;
    const VectorNd &get_qdot() const;

   private:
    // ---------- internal members -------------
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Subscription<MsgJointState>> jointStateSub;
    std::unordered_map<std::string, uint> jointState2JointIdx;
    std::thread thread;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
    // ---------- exposed members --------------
    std::shared_ptr<Model> model;
    VectorNd q;
    VectorNd qdot;
    // ---------- private functions ------------
    const std::string retrieveUrdf();
    void updateJointStates(MsgJointState msg);
};
}  // namespace neo_utils