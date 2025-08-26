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
#include <random>

using namespace std::chrono_literals;

namespace neo_utils {
// namespace conversion {
// using namespace RigidBodyDynamics::Math;
//
// std::vector<double> RbdlNd2std(const VectorNd &rbdl) {
//     return std::vector<double>(rbdl.data(), rbdl.data() + rbdl.size());
// }
//
//// if @param copy is false the VectorNd will use the std::vectors memory and
//// only point to it
// VectorNd std2RbdlNd(std::vector<double> vec) {
//     VectorNd rbdl = VectorNd(vec.size());
//     std::copy(vec.begin(), vec.end(), rbdl.data());
//     return rbdl;
// }
//
// }  // namespace conversion

/**
 * @class RBDLWrapper
 * A wrapper for rbdl that also stays consistent with the current ros2 state.
 * It therefore depends on the following processes already running:
 * - /joint_states (topic)
 * - /robot_state_publisher/robot_description (parameter)
 * /joint_states can also be updated using publishJointes()
 */
class RBDLWrapper {
   public:
    struct JointLimit {
        double q_min;
        double q_max;
        double velocity;
        double effort;
    };
    class Mask {
       public:
        bool add(const int qIdx, const int maskedIdx) {
            if (qIdx2maskedMap.find(qIdx) != qIdx2maskedMap.end()) return false;
            qIdx2maskedMap[qIdx] = maskedIdx;
            masked2qIdxMap[maskedIdx] = qIdx;
            _size += 1;
            return true;
        }
        size_t size() const { return _size; }
        int qIdx2masked(const int qIdx) const {
            auto iter = qIdx2maskedMap.find(qIdx);
            return iter != qIdx2maskedMap.end() ? iter->second : -1;
        }
        int masked2qIdx(const int maskedqIdx) const {
            auto iter = masked2qIdxMap.find(maskedqIdx);
            return iter != masked2qIdxMap.end() ? iter->second : -1;
        }

       private:
        size_t _size = 0;
        std::unordered_map<int, int> qIdx2maskedMap;
        std::unordered_map<int, int> masked2qIdxMap;
    };
    RBDLWrapper(bool floatingBase = false);
    int jointName2qIdx(const std::string &jointName) const;
    void updateMask(const std::vector<std::string> &maskedOutJoints);

    template <typename T>
    std::vector<T> maskVec(const std::vector<T> &vec) const {
        std::vector<T> maskedVec(mask.size());
        for (int originalIdx = 0; originalIdx < (int)vec.size(); originalIdx++) {
            int maskedIdx = mask.qIdx2masked(originalIdx);
            if (maskedIdx < 0 || maskedIdx > (int)vec.size()) continue;
            maskedVec[maskedIdx] = vec[originalIdx];
        }
        return maskedVec;
    }

    VectorNd maskVec(const VectorNd &vec) const {
        VectorNd maskedVec(vec.size());
        for (int originalIdx = 0; originalIdx < (int)vec.size(); originalIdx++) {
            int maskedIdx = mask.qIdx2masked(originalIdx);
            if (maskedIdx < 0 || maskedIdx > (int)vec.size()) continue;
            maskedVec[maskedIdx] = vec[originalIdx];
        }
        return maskedVec;
    }

    template <typename T>
    std::vector<T> unmaskVec(const std::vector<T> &maskedVec,
                             T fillValue) const {
        std::vector<T> unmaskedVec(jointNames.size(), fillValue);
        for (int maskedIdx = 0; maskedIdx < (int)maskedVec.size(); maskedIdx++) {
            int unmaskedIdx = mask.masked2qIdx(maskedIdx);
            if (unmaskedIdx < 0 || unmaskedIdx > (int)maskedVec.size()) continue;
            unmaskedVec[unmaskedIdx] = maskedVec[maskedIdx];
        }
        return unmaskedVec;
    }

    VectorNd unmaskVec(const VectorNd &maskedVec, double fillValue) const {
        VectorNd unmaskedVec = VectorNd::Constant(jointNames.size(), fillValue);
        for (int maskedIdx = 0; maskedIdx < (int)maskedVec.size(); maskedIdx++) {
            int unmaskedIdx = mask.masked2qIdx(maskedIdx);
            if (unmaskedIdx < 0 || unmaskedIdx > (int)maskedVec.size()) continue;
            unmaskedVec[unmaskedIdx] = maskedVec[maskedIdx];
        }
        return unmaskedVec;
    }

    std::vector<std::string> publishJoints(std::vector<double> q);
    Vector3d base2body(Vector3d pBase, const std::string &bodyName) const;
    // ------ getter for exposed members -------
    std::shared_ptr<const Model> get_model();
    VectorNd get_q() const;
    VectorNd get_qdot() const;
    JointLimit get_jointLimit(const int maskedIdx) const;
    std::vector<std::string> get_jointNames() const;
    // -------- forwarded rbdl funcs -----------

   private:
    // ---------- internal members -------------
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Subscription<MsgJointState>> jointStateSub;
    std::unordered_map<std::string, int> jointName2qIdxMap;
    std::thread thread;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
    std::shared_ptr<rclcpp::Publisher<MsgJointState>> jointStatesPub;
    Mask mask;  // reroutes qIdx to mask out joints
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
    int setupJoints(const char *xml);
};
}  // namespace neo_utils