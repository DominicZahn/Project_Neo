#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
typedef std_msgs::msg::Float64 msg_Float64;
#include <sensor_msgs/msg/joint_state.hpp>
typedef sensor_msgs::msg::JointState msg_JointState;
#include <string>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL Addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
#include <math.h>

#include <nlopt.hpp>

using namespace std::chrono_literals;

// typedef struct
//{
//     double a, b;
// } ConstraintData;
//
// double objFunc(uint n, const double *x, double *grad, void *objFuncData)
//{
//     if (grad)
//     {
//         grad[0] = 0.0;
//         grad[1] = 0.5 / sqrt(x[1]);
//     }
//     return sqrt(x[1]);
// }
//
// double constraintFunc(uint n, const double *x, double *grad,
//                       void *constraintData)
//{
//     ConstraintData *d = (ConstraintData *)constraintData;
//     double a = d->a, b = d->b;
//     if (grad)
//     {
//         grad[0] = 3 * a * (a * x[0] + b) * (a * x[0] + b);
//         grad[1] = -1.0;
//     }
//     return ((a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1]);
// }

class JointMover : public rclcpp::Node {
    class JointTopic {
       public:
        double currentPostion;  // position is only updated on /joint_state
                                // subscription
        double targetPosition;
        std::shared_ptr<rclcpp::Publisher<msg_Float64>> publisher;
    };

   public:
    /**
     * @param topicNameList vector of topic names which should match the
     *                      following two criteria:
     *                      - name in /joint_state topic
     *                      - topic with $topicNamePrefix+THIS+$topicNameSuffix
     *                        moves joints
     * @param radPerSec joint speed in Radians per Second
     */
    JointMover(const std::vector<std::string>& topicNameList,
               const std::string topicNamePrefix = "",
               const std::string topicNameSuffix = "",
               const uint updateFrequency = 100, float radPerSec = 5)
        : rclcpp::Node("jointMover") {
        this->topicNamePrefix = topicNamePrefix;
        this->topicNameSuffix = topicNameSuffix;
        this->updateFrequency = updateFrequency;
        this->setJointSpeed(radPerSec);
        // create map of joints
        this->jointMap.reserve(topicNameList.size());
        for (std::string topicName : topicNameList) {
            const std::string fullTopicName = getFullTopicName(topicName);
            auto pub = this->create_publisher<msg_Float64>(fullTopicName, 10);
            JointTopic jt = {NAN, NAN, pub};
            this->jointMap.insert({topicName, jt});
        }

        // setup /joint_state subscriber
        auto jointPositionCallback = [this](msg_JointState js) {
            size_t jointStateSize = js.name.size();
            for (uint i = 0; i < jointStateSize; i++) {
                const std::string jointName = js.name[i];
                auto it = this->jointMap.find(jointName);
                if (it == this->jointMap.end()) continue;
                const double jointPosition = js.position[i];
                it->second.currentPostion = jointPosition;
            }
        };
        this->jointStateSub = this->create_subscription<msg_JointState>(
            "/joint_states", 10, jointPositionCallback);

        // setup repeated publishing
        std::chrono::milliseconds timerRevokeDuration(
            (uint)(1000 / this->updateFrequency));
        this->timer = this->create_wall_timer(
            timerRevokeDuration, [this]() { this->publish_all(); });
    }

    void update(const std::string topicName, double position) {
        this->jointMap[topicName].targetPosition = position;
    }

    /**
     * Sets the jointSpeed in Radian per Seconds.
     * If the value is negative there is no limit and the Joint will be moved as
     * fast as posible.
     */
    void setJointSpeed(double radPerSec) {
        this->radPerUpdate =
            radPerSec > 0 ? radPerSec / this->updateFrequency : -1;
    }
    double getJointSpeed() { return this->radPerUpdate; }

   private:
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<rclcpp::Subscription<msg_JointState>> jointStateSub;

    double radPerUpdate;
    uint updateFrequency;  // in Hz
    std::string topicNamePrefix = "";
    std::string topicNameSuffix = "";
    std::unordered_map<std::string, JointTopic> jointMap;

    const std::string getFullTopicName(const std::string topicName) {
        return topicNamePrefix + topicName + topicNameSuffix;
    }

    void publish_all() {
        for (const auto& p : jointMap) {
            const std::string topicName = p.first;
            JointTopic jointTopic = p.second;
            if (std::isnan(jointTopic.currentPostion) ||
                std::isnan(jointTopic.targetPosition))
                continue;
            // position based linear interpolation
            double delta =
                jointTopic.targetPosition - jointTopic.currentPostion;
            double nextTargetPostion =
                std::copysign((double)this->radPerUpdate, delta) +
                jointTopic.currentPostion;
            if (std::abs(delta) < this->radPerUpdate)
                nextTargetPostion = jointTopic.targetPosition;

            msg_Float64 msg;
            msg.data = nextTargetPostion;
            RCLCPP_INFO(
                this->get_logger(), "%s: (%f -> %f, %f, %f)", topicName.c_str(),
                jointTopic.currentPostion, jointTopic.targetPosition,
                std::copysign(this->radPerUpdate, delta), nextTargetPostion);
            jointTopic.publisher->publish(msg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::vector<std::string> topicNameList = {"left_ankle_pitch_joint",
                                              "left_ankle_roll_joint",
                                              "left_elbow_joint",
                                              "left_hip_pitch_joint",
                                              "left_hip_roll_joint",
                                              "left_hip_yaw_joint",
                                              "left_knee_joint",
                                              "left_shoulder_pitch_joint",
                                              "left_shoulder_roll_joint",
                                              "left_shoulder_yaw_joint",
                                              "right_ankle_joint",
                                              "right_ankle_pitch_joint",
                                              "right_ankle_roll_joint",
                                              "right_elbow_joint",
                                              "right_hip_pitch_joint",
                                              "right_hip_roll_joint",
                                              "right_hip_yaw_joint",
                                              "right_knee_joint",
                                              "right_shoulder_pitch_joint",
                                              "right_shoulder_roll_joint",
                                              "right_shoulder_yaw_joint",
                                              "torso_joint"};
    auto jointMover =
        std::make_shared<JointMover>(topicNameList, "/h1/", "/cmd_pos");
    jointMover->update(topicNameList[2], 0.5);
    rclcpp::spin(jointMover);

    // RBDL check
    // rbdl_check_api_version(RBDL_API_VERSION);

    // auto readerNode = std::make_shared<rclcpp::Node>("urdf_reader");
    // auto paramClientNode = std::make_shared<rclcpp::SyncParametersClient>(
    //     readerNode, "robot_state_publisher");
    // while (!paramClientNode->wait_for_service(std::chrono::seconds(1))) {
    // }
    // const std::string urdfStr =
    //     paramClientNode->get_parameter<std::string>("robot_description");
    // Model *model = new Model();
    // Addons::URDFReadFromString(urdfStr.c_str(), model, false);
    // if (!model) {
    //     std::cerr << "Model could not be read by rbdl!" << std::endl;
    //     return -1;
    // }
    // std::cout << "Degree of freedom overview:" << std::endl;
    // std::cout << Utils::GetModelDOFOverview(*model);

    //// nlopt example
    // double lowerBounds[2] = {-HUGE_VAL, 0};  // lower bounds for a and b
    // nlopt_opt opt;
    // opt = nlopt_create(NLOPT_LD_MMA, 2);
    // nlopt_set_lower_bounds(opt, lowerBounds);
    // nlopt_set_min_objective(opt, objFunc, NULL);

    // ConstraintData constraintData[2] = {{2, 0}, {-1, 1}};
    // nlopt_add_inequality_constraint(opt, constraintFunc, &constraintData[0],
    //                                 1e-8);
    // nlopt_add_inequality_constraint(opt, constraintFunc, &constraintData[1],
    //                                 1e-8);

    // nlopt_set_xtol_rel(opt, 1e-4);  // tolerance for objective function

    // double x[2] = {1.234, 5.678};  // inital guess for a and b
    // double minf;                   // minimum objective value upon return
    // if (nlopt_optimize(opt, x, &minf) < 0)
    //     std::cout << "nlopt failed!!" << std::endl;
    // else
    //     std::cout << "found minimum at (" << x[0] << " | " << x[1]
    //               << ") = " << minf << std::endl;

    rclcpp::shutdown();
    return 0;
}
