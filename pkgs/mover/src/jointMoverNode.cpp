#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
typedef std_msgs::msg::Float64 msg_Float64;
#include <sensor_msgs/msg/joint_state.hpp>
typedef sensor_msgs::msg::JointState msg_JointState;
#include <atomic>
#include <string>

class JointMoverNode : public rclcpp::Node {
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
     */
    JointMoverNode(const std::vector<std::string>& topicNameList,
                   const std::string topicNamePrefix = "/",
                   const std::string topicNameSuffix = "",
                   const double epsilon = 0.05)
        : rclcpp::Node("jointMover") {
        this->epsilon = epsilon;
        this->topicNamePrefix = topicNamePrefix;
        this->topicNameSuffix = topicNameSuffix;
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
            this->numberOfMovingJoints = 0;
            for (uint i = 0; i < jointStateSize; i++) {
                const std::string jointName = js.name[i];
                auto it = this->jointMap.find(jointName);
                if (it == this->jointMap.end()) continue;
                const double jointPosition = js.position[i];
                it->second.currentPostion = jointPosition;

                const double delta =
                    it->second.targetPosition - it->second.currentPostion;
                if (delta > this->epsilon) {
                    this->numberOfMovingJoints++;
                }
            }
        };
        this->jointStateSub = this->create_subscription<msg_JointState>(
            "/joint_states", 10, jointPositionCallback);
    }

    /**
     * new joint states are not published unitl execute is called
     */
    void update(const std::string topicName, double position) {
        this->jointMap[topicName].targetPosition = position;
    }

    /**
     * joint values need to be set before calling execute()
     */
    void execute() {
        for (const auto& p : jointMap) {
            const std::string topicName = p.first;
            JointTopic jointTopic = p.second;
            if (std::isnan(jointTopic.currentPostion) ||
                std::isnan(jointTopic.targetPosition))
                continue;
            const double delta =
                std::abs(jointTopic.targetPosition - jointTopic.currentPostion);
            if (delta < this->epsilon) continue;
            msg_Float64 msg;
            msg.data = jointTopic.targetPosition;
            RCLCPP_INFO(this->get_logger(), "%s: (%f -> %f)", topicName.c_str(),
                        jointTopic.currentPostion, jointTopic.targetPosition);
            jointTopic.publisher->publish(msg);
        }
    }

    int getNumberOfMovingJoints() { return this->numberOfMovingJoints; }

    std::unordered_map<std::string, double> getCurrentJointStates() {
        std::unordered_map<std::string, double> map;
        for (const auto& p : this->jointMap) {
            if (!std::isnan(p.second.currentPostion))
                map.insert({p.first, p.second.currentPostion});
        }
        return map;
    }

   private:
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<rclcpp::Subscription<msg_JointState>> jointStateSub;

    std::atomic_int numberOfMovingJoints = 0;
    double epsilon = 0.0;
    std::string topicNamePrefix = "";
    std::string topicNameSuffix = "";
    std::unordered_map<std::string, JointTopic> jointMap;

    const std::string getFullTopicName(const std::string topicName) {
        return topicNamePrefix + topicName + topicNameSuffix;
    }
};
