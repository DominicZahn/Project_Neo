#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
typedef std_msgs::msg::Float64 msg_Float64;
#include <sensor_msgs/msg/joint_state.hpp>
typedef sensor_msgs::msg::JointState msg_JointState;
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
     * @param radPerSec joint speed in Radians per Second
     */
    JointMoverNode(const std::vector<std::string>& topicNameList,
               const std::string topicNamePrefix = "/",
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
            //RCLCPP_INFO(
            //    this->get_logger(), "%s: (%f -> %f, %f, %f)", topicName.c_str(),
            //    jointTopic.currentPostion, jointTopic.targetPosition,
            //    std::copysign(this->radPerUpdate, delta), nextTargetPostion);
            RCLCPP_INFO(
                this->get_logger(), "%s: (%f -> %f)", topicName.c_str(),
                jointTopic.currentPostion, jointTopic.targetPosition);
            jointTopic.publisher->publish(msg);
        }
    }
};

