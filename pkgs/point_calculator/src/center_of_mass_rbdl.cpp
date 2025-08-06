#include <rbdl/rbdl.h>

#include <cstdio>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL Addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std::chrono_literals;

class RVizPublisher : public rclcpp::Node {
   public:
    RVizPublisher(const std::string topicName) : Node("rviz_pub_" + topicName) {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            topicName, 10);
        this->lastPublishedTimestamp_ = this->get_clock()->now();
    }

    rclcpp::Duration publishPoint(Math::Vector3d p) {
        rclcpp::Time currTimestamp = this->get_clock()->now();
        geometry_msgs::msg::PointStamped pt;
        pt.header.frame_id = "odom";
        pt.header.stamp = currTimestamp;
        pt.point.x = p[0];
        pt.point.y = p[1];
        pt.point.z = p[2];
        this->publisher_->publish(pt);

        auto timeSinceLastPublish =
            currTimestamp - this->lastPublishedTimestamp_;
        this->lastPublishedTimestamp_ = currTimestamp;
        return timeSinceLastPublish;
    }

   private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Time lastPublishedTimestamp_;
};

const std::string readURDF(std::shared_ptr<rclcpp::Node> node = nullptr) {
    std::shared_ptr<rclcpp::Node> fallbackNode = nullptr;
    if (!node) {
        fallbackNode = std::make_shared<rclcpp::Node>("urdf_reader");
        node = fallbackNode;
    }
    auto paramClientNode = std::make_shared<rclcpp::SyncParametersClient>(
        node, "robot_state_publisher");
    while (!paramClientNode->wait_for_service(std::chrono::seconds(1))) {
    }
    std::string urdfStr =
        paramClientNode->get_parameter<std::string>("robot_description");
    return urdfStr;
}

/*
rbdl:           *_link
joint_states:   *_joint
*/
void updateQ(std::shared_ptr<Model> model,
             sensor_msgs::msg::JointState jointState, Math::VectorNd& q,
             Math::VectorNd& qdot) {
    for (auto p : model->mBodyNameMap) {
        std::string name = p.first;
        int wordPos = name.find_last_of("link", 0);
        name.replace(wordPos, std::string("link").length(), "joint");

        int jointStateIdx =
            std::find(jointState.name.begin(), jointState.name.end(), name) -
            jointState.name.end();
        if (jointStateIdx == 0) continue;
        const int uid = p.second;
        q[uid] = jointState.position[jointStateIdx];
        qdot[uid] = jointState.velocity[jointStateIdx];
    }
}

int main(int argc, char** argv) {
    rbdl_check_api_version(RBDL_API_VERSION);
    rclcpp::init(argc, argv);

    const std::string urdf = readURDF();
    std::shared_ptr<Model> model = std::make_shared<Model>();
    Addons::URDFReadFromString(urdf.c_str(), model.get(), true);

    std::shared_ptr<rclcpp::Node> jointReaderNode =
        std::make_shared<rclcpp::Node>("jointReaderNode");
    Math::VectorNd q, qdot;
    q = Math::VectorNd::Zero(model->q_size);
    qdot = Math::VectorNd::Zero(model->qdot_size);
    std::cout << q << " " << qdot << std::endl;
    auto jointStateSub =
        jointReaderNode->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [model, &q, &qdot](sensor_msgs::msg::JointState j) {
                updateQ(model, j, q, qdot);
            });
    std::shared_ptr<RVizPublisher> rivzPub =
        std::make_shared<RVizPublisher>("CoM");
    while (rclcpp::ok()) {
        // calculate CoM
        Math::Scalar totalMass;
        Math::Vector3d com;
        Utils::CalcCenterOfMass(*model, q, qdot, NULL, totalMass, com);
        // publish CoM
        rclcpp::Duration d = rivzPub->publishPoint(com);
    }

    rclcpp::shutdown();
    return 0;
}