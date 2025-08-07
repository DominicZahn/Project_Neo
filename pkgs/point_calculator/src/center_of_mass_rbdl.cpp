#include <rbdl/rbdl.h>

#include <cstdio>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL Addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

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
        pt.header.frame_id = "pelvis";
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

void updateQ(std::shared_ptr<Model> model,
             sensor_msgs::msg::JointState jointState, Math::VectorNd& q,
             Math::VectorNd& qdot) {
    for (uint jointStateIdx = 0; jointStateIdx < jointState.name.size(); jointStateIdx++) {
        std::string name = jointState.name[jointStateIdx];
        int wordPos = name.find("joint");
        if (wordPos >= 0) {
            name.replace(wordPos, std::string("joint").length(), "link");
        }
        // find body with same name, without "joint", to select correct link
        // link and joint have the same id in rbdl => bodyId = jointId
        uint bodyId = model->GetBodyId(name.c_str());
        if (bodyId > model->mBodies.size()) continue;
        uint q_idx = model->mJoints[bodyId].q_index;

        q[q_idx] = jointState.position[jointStateIdx];
        qdot[q_idx] = jointState.velocity.size() > 0 ? jointState.velocity[jointStateIdx] : 0.0;
    }
}

int main(int argc, char** argv) {
    rbdl_check_api_version(RBDL_API_VERSION);
    rclcpp::init(argc, argv);

    const std::string urdf = readURDF();
    std::shared_ptr<Model> model = std::make_shared<Model>();
    Addons::URDFReadFromString(urdf.c_str(), model.get(), false);

    std::shared_ptr<rclcpp::Node> jointReaderNode =
        std::make_shared<rclcpp::Node>("jointReaderNode");
    Math::VectorNd q, qdot;
    q = Math::VectorNd::Zero(model->q_size);
    qdot = Math::VectorNd::Zero(model->qdot_size);
    auto jointStateSub =
        jointReaderNode->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [model, &q, &qdot](sensor_msgs::msg::JointState j) {
                updateQ(model, j, q, qdot);
            });
    std::shared_ptr<RVizPublisher> rivzPub =
        std::make_shared<RVizPublisher>("CoM_rbdl");

    while (rclcpp::ok()) {
        rclcpp::spin_some(jointReaderNode);
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