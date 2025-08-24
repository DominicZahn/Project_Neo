#include <rbdl/rbdl.h>

#include <cstdio>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
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

class JointStateReader : public rclcpp::Node {
   public:
    VectorNd q, qdot;
    JointStateReader(const std::shared_ptr<Model> model)
        : Node("joint_state_reader") {
        this->subscription =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10,
                [this](sensor_msgs::msg::JointState j) { this->update(j); });
        this->q = VectorNd::Zero(model->q_size);
        this->qdot = VectorNd::Zero(model->qdot_size);
        this->model = model;
    }
    VectorNd *get_q() { return &q; }
    VectorNd *get_qdot() { return &qdot; }

   private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>>
        subscription;
    std::shared_ptr<Model> model;
    std::unordered_map<std::string, uint> jointState2JointIdx;

    void update(sensor_msgs::msg::JointState jointState) {
        for (uint jointStateIdx = 0; jointStateIdx < jointState.name.size();
             jointStateIdx++) {
            std::string name = jointState.name[jointStateIdx];
            auto mapIter = jointState2JointIdx.find(name);
            uint q_idx = -1;
            if (mapIter != jointState2JointIdx.end()) {
                // retrive from map
                q_idx = mapIter->second;
            } else {
                // retrive from mBodies and save to map
                int wordPos = name.find("joint");
                if (wordPos >= 0) {
                    name.replace(wordPos, std::string("joint").length(),
                                 "link");
                }
                // find body with same name, without "joint", to select correct
                // link link and joint have the same id in rbdl => bodyId =
                // jointId
                uint bodyId = model->GetBodyId(name.c_str());
                if (bodyId > model->mBodies.size()) continue;
                q_idx = model->mJoints[bodyId].q_index;

                jointState2JointIdx.insert(
                    {jointState.name[jointStateIdx], q_idx});
            }
            q[q_idx] = jointState.position[jointStateIdx];
            qdot[q_idx] = jointState.velocity.size() > 0
                              ? jointState.velocity[jointStateIdx]
                              : 0.0;
        }
    }
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

int main(int argc, char **argv) {
    rbdl_check_api_version(RBDL_API_VERSION);
    rclcpp::init(argc, argv);

    const std::string urdf = readURDF();
    std::shared_ptr<Model> model = std::make_shared<Model>();
    Addons::URDFReadFromString(urdf.c_str(), model.get(), false);

    auto jointReaderNode = std::make_shared<JointStateReader>(model);
    std::shared_ptr<RVizPublisher> rvizCoMPub =
        std::make_shared<RVizPublisher>("CoM");
    std::shared_ptr<RVizPublisher> rvizCoPPub =
        std::make_shared<RVizPublisher>("CoP");

    while (rclcpp::ok()) {
        rclcpp::spin_some(jointReaderNode);
        //  calculate CoM
        Math::Scalar totalMass;
        Math::Vector3d com;
        Utils::CalcCenterOfMass(*model, jointReaderNode->q,
                                jointReaderNode->qdot, NULL, totalMass, com);
        // publish CoM
        rclcpp::Duration d = rvizCoMPub->publishPoint(com);
        Math::Vector3d cop = {com[0], com[1], 0.0};
        rvizCoPPub->publishPoint(cop);
    }

    rclcpp::shutdown();
    return 0;
}