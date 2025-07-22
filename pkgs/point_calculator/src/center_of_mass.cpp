#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdfdom/urdf_parser/urdf_parser.h>

#include <cstdio>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class RVizPublisher : public rclcpp::Node {
   public:
    RVizPublisher(const std::string topicName) : Node("rviz_pub_" + topicName) {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            topicName, 10);
        this->lastPublishedTimestamp_ = this->get_clock()->now();
    }

    rclcpp::Duration publishPoint(geometry_msgs::msg::PointStamped pt) {
        rclcpp::Time currTimestamp = this->get_clock()->now();
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

urdf::ModelInterfaceSharedPtr readURDF() {
    auto readerNode = std::make_shared<rclcpp::Node>("urdf_reader");
    auto paramClientNode = std::make_shared<rclcpp::SyncParametersClient>(
        readerNode, "robot_state_publisher");
    while (!paramClientNode->wait_for_service(std::chrono::seconds(1))) {
    }
    std::string urdfStr =
        paramClientNode->get_parameter<std::string>("robot_description");
    return urdf::parseURDF(urdfStr);
}

geometry_msgs::msg::PointStamped calcCOM(urdf::ModelInterfaceSharedPtr urdf) {
    urdf::LinkConstSharedPtr rootLink = urdf->getRoot();
    std::vector<urdf::LinkSharedPtr> links = {};
    urdf->getLinks(links);

    auto tf2node = rclcpp::Node::make_shared("tf2_node");
    std::shared_ptr<tf2_ros::Buffer> buffer =
        std::make_shared<tf2_ros::Buffer>(tf2node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> listener =
        std::make_shared<tf2_ros::TransformListener>(*buffer);
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::Time timestamp = rclcpp::Time(0);

    geometry_msgs::msg::PointStamped com;
    com.header.frame_id = rootLink->name;
    com.header.stamp = timestamp;
    double totalMass = 0.0;
    for (urdf::LinkSharedPtr link : links) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform =
                buffer->lookupTransform(rootLink->name, link->name, timestamp,
                                        rclcpp::Duration::from_seconds(1.0));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(tf2node->get_logger(), e.what());
            com.point.x = 0;
            com.point.y = 0;
            com.point.z = 0;
            return com;
        }
        if (!link->inertial) continue;
        double mass = link->inertial->mass;
        com.point.x += transform.transform.translation.x * mass;
        com.point.y += transform.transform.translation.y * mass;
        com.point.z += transform.transform.translation.z * mass;
        totalMass += mass;
        // RCLCPP_INFO(tf2node->get_logger(), ("Checking " + link->name).c_str());
    }

    com.point.x /= totalMass;
    com.point.y /= totalMass;
    com.point.z /= totalMass;
    return com;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    urdf::ModelInterfaceSharedPtr urdfPtr = readURDF();
    std::shared_ptr<RVizPublisher> rivzPub =
        std::make_shared<RVizPublisher>("com");
    while (true) {
        auto com = calcCOM(urdfPtr);
        rclcpp::Duration d = rivzPub->publishPoint(com);
    }

    rclcpp::shutdown();
    return 0;
}