#include <cstdio>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class RVizPublisher : public rclcpp::Node {
   public:
    RVizPublisher(const std::string topicName) : Node("rviz_pub_" + topicName) {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "pub_" + topicName, 10);
    }

    rclcpp::Duration publishPoint(geometry_msgs::msg::Point pt) {
        visualization_msgs::msg::Marker marker;
        rclcpp::Time currTimestamp = this->get_clock()->now();
        marker.header.stamp = currTimestamp;
        marker.pose.position = pt;
        this->publisher_->publish(marker);

        auto timeSinceLastPublish = currTimestamp - this->lastPublishedTimestamp_;
        this->lastPublishedTimestamp_ = currTimestamp;
        return timeSinceLastPublish;
    }

   private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Time lastPublishedTimestamp_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVizPublisher>("COM"));
    rclcpp::shutdown();
    return 0;
}