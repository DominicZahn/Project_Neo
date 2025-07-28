#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
using namespace boost::geometry;
typedef model::d2::point_xy<double> Point;
typedef model::segment<Point> Segment;

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

class MinimalSubscriber : public rclcpp::Node {
   public:
    MinimalSubscriber() : Node("min_sub") {
        auto callback =
            [this](ros_gz_interfaces::msg::Contacts contacts) -> void {
            RCLCPP_INFO(this->get_logger(), "contacts received");
        };
            this->create_subscription<ros_gz_interfaces::msg::Contacts>(
                "/h1/left_contact_force", 10, callback);
    }

   private:
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr
        subscription;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    //rclcpp::spin(std::make_shared<MinimalSubscriber>());

    auto subNode = std::make_shared<rclcpp::Node>("subNode");
    auto logger = subNode->get_logger();
    [logger](ros_gz_interfaces::msg::Contacts contacts) -> void {
        RCLCPP_INFO(logger, "contacts received");
    };
    auto subscribtion = subNode->create_subscription<ros_gz_interfaces::msg::Contacts>("/h1/left_contact_force", 10, callback);
    rclcpp::spin(subNode);

    rclcpp::shutdown();
    return 0;
}