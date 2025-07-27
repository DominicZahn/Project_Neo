#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
using namespace boost::geometry;
typedef model::d2::point_xy<double> Point;
typedef model::segment<Point> Segment;

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    

    rclcpp::shutdown();
    return 0;
}