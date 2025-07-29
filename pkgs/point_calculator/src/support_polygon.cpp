#include <boost/geometry.hpp>
#include <boost/lockfree/spsc_queue.hpp>
namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::polygon<Point> Polygon;

#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

#define NOT_IMPLEMENTED                                                \
    std::cerr << "This function is not implemented yet!" << std::endl; \
    assert(false);
#define PTS_PER_POLYGON 8 // 4 for each feet
typedef boost::lockfree::spsc_queue<Point, boost::lockfree::capacity<PTS_PER_POLYGON>> PointQueue;

bool comparePoints(Point pt0, Point pt1) {
    return bg::get<0>(pt0) == bg::get<0>(pt1) &&
           bg::get<1>(pt0) == bg::get<1>(pt1);
}

class SupportPolygon : public Polygon {
   public:
    /*
    This constructor also consumes ALL points in the queue!
    */
    SupportPolygon(PointQueue &pts) {
        Polygon polygon;
        pts.consume_all(
            [&polygon](Point pt) { bg::append(polygon.outer(), pt); });
        // if (comparePoints(pts[0], *(pts.end() - 1)))
        //     bg::append(polygon.outer(), pts[0]);  // ring condition for
        //     polygon
        bg::convex_hull(polygon, this->hullPolygon);
    }

    void visualize() { NOT_IMPLEMENTED }

    bool centerOfMassInside(Point com) {
        return bg::covered_by(com, this->hullPolygon);
    }

   private:
    Polygon hullPolygon;
};

void writeToBuffer(ros_gz_interfaces::msg::Contacts contacts,
                   PointQueue &pts) {
    for (auto c : contacts.contacts) {
        for (auto pos : c.positions) {
            Point pt = Point(pos.x, pos.y);
            pts.push(pt);
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("supportPolygon");
    std::list<std::string> contactTopicList = {"/h1/left_contact_force",
                                               "/h1/right_contact_force"};
    PointQueue pts;
    std::vector<std::shared_ptr<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>>> subscriptions;
    for (const std::string &topic : contactTopicList) {
        auto callback = [&pts](ros_gz_interfaces::msg::Contacts c) -> void {
            writeToBuffer(c, pts);
        };

        auto subscription = rclcpp::create_subscription<ros_gz_interfaces::msg::Contacts>(
            node, topic, 10, callback);
        subscriptions.push_back(subscription);
    }
    std::cout << "test" << std::endl;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (pts.read_available() <= 4) continue;

        SupportPolygon supPolygon = SupportPolygon(pts);
        supPolygon.visualize();
    }

    rclcpp::shutdown();
    return 0;
}