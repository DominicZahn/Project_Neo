#include <boost/geometry.hpp>
#include <boost/lockfree/spsc_queue.hpp>
namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::polygon<Point> Polygon;
typedef bg::model::segment<Point> Segment;

#include <geometry_msgs/msg/point_stamped.hpp>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#define NOT_IMPLEMENTED                                                \
    std::cerr << "This function is not implemented yet!" << std::endl; \
    assert(false);
#define PTS_PER_POLYGON 8  // 4 for each feet
typedef boost::lockfree::spsc_queue<Point,
                                    boost::lockfree::capacity<PTS_PER_POLYGON>>
    PointQueue;

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

    void visualize(
        std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>>
            publisher) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.type = 11;  // TRIANGLE_LIST
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        Point centroid = Point(0, 0);
        bg::centroid(this->hullPolygon, centroid);
        geometry_msgs::msg::Point geoCentroid;
        geoCentroid.x = bg::get<0>(centroid);
        geoCentroid.y = bg::get<1>(centroid);
        geoCentroid.z = 0;
        geometry_msgs::msg::Point geoLastPt;
        geoLastPt.z = -1;
        for (const Point &pt : this->hullPolygon.outer()) {
            // triangulate polygon
            geometry_msgs::msg::Point geoPt;
            geoPt.x = bg::get<0>(pt);
            geoPt.y = bg::get<1>(pt);
            geoPt.z = 0.0;

            if (geoLastPt.z != -1) {  // z=-1 signals unbound value
                marker.points.push_back(geoCentroid);
                marker.points.push_back(geoPt);
                marker.points.push_back(geoLastPt);
            }
            geoLastPt = geoPt;
        }
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 0.7;

        publisher->publish(marker);
    }

    bool centerOfMassInside(Point com) {
        return bg::covered_by(com, this->hullPolygon);
    }

    /**
     * Calculate static stability based on the distance to the nearest segment.
     * If the CoM is outside of the supportPolygon the distance is negative.
     */
    double calculateStaticStability(Point com) {
        double d = toEdgeDistance(com);
        return centerOfMassInside(com) ? d : -d;
    }

   private:
    Polygon hullPolygon;

    double toEdgeDistance(Point pt) {
        std::vector<Point> polyPts = this->hullPolygon.outer();
        const size_t polyPtsCount = polyPts.size();
        double dist = MAXFLOAT;
        for (uint i = 1; i < polyPtsCount; i++) {
            const Segment seg = Segment(polyPts[i-1], polyPts[i]);
            dist = std::min(bg::distance(seg, pt), dist);
        }
        return dist;
    }
};

void writeToBuffer(ros_gz_interfaces::msg::Contacts contacts, PointQueue &pts) {
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
    std::vector<
        std::shared_ptr<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>>>
        contactSubscriptions;
    for (const std::string &topic : contactTopicList) {
        auto callback = [&pts](ros_gz_interfaces::msg::Contacts c) -> void {
            writeToBuffer(c, pts);
        };

        auto subscription =
            rclcpp::create_subscription<ros_gz_interfaces::msg::Contacts>(
                node, topic, 10, callback);
        contactSubscriptions.push_back(subscription);
    }

    Point com = Point(0, 0);
    auto comSubscription =
        rclcpp::create_subscription<geometry_msgs::msg::PointStamped>(
            node, "/com", 10, [&com](geometry_msgs::msg::PointStamped msgPt) {
                com = Point(msgPt.point.x, msgPt.point.y);
            });

    auto visPub =
        node->create_publisher<visualization_msgs::msg::Marker>("vis_PoS", 10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (pts.read_available() <= 4) continue;

        SupportPolygon supPolygon = SupportPolygon(pts);
        supPolygon.visualize(visPub);

        RCLCPP_INFO(node->get_logger(), "Static Stability: %f",
                    supPolygon.calculateStaticStability(com));
    }

    rclcpp::shutdown();
    return 0;
}