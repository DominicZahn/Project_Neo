#include <boost/geometry.hpp>
#include <boost/lockfree/spsc_queue.hpp>
namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::polygon<Point> Polygon;
typedef bg::model::segment<Point> Segment;

#include <ctime>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
typedef sensor_msgs::msg::PointCloud2 msgPointCloud2;
#include <sensor_msgs/point_cloud2_iterator.hpp>
typedef sensor_msgs::PointCloud2Iterator<float> PointCloud2Iter;
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
typedef visualization_msgs::msg::Marker visMarker;
#include <visualization_msgs/msg/marker_array.hpp>

#define NOT_IMPLEMENTED                                                \
    std::cerr << "This function is not implemented yet!" << std::endl; \
    assert(false);
#define PTS_PER_POLYGON 2048  // just cause that makes it more stable
typedef boost::lockfree::spsc_queue<Point,
                                    boost::lockfree::capacity<PTS_PER_POLYGON>>
    PointQueue;

#include "neo_utils/stability.hpp"
#define stabilityCriteria(CoM, PoS) \
    (neo_utils::Stability::sumPtDist(CoM, PoS))

bool comparePoints(Point pt0, Point pt1) {
    return bg::get<0>(pt0) == bg::get<0>(pt1) &&
           bg::get<1>(pt0) == bg::get<1>(pt1);
}

class SupportPolygon {
   public:
    /*
    This constructor also consumes ALL points in the queue!
    */
    SupportPolygon(PointQueue &pts) {
        Polygon polygon;
        pts.consume_all(
            [&polygon](Point pt) { bg::append(polygon.outer(), pt); });
        bg::convex_hull(polygon, this->hullPolygon);
    }

    void visualizePolygon(
        std::shared_ptr<rclcpp::Publisher<visMarker>> pubPolygon) {
        visMarker marker;
        marker.header.frame_id = "pelvis";
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
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.color.a = 0.7;

        pubPolygon->publish(marker);
    }

    void visualizeStability(
        std::shared_ptr<rclcpp::Publisher<msgPointCloud2>> pub,
        const uint gridLength = 50) {
        float x0 = 0;
        float x1 = 0;
        float y0 = 0;
        float y1 = 0;
        for (const Point &p : this->hullPolygon.outer()) {
            minmaxSetter(bg::get<0>(p), x0, x1);
            minmaxSetter(bg::get<1>(p), y0, y1);
        }
        const float stepSizeX = (x1 - x0) / gridLength;
        const float stepSizeY = (y1 - y0) / gridLength;

        msgPointCloud2 msgPcl;
        msgPcl.width = gridLength;
        msgPcl.height = gridLength;
        msgPcl.header.frame_id = "pelvis";
        msgPcl.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(msgPcl);
        modifier.setPointCloud2Fields(
            4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
            sensor_msgs::msg::PointField::FLOAT32, "z", 1,
            sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
            sensor_msgs::msg::PointField::FLOAT32);

        PointCloud2Iter xIt(msgPcl, "x");
        PointCloud2Iter yIt(msgPcl, "y");
        PointCloud2Iter zIt(msgPcl, "z");
        PointCloud2Iter vIt(msgPcl, "intensity");

        std::srand(std::time({}));

        for (float i = 0; i < gridLength; i++) {
            for (float j = 0; j < gridLength; j++) {
                const float x = x0 + i * stepSizeX;
                const float y = y0 + j * stepSizeY;

                Point simulatedCoM = Point(x, y);
                float v = neo_utils::Stability::minEdgeDist(simulatedCoM,
                                                            this->hullPolygon);
                if (v < 0) continue;
                *vIt = v;
                *xIt = x;
                *yIt = y;
                *zIt = 0.0;

                ++xIt;
                ++yIt;
                ++zIt;
                ++vIt;
            }
        }

        pub->publish(msgPcl);
    }

    bool centerOfMassInside(Point com) {
        return bg::covered_by(com, this->hullPolygon);
    }

    const Polygon &getHull() { return this->hullPolygon; }

   private:
    Polygon hullPolygon;

    static void minmaxSetter(const float &v, float &out_min, float &out_max) {
        out_min = std::min(v, out_min);
        out_max = std::max(v, out_max);
    }
};

void writeToBuffer(ros_gz_interfaces::msg::Contacts contacts, PointQueue &pts) {
    for (auto c : contacts.contacts) {
        for (auto pos : c.positions) {
            Point pt = Point(pos.x, pos.y);
            while (!pts.push(pt)) {
                pts.reset();
            }
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
            node, "/CoM", 10, [&com](geometry_msgs::msg::PointStamped msgPt) {
                com = Point(msgPt.point.x, msgPt.point.y);
            });

    auto visPubPoly = node->create_publisher<visMarker>("vis_PoS", 10);
    auto visPubStability =
        node->create_publisher<msgPointCloud2>("vis_Stability", 10);
    auto stabilityPub =
        node->create_publisher<std_msgs::msg::Float32>("stability", 10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (pts.read_available() < PTS_PER_POLYGON) continue;

        SupportPolygon supPolygon = SupportPolygon(pts);
        supPolygon.visualizePolygon(visPubPoly);
        supPolygon.visualizeStability(visPubStability);

        float stabilityMetric =
            neo_utils::Stability::minEdgeDist(com, supPolygon.getHull());
        std_msgs::msg::Float32 stabilityMsg;
        stabilityMsg.data = stabilityMetric;
        stabilityPub->publish<std_msgs::msg::Float32>(stabilityMsg);

        RCLCPP_INFO(node->get_logger(), "Static Stability: %f",
                    stabilityMetric);
    }

    rclcpp::shutdown();
    return 0;
}