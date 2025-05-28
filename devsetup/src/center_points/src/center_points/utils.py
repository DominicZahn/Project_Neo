import rospy, logging
from geometry_msgs.msg import PointStamped, Point
from urdf_parser_py.urdf import URDF
import numpy as np

def urdf_link_map() -> dict:
    logger = logging.getLogger('urdf_parser_py')
    prev_level = logger.level
    logger.setLevel(logging.CRITICAL)
    link_dict = URDF.from_parameter_server().link_map
    logger.setLevel(prev_level)
    return link_dict

def Point_to_numpy(p : Point) -> np.ndarray:
    return np.array([
        p.x,
        p.y,
        p.z
    ])

def numpy_to_Point(p : np.ndarray) -> Point:
    return Point(p[0], p[1], p[2])

class PointCommunicator:
    def __init__(self, name : str , rate : int = 10) -> None:
        self.pub = rospy.Publisher(name+'_pub', PointStamped, queue_size=10)
        rospy.init_node(name+'_node', anonymous=False)
        self.rate = rospy.Rate(rate)

    def publish_point(self, pt : np.ndarray):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.point = Point(pt[0], pt[1], pt[2])
        self.pub.publish(msg)
        # rospy.loginfo(msg)
        self.rate.sleep()