import rospy, os, sys
from geometry_msgs.msg import PointStamped, Point
from urdf_parser_py.urdf import URDF
import numpy as np

def urdf_link_map() -> dict:
    prev_stdout = sys.stderr
    sys.stderr = open(os.devnull, 'w')
    link_dict = URDF.from_parameter_server().link_map
    sys.stderr = prev_stdout
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
        self.rate = rospy.Rate(rate)

    def publish_point(self, pt : np.ndarray):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.point = Point(pt[0], pt[1], pt[2])
        self.pub.publish(msg)
        self.rate.sleep()