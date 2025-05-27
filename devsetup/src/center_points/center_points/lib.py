import rospy
from geometry_msgs.msg import PointStamped, Point
import numpy as np

class PointCommunicator:
    def __init__(self, name : str , rate : int = 10) -> None:
        self.pub = rospy.Publisher(name, PointStamped, queue_size=10)
        rospy.init_node(name, anonymous=False)
        self.rate = rospy.Rate(rate)

    def publish_point(self, pt : np.ndarray):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.point = Point(pt[0], pt[1], pt[2])
        self.pub.publish(msg)
        rospy.loginfo(msg)
        self.rate.sleep()