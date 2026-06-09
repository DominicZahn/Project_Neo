import numpy as np
import casadi as c

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

class GazeboCom_Node(Node):
    def __init__(self, joint_names : list[str], q0 : list[float]):
        super().__init__('h1wrapper_pub_node_gz')
        # create subscriber for joint_states
        self.joint_states_map = {}
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.js_callback,
            10)
        res, msg_init = wait_for_message(
            msg_type=JointState,
            node=self,
            topic='/joint_states',
            time_to_wait=5)
        assert(res)
        self.js_callback(msg_init)

        # create publishers
        self.pub_dict = {}
        for name in joint_names:
             topic_name = self.joint2topic(name)
             self.pub_dict[name] = self.create_publisher(Float64, topic_name, 10)
        self.motor_command = Float64() # to reuse single command
    
    def js_callback(self, msg):
        self.joint_states_map = dict(zip(msg.name, msg.position))
        
    def update_joints(self,
        q : list[float],
        names : list[str],
        qdot : list[float] = [],
        qddot : list[float] = []):
        rclpy.spin_once(self)

        for name, qi in zip(names, q):
            if name == 'universe':
                continue
            self.motor_command.data = qi
            self.pub_dict[name].publish(self.motor_command)
        

    def joint2topic(self, joint_name : str):
        return '/h1/'+joint_name+'/cmd_pos'
   
class RVizCom_Node(Node):
    def __init__(self, pub_name_list : list[str]):
        super().__init__('h1wrapper_pub_node_rviz')
        self.pub_joint_states = self.create_publisher(
            JointState,
            'joint_states',
            1)
        self.pub_dict = {}
        for name in pub_name_list:
            self.pub_dict[name] = self.create_publisher(
                PointStamped,
                name,
                1)

    def update_joints(self,
        q : list[float],
        names : list[str],
        qdot : list[float] = [],
        qddot : list[float] = []):

        msg = JointState()
        msg.name = names
        msg.position = q
        msg.velocity = qdot
        msg.effort = qddot
        msg.header.frame_id = ""
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint_states.publish(msg)

    def publish_point(self, p : np.ndarray | c.SX, pub_name : str) -> bool:
        if pub_name not in self.pub_dict.keys():
            return False
        if type(p) == c.SX and p.size1() != 3:
            return False
        if type(p) == np.ndarray and p.size != 3:
            return False
        msg = PointStamped()
        msg.point.x = p[0][0]
        msg.point.y = p[1][0]
        msg.point.z = p[2][0]
        msg.header.frame_id = 'left_ankle_roll_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_dict[pub_name].publish(msg)
        return True

