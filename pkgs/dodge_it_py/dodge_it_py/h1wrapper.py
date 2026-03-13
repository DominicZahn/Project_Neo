import time
import pinocchio as pin
from pinocchio import RobotWrapper
import pinocchio.casadi as cpin

import numpy as np
import casadi as c

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float64

model_dir ='/home/robot/ws/pkgs/ros2_heinz/h1_gazebo_sim/ros_gz_h1_description/models/h1_ign/'
# urdf_file = 'h1_2_handless.urdf' # 'h1_2.urdf'
urdf_file = 'h1_2_handless_foot_root.urdf' # 'h1_2.urdf'
sdf_file = 'model.sdf'
mesh_dir = 'meshes/'

def stdvec2list(stdvec) -> list:
    l = []
    for v in stdvec:
        l.append(v)
    return l

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
        
        # move slowly to starting position
        # self.slow_motion(q0, joint_names)
    
    def js_callback(self, msg):
            self.joint_states_map = dict(zip(msg.name, msg.position))

    """
    NOT WORKING
    def slow_motion(self, q0 : list[float], names : list[str]):
        cycle = 0
        while True:
            q_curr = np.zeros_like(names,dtype=float)
            for i in range(len(names)):
                q_curr[i] = self.joint_states_map[names[i]]
                
            cutoff = 0.005
            dq = (np.array(q0) - q_curr)*0.01
            dq[dq < cutoff] = 0.0
            cycle += 1
            print(cycle, np.linalg.norm(dq), dq)
            if np.linalg.norm(dq) < cutoff:
                break
            q_next = q_curr + dq
            self.update_joints(q_next.tolist(), names)
            time.sleep(1)
        input("Press any key to start...")
    """
        
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

class MirrorLayer():
    def __init__(self, size : int):
        self.mapper = [[i] for i in range(size)]
    
    def set_mirror(self, real_id : int, mirrored_id) -> bool:
        if real_id >= len(self.mapper) and mirrored_id >= len(self.mapper):
            return False
        
        self.mapper[real_id].append(mirrored_id)
        # remove mirrored_id from real_id entries
        self.mapper[mirrored_id] = []
        for i in range(mirrored_id+1, len(self.mapper)):
            # move real id forward
            if self.mapper[i]:
                self.mapper[i][0] -= 1
        return True
    
class PolygonOfSupport():
    """
    01---11        
    |     |
    |     |
    00---10
    """
    def __init__(self) -> None:
        self.xl = 0.0339
        self.xu = -0.363
        self.yl = -0.07 # -0.0875
        self.yu = 0.175 
        self._center = c.SX([
            (self.yu+self.yl)/2,
            (self.xu+self.xl)/2,
            0.0
        ])
    
    def get_corners(self) -> list[c.SX]:
        """
        00, 10, 11, 01
        """
        return [
            c.SX([self.xl,self.yl,0.0]),
            c.SX([self.xu,self.yl,0.0]),
            c.SX([self.xu,self.yu,0.0]),
            c.SX([self.xl,self.yu,0.0])
        ]
    
    def get_center(self) -> c.SX:
        return self._center

    def stability_centerDistParable(self, p : c.SX) -> c.SX:
        yp = p[0]
        yc = self._center[0]
        return (yp-yc)**2/(self.yl-yc)**2
    
    def stability_centerDist(self, p : c.SX) -> c.SX:
        d = self._center - p
        return c.dot(d,d)
    
    def stability_minEdgeY(self, p : c.SX) -> c.SX:
        xp = p[1]
        yp = p[0]
        xc = self._center[1]
        yc = self._center[0]
        # dx0 = (xp-self._x0)/(xc-self._x0)
        # dx1 = (xp-self._x1)/(xc-self._x1)
        dy0 = (yp-self.yl)/(yc-self.yl)-1
        dy1 = (yp-self.yu)/(yc-self.yu)-1
        return c.fmax(dy0, dy1)


class H1Wrapper():
    def _dynamic2fixedJoints(self,
                            dynamic_joint_names : list[str]) -> list[str]:
        fixed_joint_names = []
        for joint_name in self.robot.model.names:
            if not self.robot.model.existJointName(joint_name):
                print("Warning: joint " + str(joint_name) + " does not belong to the model!")
                continue
            if joint_name in dynamic_joint_names:
                continue
            fixed_joint_names.append(joint_name) 
        return fixed_joint_names

    def __init__(self):
        self.gazebo = False

        self.robot = pin.RobotWrapper.BuildFromURDF(model_dir+urdf_file)
        self.all_joint_names = stdvec2list(self.robot.model.names)
        self.all_joint_values0 = np.zeros(len(self.all_joint_names)).tolist()

        self.mirror_layer = MirrorLayer(self.robot.nq+1)

        rclpy.init()
        self.rviz_node = RVizCom_Node([
            'CoM_proj',
            'CoM',
            'PoS_center',
            'head_pos'
            ])
        self.joint_node = self.rviz_node

        self.init_casadi()

    def init_gazebo(self, q0_map : dict[str,float]):
        if type(self.joint_node) is GazeboCom_Node:
            self.joint_node.destroy_node()
        q0 = self.all_joint_values0
        for name, qi in q0_map.items():
            i = self.all_joint_names.index(name)
            q0[i] = qi
        self.joint_node = GazeboCom_Node(self.all_joint_names[1:], q0[1:])
        self.gazebo = True

    def uses_gazebo(self):
        return self.gazebo

    def init_casadi(self):
        cmodel = cpin.Model(self.robot.model)
        # cmodel.gravity.linear = c.SX.zeros(3)
        cdata = cmodel.createData()
        nq = cmodel.nq
        nv = cmodel.nv

        # dynamic / kinematics
        """
        self._q = c.SX.sym('q', nq)
        self._q = self.full2mirrored(self._q)
        self._qdot = c.SX.sym('qdot', nv)
        self._qdot = self.full2mirrored(self._qdot)
        self._tau = c.SX.sym('tau', nv)
        self._tau = self.full2mirrored(self._tau)
        cpin.aba(cmodel, cdata, self._q, self._qdot ,self._tau)
        self._qddot = cdata.ddq
        self._qddot = self.full2mirrored(self._qddot)
        """
        self._q = c.SX.sym('q', nq)
        self._q = self.full2mirrored(self._q)
        self._qdot = c.SX.sym('qdot', nv)
        self._qdot = self.full2mirrored(self._qdot)
        self._tau = c.SX.sym('tau', nv)
        self._tau = self.full2mirrored(self._tau)
        self._qddot = c.SX.sym('qddot', nv)
        self._qddot = self.full2mirrored(self._qddot)
        cpin.framesForwardKinematics(cmodel, cdata, self._q)
        
        # CoM
        self._CoM = cpin.centerOfMass(cmodel, cdata, self._q)
        self._CoM_proj = self.proj2PoS(self._CoM)

        # PoS (hardcoded)
        self._PoS = PolygonOfSupport()
        
        # head (lidar_link)
        id_head = self.robot.model.getFrameId('lidar_link')
        self._head_pos = cdata.oMf[id_head].translation

    def get_q(self, reduced=True):
        if not reduced:
            return self._q
        return self.mirrored2reduced(self._q)
    
    def get_qdot(self, reduced=True):
        if not reduced:
            return self._qdot
        return self.mirrored2reduced(self._qdot) 
    
    def get_qddot(self, reduced=True):
        if not reduced:
            return self._qddot
        return self.mirrored2reduced(self._qddot)
    
    def get_tau(self, reduced=True):
        if not reduced:
            return self._tau
        return self.mirrored2reduced(self._tau)
    
    def get_upperPosLimit(self, reduced=True):
        qub = self.robot.model.upperPositionLimit
        if not reduced:
            return qub
        return self.mirrored2reduced(qub)
    
    def get_lowerPosLimit(self, reduced=True):
        qlb = self.robot.model.lowerPositionLimit
        if not reduced:
            return qlb
        return self.mirrored2reduced(qlb)
    
    def get_PoS_center(self) -> c.SX:
        return self._PoS.get_center()
    def get_head_pos(self):
        return self._head_pos

    def set_q0(self, q0 : np.ndarray | c.SX):
        if type(q0) is c.SX:
            nq0 = np.array(c.DM(q0))
        else:
            nq0 = q0
        self.q0 = nq0

    def get_nq(self, reduced : bool = False) -> int:
        """
        Retrun number of joints in q without 'universe' joint
        
        :param reduced: reduce and exclude mirrored joints
        :type reduced: bool
        :return: joint count
        :rtype: int
        """
        nq = 0
        if not reduced:
            return self.robot.nq
        
        for e in self.mirror_layer.mapper:
            if e: nq += 1
        return nq - 1 # remove universe

    def getJointId(self, name : str) -> int:
        model = self.robot.model
        if not model.existJointName(name):
            return -1 
        
        id = model.getJointId(name)

        entry = self.mirror_layer.mapper[id]
        if len(entry) > 0: # not mirrored id
            return entry[0]
        
        # mirrored id
        for i in range(len(self.mirror_layer.mapper)):
            e = self.mirror_layer.mapper[i]
            if len(e) < 1:
                continue
            for mirrored_id in e[1:]:
                if mirrored_id == id:
                    return e[0]
        return -1

    def proj2PoS(self, p : c.SX) -> c.SX:
        p_proj = c.SX(3,1)
        p_proj[0] = p[0]
        p_proj[1] = p[1]
        p_proj[2] = 0.0
        return p_proj
    
    def fixJoints(self,
                  joint_names : list[str],
                  joint_values : list[float] | None = None,
                  dynamic_representation = False):
        if dynamic_representation:
            fixed_joint_names = self._dynamic2fixedJoints(joint_names)
        else:
            fixed_joint_names = joint_names

        # remove invalid joints (iterate from tail to head)
        for i in range(len(fixed_joint_names)-1, -1, -1):
            name = fixed_joint_names[i]
            if not self.robot.model.existJointName(name) or name == 'universe':
                fixed_joint_names.pop(i)
                if joint_values:
                    joint_values.pop(i)
                print('Warning:', name, 'was not fixed.')

        self.robot = self.robot.buildReducedRobot(
            fixed_joint_names,
            joint_values)
        
        print('Warning: Mirror Layer was reset.')
        self.mirror_layer = MirrorLayer(self.robot.nq+1)
        self.init_casadi()
        
    def mirrorJoints(self,
                     joint_name_real : str,
                     joint_name_mirrored : str) -> bool:
        model = self.robot.model
        joint_existence = model.existJointName(joint_name_real) and model.existJointName(joint_name_mirrored)
        if not joint_existence:
            return False
        
        r_id = model.getJointId(joint_name_real)
        m_id = model.getJointId(joint_name_mirrored)
        
        res = self.mirror_layer.set_mirror(r_id, m_id)
        self.init_casadi()
        return res
    
    def full2mirrored(self, q : np.ndarray | c.SX) -> np.ndarray | c.SX:
        q_reduced = self.mirrored2reduced(q)
        return self.reduced2mirrored(q_reduced)
    
    def reduced2mirrored(self, q : np.ndarray | c.SX) -> np.ndarray | c.SX:
        nq_mirrored = self.get_nq(reduced=False)
        if type(q) is np.ndarray:
            q_mirrored = np.zeros((nq_mirrored))
            q_size = q.size
        else:
            q_mirrored = c.SX(nq_mirrored,1)
            q_size = q.size1()

        assert(q_size == self.get_nq(reduced=True))
        mapper = self.mirror_layer.mapper
        for i, mirror_list in zip(range(len(mapper)-1), mapper[1:]):
            if len(mirror_list) < 1: # mirrored
                continue
            
            q_mirrored[i] = q[mirror_list[0]-1]
            for j in mirror_list[1:]:
                q_mirrored[j-1] = q[mirror_list[0]-1]
            
        return q_mirrored

            
    def mirrored2reduced(self, q : np.ndarray | c.SX) -> np.ndarray | c.SX:
        nq_reduced = self.get_nq(reduced=True)
        if type(q) is np.ndarray:
            q_reduced = np.zeros((nq_reduced))
            q_size = q.size
        else:
            q_reduced = c.SX(nq_reduced,1)
            q_size = q.size1()
        assert(q_size == self.get_nq(reduced=False))

        id_reduced = 0
        for i in range(q_size):
            if len(self.mirror_layer.mapper[i+1]) < 1: # mirrored
                continue
            q_reduced[id_reduced] = q[i]
            id_reduced += 1
        return q_reduced
        
    def jointNames(self, reduced : bool=False) -> list[str]:
        all_names = list(self.robot.model.names)
        if not reduced:
            return all_names
        reduced_names = [] 
        for i in range(len(all_names)):
            entry = self.mirror_layer.mapper[i]
            if entry:
                reduced_names.append(all_names[i])
        return reduced_names
        
    def visualize(self,
                      q : list[float],
                      names : list[str],
                      qdot : list[float] = [],
                      qddot : list[float] = []) -> list[str]:
        assert(len(q) == len(names))
        
        # publish joint_states
        full_names = self.all_joint_names
        full_q = self.all_joint_values0
        full_qdot = []
        full_qddot = []
        error_names = []
        for i_param in range(len(names)):
            name = names[i_param]
            if not name in full_names:
                print('ERROR:', name, 'is not a viable joint')
                error_names.append(name)
                continue
            if not self.robot.model.existJointName(name):
                print('ERROR:', name, 'fixed in this model')
                error_names.append(name)
                continue
            i_full = full_names.index(name)
            full_q[i_full] = q[i_param]
            if len(qdot) > 0:
                full_qdot[i_full] = qdot[i_param]
            if len(qddot) > 0:
                full_qddot[i_full] = qddot[i_param]

        self.joint_node.update_joints(
            full_q,
            full_names,
            full_qdot,
            full_qddot)
        
        # visualize points
        nq_reduced = self.get_nq(reduced=True)
        q_red = c.SX(nq_reduced,1)
        for qi, namei in zip(q, names):
            id = self.getJointId(namei) - 1
            q_red[id] = qi
        q_sym = self.get_q(reduced=True)

        CoM_proj_func = c.Function('CoM_func', [q_sym], [self._CoM_proj])
        CoM_proj_q = np.array(c.DM(CoM_proj_func(q_red)))
        if (not self.rviz_node.publish_point(CoM_proj_q, 'CoM_proj')):
            print('ERROR: CoM_proj could not be published')

        CoM_func = c.Function('CoM', [q_sym], [self._CoM])
        CoM_q = np.array(c.DM(CoM_func(q_red)))
        if (not self.rviz_node.publish_point(CoM_q, 'CoM')):
            print('ERROR: CoM could not be published')

        PoS_func = c.Function('PoS_func', [q_sym], [self._PoS.get_center()])
        PoS_q = np.array(c.DM(PoS_func(q_red)))
        if (not self.rviz_node.publish_point(PoS_q, 'PoS_center')):
            print('ERROR: PoS_center could not be published')
 
        head_func = c.Function('head_func', [q_sym], [self._head_pos])
        head_q = np.array(c.DM(head_func(q_red)))
        if (not self.rviz_node.publish_point(head_q, 'head_pos')):
            print('ERROR: head_pos could not be published')       
        return error_names