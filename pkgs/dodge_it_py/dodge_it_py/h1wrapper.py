import pinocchio as pin
from pinocchio import RobotWrapper
import pinocchio.casadi as cpin

import numpy as np
import casadi as c

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

model_dir ='/home/robot/ws/pkgs/ros2_heinz/h1_gazebo_sim/ros_gz_h1_description/models/h1_ign/'
urdf_file = 'h1_2_handless.urdf' # 'h1_2.urdf'
sdf_file = 'model.sdf'
mesh_dir = 'meshes/'

def stdvec2list(stdvec) -> list:
    l = []
    for v in stdvec:
        l.append(v)
    return l
   
class JointState_Node(Node):
    def __init__(self):
        super().__init__('jackson_pub_node')
        self.pub = self.create_publisher(
            JointState,
            'joint_states',
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
        self.pub.publish(msg)

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

class H1Wrapper():
    def _dynamic2fixedJoints(self,
                            dynamic_joint_names : list[str]) -> str:
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
        self.robot = pin.RobotWrapper.BuildFromURDF(model_dir+urdf_file)
        self.all_joint_names = stdvec2list(self.robot.model.names)
        self.all_joint_values0 = np.zeros(len(self.all_joint_names)).tolist()

        # mirror layer
        self.mirror_layer = MirrorLayer(self.robot.nq+1)

        # rclpy Nodes
        rclpy.init()
        self.node = JointState_Node()

        self.init_casadi()

    def init_casadi(self):
        cmodel = cpin.Model(self.robot.model)
        cdata = cmodel.createData()
        nq = cmodel.nq
        nv = cmodel.nv
        
        self.q = c.SX.sym('q', nq)
        self.qdot = c.SX.sym('qdot', nv)
        self.tau = c.SX.sym('tau', nv)
        cpin.aba(cmodel, cdata, self.q, self.qdot ,self.tau)
        self.qqdot = cdata.ddq
        
        # CoM
        cpin.framesForwardKinematics(cmodel, cdata, self.q)
        CoM = cpin.centerOfMass(cmodel, cdata, self.q)

        # PoS
        l_ankle = cdata.oMf[13] # left_ankle_roll_link
        r_ankle = cdata.oMf[25] # right_ankle_roll_link
        
        self.PoS_center = cpin.SE3()
        self.PoS_center.translation = (r_ankle.translation + l_ankle.translation) / 2
        self.PoS_center.rotation = l_ankle.rotation
        self.CoM_proj = self.proj2PoS(CoM)
        
        # head (lidar_link)
        id_head = self.robot.model.getFrameId('lidar_link')
        self.head_pos = cdata.oMf[id_head].translation

    def set_q0(self, q0 : np.ndarray | c.SX) -> np.ndarray:
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
            
    def stability(self) -> c.SX:
        d = self.PoS_center.translation - self.CoM_proj
        return c.dot(d,d)

    def proj2PoS(self, p : c.SX) -> c.SX:
        v = p - self.PoS_center.translation
        PoS_normal = (self.PoS_center.rotation @ c.SX([0,0,1]))
        PoS_normal /= c.dot(PoS_normal, PoS_normal)
        dist = c.dot(v, PoS_normal)
        p_proj = p - dist * PoS_normal
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
        self.init_casadi()
        
        print('Warning: Mirror Layer was reset.')
        self.mirror_layer = MirrorLayer(self.robot.nq+1)
        
    def mirrorJoints(self,
                     joint_name_real : str,
                     joint_name_mirrored : str) -> bool:
        model = self.robot.model
        joint_existence = model.existJointName(joint_name_real) and model.existJointName(joint_name_mirrored)
        if not joint_existence:
            return False
        
        r_id = model.getJointId(joint_name_real)
        m_id = model.getJointId(joint_name_mirrored)
        
        return self.mirror_layer.set_mirror(r_id, m_id)
        
    def reduced2mirrored(self, q : np.ndarray | c.SX) -> np.ndarray | c.SX:
        nq_mirrored = self.get_nq(reduced=False)
        if type(q) is np.ndarray:
            q_mirrored = np.zeros((nq_mirrored,1))
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
            print(q_mirrored)
            
        return q_mirrored

            
    def mirrored2reduced(self, q : np.ndarray | c.SX) -> np.ndarray | c.SX:
        nq_reduced = self.get_nq(reduced=True)
        if type(q) is np.ndarray:
            q_reduced = np.zeros((nq_reduced,1))
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
        
    def publishJoints(self,
                      q : list[float],
                      names : list[str],
                      qdot : list[float] = [],
                      qddot : list[float] = []) -> list[str]:
        assert(len(q) == len(names))
        
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

        self.node.update_joints(
            full_q,
            full_names,
            full_qdot,
            full_qddot)
        return error_names