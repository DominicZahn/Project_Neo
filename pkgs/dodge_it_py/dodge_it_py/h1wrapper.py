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

class MirrorLayer():
    def __init__(self, size : int):
        # values in mapper
        # -1: entry has no mirror
        #  i: entry at index is mirrored to i
        self.mapper = np.full(size, -1)
    
    def set_mirror(self, real_id : int, mirror_id : int) -> bool:
        out_of_range = real_id >= self.mapper.size or mirror_id >= self.mapper.size
        if out_of_range:
            return False
        
        self.mapper[real_id] = mirror_id
#        for i in range(mirror_id, self.mapper.size):
#            mapping = self.mapper[i]
#            if mapping >= 0:
#                self.mapper[i] -= 1
        return True
   
    def reduced2mirrored_joints(self,
                                    q_reduced : np.ndarray | c.SX) -> np.ndarray | c.SX:
        
        q_mirrored = q_reduced.copy()
        for i in range(self.mapper.size):
            mirror_id = self.mapper[i]
            if mirror_id >= 0:
                q_mirrored = np.insert(q_mirrored, mirror_id, q_reduced[i])
        return q_mirrored
                
    
    def mirrored2realReduced_joints(self,
                                    q_mirrored : c.SX | np.ndarray) -> c.SX | np.ndarray:
        pass
    
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
        self.mirror_layer = MirrorLayer(self.robot.q0.size)

        # rclpy Nodes
        rclpy.init()
        self.node = JointState_Node()

        # casadi variables
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
        
        self.mirror_layer = MirrorLayer(self.robot.q0.size)
        
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
    
    def reduced2Mirrored(self, q : np.ndarray | c.SX) -> np.ndarray | c.SX:
        return self.mirror_layer.reduced2mirrored_joints(q)
    
    def mirrored2reduced(self, q : np.ndarray | c.SX) -> np.ndarray | c.SX:
        return self.mirror_layer.mirrored2realReduced_joints(q)
    
    def jointNames(self, reduced=False) -> list[str]:
        if not reduced:
            return list(self.robot.model.names)
        
        # remove mirrored
        names = list(self.robot.model.names)
        mirror_i_list = []
        for i in range(1,len(names)): # skip 'universe' at 0
            mirror_i = self.mirror_layer.mapper[i-1]
            if mirror_i >= 0: # no 'universe' in mapper
                mirror_i_list.append(mirror_i)
        mirror_i_list = sorted(mirror_i_list, reverse=True)
        for mirror_i in mirror_i_list:
            names.pop(mirror_i)
        return names
        
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