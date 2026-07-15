import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g

import pinocchio.casadi as cpin
import casadi as c

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation
from pathlib import Path
from time import sleep
from csv import writer as CsvWriter

from ext_pkgs.dodge_it_py.dodge_it_py.stability import (
  zmp_centroidal, zmp_full
)

# ----------------- MODEL LOCATION -----------------
MODEL_PATH = Path('/home/robot/ws/src/ros2_heinz/h1_gazebo_sim/ros_gz_h1_description/models/')
MESH_DIR = MODEL_PATH.parent.parent
URDF_FILENAME = "h1_2_handless.urdf"
URDF_MODEL_PATH = MODEL_PATH / "h1_ign/" / URDF_FILENAME

SRDF_FILENAME = "nofloat-h1_2.srdf"
SRDF_FULL_PATH = Path('/home/robot/ws/simple_example/') / SRDF_FILENAME
# ------------------ DEFAULTS -----------------------
DEFAULT_FEET_FRAMES = ["left_ankle_roll_link", "right_ankle_roll_link"]
DEFAULT_REFERENCE_CONF = "knees_bend_0.4"
# ------------------ FRAME MARKERS ------------------
HEAD_FRAME = 'lidar_link'

class H1Wrapper_v2():
    """
    An derived version from the H1Wrapper with some major exceptions:
    - custom floating base
        -> original model
        -> nq = nv because of custom 6D composite joint
    - feet collisions
    - runtime changes are know a lot more difficult
        -> As a result the wrapper should be much more stable
    Obviously there are some general changes and advancements in the code structure to better reflect the current requirements.
    """
    def __init__(self,
                 q0 : npt.NDArray[np.float32] | str = DEFAULT_REFERENCE_CONF,
                 dynamicJoints : list[str] = [],
                 feetFrames : list[str] = DEFAULT_FEET_FRAMES):
        self._setupModels()
        self._setInitalPose(q0)
        self._fixJoints(dynamicJoints)
        self._setupContacts(feetFrames)
        self._initCasadi()
        self._setupVis()
        
    def _setupModels(self):
        # custom floating base to avoid using quaternions in q
        rootJoint = pin.JointModelComposite()
        rootJoint.addJoint(pin.JointModelPX())
        rootJoint.addJoint(pin.JointModelPY())
        rootJoint.addJoint(pin.JointModelPZ())
        rootJoint.addJoint(pin.JointModelRX())
        rootJoint.addJoint(pin.JointModelRY())
        rootJoint.addJoint(pin.JointModelRZ())

        model, collisionModel, visualModel = pin.buildModelsFromUrdf(
            URDF_MODEL_PATH, MESH_DIR, rootJoint
        )
        self.model : pin.Model = model
        self.collisionModel : pin.GeometryModel = collisionModel
        self.visualModel : pin.GeometryModel = visualModel
    
    def _setInitalPose(self, q0 : npt.NDArray[np.float32] | str | None):
        pin.loadReferenceConfigurations(self.model, SRDF_FULL_PATH, verbose=False)
        assert(self.model.referenceConfigurations is not None)
        assert(self.model.referenceConfigurations[DEFAULT_REFERENCE_CONF] is not None)
        
        if type(q0) is str:
            q0 = self.model.referenceConfigurations[q0]
        assert(type(q0) is np.ndarray)
        assert(q0.size == self.model.nq)
        self.q0 = q0

    def _fixJoints(self, dynamicJoints : list[str]):
        assert(all([self.model.existJointName(q) for q in dynamicJoints]))
        jointNames = self.model.names
        assert(jointNames is not None)
        dynamicJointNameMask = np.array(list(map(lambda q: q in dynamicJoints and q != 'universe' and q != 'root_joint', jointNames)))

        # adjusting q0
        assert(self.model.nq is not None)
        n_q0 = np.copy(self.q0[:len(dynamicJoints)+6])
        n_q0[6:] = self.q0[6:][dynamicJointNameMask[2:]]

        # rebuilding model without fixed joints
        fixedJointNames = np.array(self.model.names)[np.logical_not(dynamicJointNameMask)][2:]
        fixedJointIds = list(map(lambda q: self.model.getJointId(q), fixedJointNames))
        newModel, newGeoStdVec = pin.buildReducedModel(
            self.model,
            pin.StdVec_GeometryModel([self.collisionModel, self.visualModel]),
            pin.StdVec_Index(fixedJointIds),
            self.q0)
        self.q0 = np.copy(n_q0)

        assert(type(newModel) is pin.Model)
        self.model = newModel
        assert(
            len(newGeoStdVec) == 2 and
            type(newGeoStdVec[0]) is pin.GeometryModel and
            type(newGeoStdVec[1]) is pin.GeometryModel
        )
        self.collisionModel = newGeoStdVec[0]
        self.visualModel = newGeoStdVec[1]

    def _setupContacts(self, feetFrameNames : list[str]):
        assert(any([self.model.existFrame(f) for f in feetFrameNames]))
        self.feetFrames = feetFrameNames
        self.feetFrameIds = [self.model.getFrameId(frame_name) for frame_name in feetFrameNames]

        self.cContactModels = []
        self.cContactData = []
        assert(self.model.frames)
        for id in self.feetFrameIds:
            frame = self.model.frames[id]
            contactModel = pin.RigidConstraintModel(
                pin.ContactType.CONTACT_6D,
                self.model,
                frame.parentJoint,
                frame.placement
            )
            cContactModel = cpin.RigidConstraintModel(contactModel)
            contactData = cContactModel.createData()

            self.cContactModels.append(cContactModel)
            self.cContactData.append(contactData)
        self.cContactModels = cpin.StdVec_RigidConstraintModel(self.cContactModels)
        self.cContactData = cpin.StdVec_RigidConstraintData(self.cContactData)

    def _initCasadi(self):
        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()
        nq = self.cmodel.nq
        nv = self.cmodel.nv

        # define variables
        self.q = c.SX.sym('q', nq)
        self.qdot = c.SX.sym('qdot', nv)
        self.tau = c.SX.sym('tau', nv)

        # define dynamics
        proxSettings = cpin.ProximalSettings(None, 1e-12, 5)
        # proxSettings = cpin.ProximalSettings(None, 0, 1)
        cpin.initConstraintDynamics(
            self.cmodel,
            self.cdata,
            self.cContactModels,
            self.cContactData)
        cpin.constraintDynamics(
            self.cmodel,
            self.cdata,
            self.q,
            self.qdot,
            self.tau,
            self.cContactModels,
            self.cContactData,
            proxSettings)
        self.qddot = self.cdata.ddq
        self.contactForces = self.cdata.lambda_c

        cpin.computeAllTerms(self.cmodel, self.cdata, self.q, self.qdot)
        cpin.updateFramePlacements(self.cmodel, self.cdata)

        # ZMP
        self.ZMP = zmp_full(
            self.cmodel,
            self.cdata,
            self.q,
            self.qdot,
            self.qddot,
            self.feetFrameIds)
#        self.ZMP = zmp_centroidal(
#            self.cmodel,
#            self.cdata,
#            self.q,
#            self.qdot,
#            self.qddot)
 
        
        # CoM
        self.CoM = cpin.centerOfMass(self.cmodel, self.cdata, self.q)
        
        # head
        assert(self.model.existFrame(HEAD_FRAME))
        headId = self.model.getFrameId(HEAD_FRAME)
        self.headPos = self.cdata.oMf[headId].translation

    def _setupVis(self):
        self._vis = MeshcatVisualizer(
            model=self.model,
            collision_model=self.collisionModel,
            visual_model=self.visualModel,
            copy_models=True
        )
        self._vis.initViewer(loadModel=True)
        self._vis.loadViewerModel()
        self._vis.viewer["zmp"].set_object(
            g.Sphere(0.02),
            g.MeshPhongMaterial(0xf81802))
#        self._vis.viewer["F_l"].set_object(
#            g.Cylinder(1, 0.01),
#            g.MeshPhongMaterial(0xa2bb7d))
#        self._vis.viewer["F_r"].set_object(
#            g.Cylinder(1, 0.01),
#            g.MeshPhongMaterial(0xa2bb7d))

    def _visualizeZMP(self, pos : npt.NDArray):
        mat = pin.SE3(
            rotation=np.eye(3),
            translation=pos
        ).homogeneous
        assert(mat is not None)
        self._vis.viewer["zmp"].set_transform(mat)

    def _visualizeForce(self,
                       pos : npt.NDArray,
                       orientation : Rotation,
                       magnitude : float,
                       name : str):
        poseMat = pin.SE3(
                rotation=orientation.as_matrix(),
                translation=pos
                ).homogeneous
        assert(poseMat is not None)
        scaleMat = np.diag([magnitude,magnitude,magnitude,1])
        breakpoint()
        self._vis.viewer[name].set_transform(poseMat)

    def qId(self, jointName : str) -> int:
        assert(self.model.existJointName(jointName))

        idx_qs = self.model.idx_qs
        assert(idx_qs is not None)
        idx_qs = idx_qs.tolist()

        jointId = self.model.getJointId(jointName)
        return idx_qs[jointId]

    def visualizeJointConfig(self,
                             q : npt.NDArray,
                             qdot : npt.NDArray,
                             tau : npt.NDArray):
        self._vis.display(q)

        # ZMP
        func = c.Function("f_zmp",
                   [self.q, self.qdot, self.tau],
                   [self.ZMP])
        zmp = c.DM(func(q, qdot, tau)).toarray()
        assert(type(zmp) is np.ndarray)
        self._visualizeZMP(zmp)

        # contact forces
        func_F_l = c.Function("f_F_l",
                              [self.q, self.qdot, self.tau],
                              [self.contactForces[:6]])
        F_l = c.DM(func_F_l(q, qdot, tau))

        func_F_r = c.Function("f_F_r",
                              [self.q, self.qdot, self.tau],
                              [self.contactForces[6:]])
        F_r = c.DM(func_F_r(q, qdot, tau))
        print("F_l:", F_l)
        print("F_r:", F_r)
        print()

        # self._visualizeForce()

    def visualizeJointTrajecotry(self,
                                 q_arr : npt.NDArray,
                                 qdot_arr : npt.NDArray,
                                 tau_arr : npt.NDArray,
                                 t_arr : npt.NDArray,
                                 timeMultiplier : float = 1.0):
        t_last = 0.0
        for q, qdot, tau, t in zip(q_arr, qdot_arr, tau_arr, t_arr):
            self.visualizeJointConfig(q, qdot, tau)
            sleep((t - t_last) * timeMultiplier)
            t_last = t
            # print(t, q)

    def saveJointTrajectory(self,
                            q : np.ndarray,
                            fileName : Path):
        file = open(fileName, 'w')
        writer = CsvWriter(file)
        assert(self.model.names)

         # remove floating-base
        jointNames = self.model.names.tolist()[2:]
        jointValues = q[:,6:]

        writer.writerow(jointNames)
        writer.writerows(jointValues)