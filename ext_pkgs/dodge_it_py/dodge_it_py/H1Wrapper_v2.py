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
from rich import print
from dataclasses import dataclass
from pathlib import Path
import cv2
from playwright.sync_api import sync_playwright

from ext_pkgs.dodge_it_py.dodge_it_py.stability import (
  zmp_centroidal, zmp_full
)
from ext_pkgs.dodge_it_py.dodge_it_py.collisionSDF import CollisionSDF, CollisionSimplex

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
# -------------- EXPERIMENTAL CONTROLS --------------
TAU_STAND = dict({
    'left_hip_yaw_joint': 0.03796477759770445,
    'left_hip_pitch_joint': 3.2653362685509135,
    'left_hip_roll_joint': -0.38801762511388915,
    'left_knee_joint': -37.09299323044395,
    'left_ankle_pitch_joint': 10.334703368649418,
    'left_ankle_roll_joint': -0.12570678970096427,
    'right_hip_yaw_joint': -0.03660783790805394,
    'right_hip_pitch_joint': 3.2649740053192255,
    'right_hip_roll_joint': -0.005022745811303936,
    'right_knee_joint': -37.068742354522044,
    'right_ankle_pitch_joint': 10.322914141126112,
    'right_ankle_roll_joint': -0.30208761181502863,
    'torso_joint': -1.1657341758564144e-15,
    'left_shoulder_pitch_joint': -3.124021972677831,
    'left_shoulder_roll_joint': 0.02950822493999855,
    'left_shoulder_yaw_joint': 1.1032841307212493e-15,
    'left_elbow_joint': -2.9655000492300005,
    'left_wrist_roll_joint': -0.015256825919999354,
    'left_wrist_pitch_joint': -0.15518525327999977,
    'left_wrist_yaw_joint': -4.163336342344337e-16,
    'right_shoulder_pitch_joint': -3.12402197267783,
    'right_shoulder_roll_joint': -0.0602554332599991,
    'right_shoulder_yaw_joint': 2.220446049250313e-16,
    'right_elbow_joint': -2.96550004923,
    'right_wrist_roll_joint': -0.015490382400001446,
    'right_wrist_pitch_joint': -0.15518525328000016,
    'right_wrist_yaw_joint': 0.0
})
# ------------- COLLISION ELLIPSOIDS -----------------
COLLISION = dict({
    #    NAME                  |          RADIUS         |          T_OFFSET
    'lidar_link':               (c.SX([0.10,  0.10,  0.15]), c.SX([0.0,   0.0,  0.035])),
    'torso_link':               (c.SX([0.12,  0.12,  0.35]), c.SX([0.00,  0.00, -0.25])),
    'left_shoulder_pitch_link':   (c.SX([0.06,  0.12,  0.06]), c.SX([0.00,  0.00,  0.00])),
    'left_shoulder_yaw_link':   (c.SX([0.06,  0.06,  0.24]), c.SX([0.00,  0.00,  0.02])),
    'left_wrist_roll_link':     (c.SX([0.20,  0.06,  0.06]), c.SX([0.00,  0.00,  0.00])),
    'right_shoulder_pitch_link':   (c.SX([0.06,  0.12,  0.06]), c.SX([0.00,  0.00,  0.00])),
    'right_shoulder_yaw_link':  (c.SX([0.06,  0.06,  0.24]), c.SX([0.00,  0.00,  0.02])),
    'right_wrist_roll_link':    (c.SX([0.20,  0.06,  0.06]), c.SX([0.00,  0.00,  0.00])),

})
# --------------------- HEADLESS ---------------------
@dataclass
class HeadlessData:
    dir : str
    camPos : npt.NDArray
    camLookAt : npt.NDArray
    resolution : tuple[int,int]

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
                 feetFrames : list[str] = DEFAULT_FEET_FRAMES,
                 showCollisionSDF : bool = False,
                 headlessData : HeadlessData | None = None):
        self._setupModels()
        self._setInitalPose(q0)
        self._fixJoints(dynamicJoints)
        self._setupContacts(feetFrames)
        self._initCasadi()
        self._setupVis(showCollisionSDF, headlessData)

    def setCollision(self,
             projectileFunc : c.Function):
        self._setupCollision()
        self._setupProjectile(projectileFunc)
        
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

    def _setupCollision(self):
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.q)
        simplexList = []
        for frame, (radius, offset) in COLLISION.items():
            assert(self.model.existFrame(frame))
            frameId = self.model.getFrameId(frame)
            localToWorld = self.cdata.oMf[frameId]
            worldToLocal = localToWorld.inverse()
            worldToLocalOffset = cpin.SE3(
                worldToLocal.rotation,
                worldToLocal.translation + offset
            ).homogeneous
            simplex = CollisionSimplex(worldToLocalOffset, radius)
            simplexList.append(simplex)

        self.collisionSDF = CollisionSDF(
            simplexList,
            self.q0,
            1e-2,
            self.q)
        if self.showCollisionSDF:
            self.collisionSDF.enableVis(self._vis, 96, 2.0)
    
    def _setInitalPose(self, q0 : npt.NDArray[np.float32] | str | None):
        pin.loadReferenceConfigurations(self.model, SRDF_FULL_PATH, verbose=False)
        assert(self.model.referenceConfigurations is not None)
        assert(self.model.referenceConfigurations[DEFAULT_REFERENCE_CONF] is not None)
        
        if type(q0) is str:
            q0 = self.model.referenceConfigurations[q0]
        assert(type(q0) is np.ndarray)
        assert(q0.size == self.model.nq)
        self.q0 = q0

    def _setupProjectile(self, projectileFunc : c.Function):
        self._projectileFunc = projectileFunc

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
        
        self.t = c.SX.sym("t", 1)

        # define dynamics
        proxSettings = cpin.ProximalSettings(None, 1e-12, 10)
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
        self.ZMP = zmp_centroidal(
            self.cmodel,
            self.cdata,
            self.q,
            self.qdot,
            self.qddot)

        # CoM
        self.CoM = cpin.centerOfMass(self.cmodel, self.cdata, self.q)

        # head
        assert(self.model.existFrame(HEAD_FRAME))
        headId = self.model.getFrameId(HEAD_FRAME)
        self.headPos = self.cdata.oMf[headId].translation

    def cProjectileRobotDistance(self, t : c.SX) -> c.SX:
        p_projectile = self._projectileFunc(t)
        assert(type(p_projectile) is c.SX)
        d = self.collisionSDF.cDistanceFunc(p_projectile, self.q)
        return d

    def _setupVis(self,
                  showCollisionSDF : bool,
                  headlessData : HeadlessData | None):
        self.showCollisionSDF = showCollisionSDF
        self._vis = MeshcatVisualizer(
            model=self.model,
            collision_model=self.collisionModel,
            visual_model=self.visualModel,
            copy_models=True
        )
        self._vis.initViewer(loadModel=True, open=False)

        # launch headless playwright browser
        if headlessData is not None:
            p = Path(headlessData.dir)
            if p.exists():
                print("[bold red][ERROR] output headless directory already exists![/bold red]")
                assert(False)
            p.mkdir()

            url = self._vis.viewer.url()
            playwright = sync_playwright().start()
            headlessBrowser = playwright.chromium.launch(
                headless=True,
                args = ["--use-gl=swiftshader",
                        "--enable-webgl",
                        "--ignore-gpu-blocklist"])
            assert(headlessBrowser.is_connected())
            print("[INFO] connected headless chromium")
            page = headlessBrowser.new_page(viewport={
                "width": headlessData.resolution[0],
                "height": headlessData.resolution[1]})
            page.goto(url)
            assert(not page.is_closed())
            print("[INFO] meshcat viewer opened")
        self.headlessData = headlessData

        self._vis.loadViewerModel()
        self._vis.setBackgroundColor("gray")

        self._vis.viewer["zmp"].set_object(
            g.Sphere(0.02),
            g.MeshPhongMaterial(0xf81802))
#        self._vis.viewer["F_l"].set_object(
#            g.Cylinder(1, 0.01),
#            g.MeshPhongMaterial(0xa2bb7d))
#        self._vis.viewer["F_r"].set_object(
#            g.Cylinder(1, 0.01),
#            g.MeshPhongMaterial(0xa2bb7d))
        self._vis.viewer["projectile"].set_object(
            g.Sphere(0.02),
            g.MeshPhongMaterial(0x5e4f49))

    def _visualizeZMP(self, pos : npt.NDArray):
        mat = pin.SE3(
            rotation=np.eye(3),
            translation=pos
        ).homogeneous
        assert(mat is not None)
        self._vis.viewer["zmp"].set_transform(mat)

    def _moveCamera(self,
                   pos: np.ndarray,
                   lookAt: np.ndarray,
                   up : npt.NDArray = np.array([0.,0.,1.])) -> None:
            camInvDirection = pos - lookAt
            camInvDirection /= np.linalg.norm(camInvDirection)
            camRight = np.cross(up, camInvDirection)
            camRight /= np.linalg.norm(camRight)
            camUp = np.cross(camInvDirection, camRight)

            R = np.eye(4)
            R[:3,:3] = np.column_stack([camRight, camUp, camInvDirection])
            T = np.eye(4)
            T[:3,3] = pos
            pose = R @ T
            assert(pose is not None)
            self._vis.setCameraPose(pose)

    def autoCapture(self,
                    outFile : str,
                    camPos : npt.NDArray,
                    camLookAt : npt.NDArray):
        self._moveCamera(camPos, camLookAt)
        imgRGB = self._vis.captureImage()
        imgBGR = cv2.cvtColor(imgRGB, cv2.COLOR_RGB2BGR)
        cv2.imwrite(outFile, imgBGR)

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

    def _visualizeProjectile(self,
                         pos : npt.NDArray):
        poseMat = pin.SE3(
            rotation=np.eye(3),
            translation=pos
        ).homogeneous
        assert(poseMat is not None)
        self._vis.viewer["projectile"].set_transform(poseMat)

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
                             tau : npt.NDArray,
                             t : float):
        self._vis.display(q)

        # ZMP
        func = c.Function("f_zmp",
                   [self.q, self.qdot, self.tau],
                   [self.ZMP])
        zmp = c.DM(func(q, qdot, tau)).toarray()
        assert(type(zmp) is np.ndarray)
        self._visualizeZMP(zmp)

        # collision SDF
        if self.showCollisionSDF:
            self.collisionSDF.updateJoints(q)
        # approaching projectile 
        pos = self._projectileFunc(t)
        assert(type(pos) is c.DM)
        d_func = c.Function("d1", [self.q], [self.cProjectileRobotDistance(c.SX(t))])
        d = d_func(q)
        color = "white" if d > 0.0 else "#ff0000"
        print(f"[{color}]d: {d}[/{color}]")
        pos = pos.toarray()
        self._visualizeProjectile(pos) # type: ignore

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
        for idx, q, qdot, tau, t in zip(range(len(q_arr)), q_arr, qdot_arr, tau_arr, t_arr):
            self.visualizeJointConfig(q, qdot, tau, t)
            if self.headlessData is None:
                sleep((t - t_last) * timeMultiplier)
                t_last = t
            else:
                fileName = f"{self.headlessData.dir}/{str(idx).zfill(5)}.png"
                self.autoCapture(fileName,
                                 np.array([1.0, 1.0, 1.5]),
                                 np.array([0.0, 0.0, 0.7]))

    def getStandControls(self) -> npt.NDArray:
        names = self.model.names.tolist()[2:]
        assert(names)
        tau = np.zeros(len(names))
        for name in names:
            i = self.qId(name) - 6
            tau[i] = TAU_STAND[name]
        return tau

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