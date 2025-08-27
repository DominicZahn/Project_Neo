#include <math.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#include <boost/geometry.hpp>
#include <cstdio>
#include <nlopt.hpp>
#include <rclcpp/rclcpp.hpp>
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>
    Point;
typedef boost::geometry::model::polygon<Point> Polygon;

#include <rbdl/rbdl.h>
using namespace RigidBodyDynamics::Math;

#include "neo_utils/rbdlWrapper.hpp"
#include "neo_utils/stability.hpp"
#define stabilityCriteria(CoM, PoS) \
    (neo_utils::Stability::distCentroid(CoM, PoS))
typedef neo_utils::RBDLWrapper::JointLimit JointLimit;

#define GND_APPROX_FRAME "left_ankle_roll_link"

struct ObjFuncData {
    neo_utils::RBDLWrapper *rbdl;
    Polygon PoS;
};

struct RelBodyConstraintData {
    neo_utils::RBDLWrapper *rbdl;
    const std::string &bodyName;
    const std::string &refBodyName;
    Vector3d bodyPosInRef;
};

struct LimboConstraintData {
    neo_utils::RBDLWrapper *rbdl;
    const std::string &bodyName;
    const double z;
};

std::string nloptResult2Str(nlopt::result result) {
    switch (result) {
        // --- Success codes ---
        case nlopt::SUCCESS:
            return "SUCCESS: NLOPT_SUCCESS (Generic success)";
        case nlopt::STOPVAL_REACHED:
            return "SUCCESS: NLOPT_STOPVAL_REACHED (Stopval reached)";
        case nlopt::FTOL_REACHED:
            return "SUCCESS: NLOPT_FTOL_REACHED (ftol_rel or ftol_abs reached)";
        case nlopt::XTOL_REACHED:
            return "SUCCESS: NLOPT_XTOL_REACHED (xtol_rel or xtol_abs reached)";
        case nlopt::MAXEVAL_REACHED:
            return "SUCCESS: NLOPT_MAXEVAL_REACHED (maxeval reached)";
        case nlopt::MAXTIME_REACHED:
            return "SUCCESS: NLOPT_MAXTIME_REACHED (maxtime reached)";

        // --- Error codes ---
        case nlopt::FAILURE:
            return "ERROR: NLOPT_FAILURE (Generic failure)";
        case nlopt::INVALID_ARGS:
            return "ERROR: NLOPT_INVALID_ARGS (Invalid arguments)";
        case nlopt::OUT_OF_MEMORY:
            return "ERROR: NLOPT_OUT_OF_MEMORY (Ran out of memory)";
        case nlopt::ROUNDOFF_LIMITED:
            return "ERROR: NLOPT_ROUNDOFF_LIMITED (Roundoff errors limited "
                   "progress)";
        case nlopt::FORCED_STOP:
            return "ERROR: NLOPT_FORCED_STOP (Forced stop)";

        // --- Unknown code ---
        default:
            return "ERROR: Unknown nlopt::result code (" +
                   std::to_string(static_cast<int>(result)) + ")";
    }
}

long iterCounter = 0;
double objectiveFunc(const std::vector<double> &masked_q,
                     std::vector<double> &grad, void *data) {
    (void)grad;  // grad is not needed because of COBYLA / BOBYGA
    auto objData = reinterpret_cast<ObjFuncData *>(data);
    std::vector<double> unmasked_q = objData->rbdl->unmaskVec(masked_q, 0.0);
    const size_t q_size = unmasked_q.size();

    // ---------- DEBUG for visualization ----------
    objData->rbdl->publishJoints(masked_q);
    //  ---------------------------------------------

    auto model = *objData->rbdl->get_model();
    Polygon PoS = objData->PoS;
    VectorNd qVec = VectorNd::Zero(q_size);
    for (size_t i = 0; i < q_size; i++) {
        qVec[i] = unmasked_q[i];
    }
    VectorNd qDotVec = VectorNd::Zero(q_size);
    qDotVec = objData->rbdl->unmaskVec(objData->rbdl->get_qdot(), 0.0);
    Vector3d world_CoM = Vector3d::Zero();
    Scalar totalMass = 0.0;
    RigidBodyDynamics::Utils::CalcCenterOfMass(model, qVec, qDotVec, NULL,
                                               totalMass, world_CoM);
    Vector3d gnd_CoM =
        objData->rbdl->base2body(world_CoM, GND_APPROX_FRAME, qVec);
    Point CoMpt = Point(gnd_CoM[0], gnd_CoM[1]);
    double stability = stabilityCriteria(CoMpt, PoS);

    return stability;
}

double limboConstraint(const std::vector<double> &masked_q,
                            std::vector<double> &grad, void *data) {
    (void)grad;  // grad is not needed because of COBYLA / BOBYGA
    auto castedData = reinterpret_cast<LimboConstraintData *>(data);
    neo_utils::RBDLWrapper *rbdl = castedData->rbdl;
    const std::string &bodyName = castedData->bodyName;
    const double zInRef = castedData->z;

    std::vector<double> unmasked_q = rbdl->unmaskVec(masked_q, 0.0);
    VectorNd vec_q = VectorNd::Zero(unmasked_q.size());
    for (size_t i = 0; i < unmasked_q.size(); i++) {
        vec_q[i] = unmasked_q[i];
    }
    Vector3d bodyOriginInWorld =
        rbdl->body2base(Vector3d(0, 0, 0), bodyName, vec_q);
    Vector3d bodyOriginInRef =
        rbdl->base2body(bodyOriginInWorld, GND_APPROX_FRAME, vec_q);


    const double dz = bodyOriginInRef[2] + zInRef;
    std::cout << "\r" << std::setw(10) << iterCounter++ << ": " << dz
              << std::flush;

    return dz;
}

double relBodyConstraint(const std::vector<double> &masked_q,
                            std::vector<double> &grad, void *data) {
    (void)grad;  // grad is not needed because of COBYLA / BOBYGA
    auto castedData = reinterpret_cast<RelBodyConstraintData *>(data);
    neo_utils::RBDLWrapper *rbdl = castedData->rbdl;
    const std::string &bodyName = castedData->bodyName;
    const std::string &refBodyName = castedData->refBodyName;
    Vector3d posInRef = castedData->bodyPosInRef;

    std::vector<double> unmasked_q = rbdl->unmaskVec(masked_q, 0.0);
    VectorNd vec_q = VectorNd::Zero(unmasked_q.size());
    for (size_t i = 0; i < unmasked_q.size(); i++) {
        vec_q[i] = unmasked_q[i];
    }
    Vector3d bodyOriginInWorld =
        rbdl->body2base(Vector3d(0, 0, 0), bodyName, vec_q);
    Vector3d bodyOriginInRef =
        rbdl->base2body(bodyOriginInWorld, refBodyName, vec_q);

    double d = (bodyOriginInRef - posInRef).norm();

    std::cout << "\r" << std::setw(10) << iterCounter++ << ": " << d
              << std::flush;

    return d - 0.1;
}

double rightFeetConstraint(const std::vector<double> &masked_q,
                           std::vector<double> &grad, void *data) {
    (void)grad;  // grad is not needed because of COBYLA / BOBYGA
    neo_utils::RBDLWrapper *rbdl =
        reinterpret_cast<neo_utils::RBDLWrapper *>(data);
    std::vector<double> unmasked_q = rbdl->unmaskVec(masked_q, 0.0);
    VectorNd unmasked_qVec = VectorNd::Zero(unmasked_q.size());
    for (size_t i = 0; i < unmasked_q.size(); i++) {
        unmasked_qVec[i] = unmasked_q[i];
    }

    Vector3d baseRightFoot = rbdl->body2base(
        Vector3d(0, 0, 0), "right_ankle_roll_link", unmasked_qVec);
    Vector3d gndRightFoot =
        rbdl->base2body(baseRightFoot, "left_ankle_roll_link", unmasked_qVec);

    gndRightFoot[1] = 0.0;  // only offset in xz
    const double feetOffset = gndRightFoot.norm();
    return feetOffset;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    neo_utils::RBDLWrapper *rbdlWrapper = new neo_utils::RBDLWrapper();
    // put robot into default position
    std::vector<double> q_0(rbdlWrapper->get_q().size(), 0.0);
    q_0[rbdlWrapper->jointName2qIdx("left_hip_pitch_joint")] = 1.0;
    q_0[rbdlWrapper->jointName2qIdx("right_hip_pitch_joint")] = 1.0;
    q_0[rbdlWrapper->jointName2qIdx("left_knee_joint")] = 1.5;
    q_0[rbdlWrapper->jointName2qIdx("right_knee_joint")] = 1.5;
    q_0[rbdlWrapper->jointName2qIdx("right_ankle_pitch_joint")] = -0.8;
    q_0[rbdlWrapper->jointName2qIdx("left_ankle_pitch_joint")] = -0.8;
    rbdlWrapper->publishJoints(q_0);
    //

    rbdlWrapper->updateMask(
        {// --------- wrist ---------
         "right_wrist_roll_joint", "right_wrist_pitch_joint",
         "right_wrist_yaw_joint", "left_wrist_roll_joint",
         "left_wrist_pitch_joint", "left_wrist_yaw_joint",
         // --------- keep PoS const (parallel feet) -------
         "left_hip_yaw_joint", "left_hip_roll_joint", "right_hip_yaw_joint",
         "right_hip_roll_joint", "left_ankle_roll_joint",
         "right_ankle_roll_joint",
         // ---------- further limiting ----------
         "left_elbow_joint", "right_elbow_joint", "left_shoulder_pitch_joint",
         "right_shoulder_pitch_joint", "left_shoulder_roll_joint",
         "right_shoulder_roll_joint", "left_shoulder_yaw_joint",
         "right_shoulder_yaw_joint", "left_elbow_joint", "right_elbow_joint",
         "torso_joint",
         // ---------- right leg ----------
         //"right_hip_pitch_joint", "right_knee_joint",
         //"right_ankle_pitch_joint"
        });

    const int q_size = rbdlWrapper->get_jointNames().size();
    std::vector<double> q_lb(q_size);
    std::vector<double> q_ub(q_size);
    std::vector<double> q_opt = rbdlWrapper->maskVec(q_0);
    for (int i = 0; i < q_size; i++) {
        JointLimit jl = rbdlWrapper->get_jointLimit(i);
        q_lb[i] = jl.q_min;
        q_ub[i] = jl.q_max;
    }

    ObjFuncData objFuncData;
    objFuncData.rbdl = rbdlWrapper;

    // hardcoded PoS from default position
    std::vector<Point> PoS_pts = {Point(0.15, -0.35), Point(-0.05, 0.05),
                                  Point(-0.05, -0.35), Point(0.15, 0.05)};
    Polygon PoS_rng, PoS;
    for (Point p : PoS_pts) {
        bg::append(PoS_rng.outer(), p);
    }
    bg::convex_hull(PoS_rng, PoS);
    //

    objFuncData.PoS = PoS;

    nlopt::opt globalOpt(nlopt::LN_COBYLA, q_size);
    globalOpt.set_lower_bounds(q_lb);
    globalOpt.set_upper_bounds(q_ub);
    globalOpt.set_min_objective(objectiveFunc, &objFuncData);
    globalOpt.add_equality_constraint(rightFeetConstraint, rbdlWrapper,
    1e-3);
    LimboConstraintData headLimboData = { rbdlWrapper, "lidar_link", -0.166 };
    globalOpt.add_inequality_constraint(limboConstraint, &headLimboData, 1e-3);

    globalOpt.set_ftol_abs(1e-8);
    globalOpt.set_ftol_rel(-1);
    globalOpt.set_xtol_abs(1e-8);
    globalOpt.set_xtol_rel(-1);
    globalOpt.set_initial_step(1e-5);

    auto jointNames = rbdlWrapper->get_jointNames();
    double maxf = 0.0;
    try {
        nlopt::result result = globalOpt.optimize(q_opt, maxf);

        std::cout << "Configuration:" << std::endl;
        for (int i = 0; i < q_size; i++) {
            std::cout << std::setw(30) << jointNames[i] << ": " << q_opt[i]
                      << std::endl;
        }
        std::cout << std::endl
                  << " = " << std::setprecision(10) << maxf << std::endl;

        std::cout << nloptResult2Str(result) << std::endl;
    } catch (std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    auto errorJoints = rbdlWrapper->publishJoints(q_opt);
    if (!errorJoints.empty()) {
        std::cout << "Error Joints: " << std::endl;
        for (auto name : errorJoints) {
            std::cout << name << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}
