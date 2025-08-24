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
#define stabilityCriteria(CoM, PoS) (neo_utils::Stability::distCentroid(CoM, PoS))
typedef neo_utils::RBDLWrapper::JointLimit JointLimit;

struct ObjFuncData {
    neo_utils::RBDLWrapper *rbdl;
    Polygon PoS;
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

double objectiveFunc(const std::vector<double> &q, std::vector<double> &grad,
                     void *data) {
    (void)grad;  // grad is not needed because of COBYLA
    auto objData = reinterpret_cast<ObjFuncData *>(data);

    // ---------- DEBUG for visualization ----------
    objData->rbdl->publishJoints(q);
    //  ---------------------------------------------

    auto model = *objData->rbdl->get_model();
    Polygon PoS = objData->PoS;
    VectorNd qVec = VectorNd::Zero(q.size());
    for (int i = 0; i < q.size(); i++) {
        qVec[i] = q[i];
    }
    VectorNd qDotVec = VectorNd::Zero(q.size());
    qDotVec = objData->rbdl->get_qdot();
    Vector3d CoM = Vector3d::Zero();
    Scalar totalMass = 0.0;
    RigidBodyDynamics::Utils::CalcCenterOfMass(
        model, qVec, qDotVec, NULL, totalMass, CoM);
    Point CoMpt = Point(CoM[0], CoM[1]);
    double stability = stabilityCriteria(CoMpt, PoS);
    return stability;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    neo_utils::RBDLWrapper *rbdlWrapper = new neo_utils::RBDLWrapper();
    // put robot into default position
    std::vector<double> q_0(rbdlWrapper->get_q().size(), 0.0);
        rbdlWrapper->publishJoints(q_0);
    //

    const int q_size = rbdlWrapper->get_q().size();
    std::vector<double> q_lb(q_size);
    std::vector<double> q_ub(q_size);
    std::vector<double> q_opt(q_size);
    std::vector<JointLimit> jointLimits = rbdlWrapper->get_qLimits();
    for (int i = 0; i < q_size; i++) {
        q_lb[i] = jointLimits[i].q_min;
        q_ub[i] = jointLimits[i].q_max;
        q_opt[i] = q_0[i];
    }

    ObjFuncData objFuncData;
    objFuncData.rbdl = rbdlWrapper;

    // hardcoded PoS from default position
    std::vector<Point> PoS_pts = {Point(-0.07, 0.2), Point(-0.07, -0.2),
                                  Point(0.16, 0.2), Point(0.16, -0.2)};
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
    
    globalOpt.set_ftol_abs(1e-8);
    globalOpt.set_xtol_abs(1e-8);
    //globalOpt.set_maxtime(30);
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
