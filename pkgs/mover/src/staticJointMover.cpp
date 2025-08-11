#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <memory>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL Addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
#include <math.h>

#include <nlopt.hpp>

using namespace std::chrono_literals;

#include "jointMoverNode.cpp"

// typedef struct
//{
//     double a, b;
// } ConstraintData;
//
// double objFunc(uint n, const double *x, double *grad, void *objFuncData)
//{
//     if (grad)
//     {
//         grad[0] = 0.0;
//         grad[1] = 0.5 / sqrt(x[1]);
//     }
//     return sqrt(x[1]);
// }
//
// double constraintFunc(uint n, const double *x, double *grad,
//                       void *constraintData)
//{
//     ConstraintData *d = (ConstraintData *)constraintData;
//     double a = d->a, b = d->b;
//     if (grad)
//     {
//         grad[0] = 3 * a * (a * x[0] + b) * (a * x[0] + b);
//         grad[1] = -1.0;
//     }
//     return ((a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1]);
// }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::vector<std::string> topicNameList = {"left_ankle_pitch_joint",
                                              "left_ankle_roll_joint",
                                              "left_elbow_joint",
                                              "left_hip_pitch_joint",
                                              "left_hip_roll_joint",
                                              "left_hip_yaw_joint",
                                              "left_knee_joint",
                                              "left_shoulder_pitch_joint",
                                              "left_shoulder_roll_joint",
                                              "left_shoulder_yaw_joint",
                                              "right_ankle_joint",
                                              "right_ankle_pitch_joint",
                                              "right_ankle_roll_joint",
                                              "right_elbow_joint",
                                              "right_hip_pitch_joint",
                                              "right_hip_roll_joint",
                                              "right_hip_yaw_joint",
                                              "right_knee_joint",
                                              "right_shoulder_pitch_joint",
                                              "right_shoulder_roll_joint",
                                              "right_shoulder_yaw_joint",
                                              "torso_joint"};
    auto jointMover =
        std::make_shared<JointMoverNode>(topicNameList, "/h1/", "/cmd_pos");
    jointMover->update(topicNameList[2], 0.5);
    rclcpp::spin(jointMover);

    // RBDL check
    // rbdl_check_api_version(RBDL_API_VERSION);

    // auto readerNode = std::make_shared<rclcpp::Node>("urdf_reader");
    // auto paramClientNode = std::make_shared<rclcpp::SyncParametersClient>(
    //     readerNode, "robot_state_publisher");
    // while (!paramClientNode->wait_for_service(std::chrono::seconds(1))) {
    // }
    // const std::string urdfStr =
    //     paramClientNode->get_parameter<std::string>("robot_description");
    // Model *model = new Model();
    // Addons::URDFReadFromString(urdfStr.c_str(), model, false);
    // if (!model) {
    //     std::cerr << "Model could not be read by rbdl!" << std::endl;
    //     return -1;
    // }
    // std::cout << "Degree of freedom overview:" << std::endl;
    // std::cout << Utils::GetModelDOFOverview(*model);

    //// nlopt example
    // double lowerBounds[2] = {-HUGE_VAL, 0};  // lower bounds for a and b
    // nlopt_opt opt;
    // opt = nlopt_create(NLOPT_LD_MMA, 2);
    // nlopt_set_lower_bounds(opt, lowerBounds);
    // nlopt_set_min_objective(opt, objFunc, NULL);

    // ConstraintData constraintData[2] = {{2, 0}, {-1, 1}};
    // nlopt_add_inequality_constraint(opt, constraintFunc, &constraintData[0],
    //                                 1e-8);
    // nlopt_add_inequality_constraint(opt, constraintFunc, &constraintData[1],
    //                                 1e-8);

    // nlopt_set_xtol_rel(opt, 1e-4);  // tolerance for objective function

    // double x[2] = {1.234, 5.678};  // inital guess for a and b
    // double minf;                   // minimum objective value upon return
    // if (nlopt_optimize(opt, x, &minf) < 0)
    //     std::cout << "nlopt failed!!" << std::endl;
    // else
    //     std::cout << "found minimum at (" << x[0] << " | " << x[1]
    //               << ") = " << minf << std::endl;

    rclcpp::shutdown();
    return 0;
}
