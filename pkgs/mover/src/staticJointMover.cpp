#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#include <fstream>
#include <memory>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float32.hpp>
#include <unordered_map>
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
#define NAME_MODE 0
#define POSITION_MODE 1

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

bool jointPositionsFromTxt(const std::string &filePath,
                           std::vector<std::string> &outNames,
                           std::vector<double> &outPositions) {
    std::ifstream file(filePath);
    if (!file) {
        return false;
    }

    std::string line;
    uint mode = -1;
    while (std::getline(file, line) &&
           (outNames.size() == 0 || outPositions.size() < outNames.size())) {
        if (line.find("name") == 0) {
            mode = NAME_MODE;
            continue;
        } else if (line.find("position") == 0) {
            mode = POSITION_MODE;
            outPositions.reserve(outNames.size());
            continue;
        }

        line = line.substr(2);
        switch (mode) {
            case NAME_MODE:
                outNames.push_back(line);
                break;
            case POSITION_MODE:
                outPositions.push_back(std::stod(line));
                break;
            default:
                break;
        }
    }
    file.close();
    return true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::vector<std::string> names;
    std::vector<double> positions;
    if (!jointPositionsFromTxt(argv[1], names, positions)) return -1;
    auto jointMover =
        std::make_shared<JointMoverNode>(names, "/h1/", "/cmd_pos", 1, 0.01);
    for (uint i = 0; i < names.size(); i++) {
        jointMover->update(names[i], positions[i]);
    }

    auto posNode = std::make_shared<rclcpp::Node>("PoS_node");
    float stability = 1.0;
    auto stabilitySub = posNode->create_subscription<std_msgs::msg::Float32>(
        "/stability", 10,
        [&stability](std_msgs::msg::Float32 m) { stability = m.data; });
    while (rclcpp::ok()) {
        rclcpp::spin_some(jointMover);
        rclcpp::spin_some(posNode);
        if (stability < 0.05) {
            RCLCPP_INFO(posNode->get_logger(), "!!! UNSTABLE CONFIGURATION !!! %f", stability);
            rclcpp::shutdown();
            return -1;
        }
    }

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
