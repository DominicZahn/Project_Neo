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
        std::make_shared<JointMoverNode>(names, "/h1/", "/cmd_pos");
    rclcpp::spin_some(jointMover);
    std::unordered_map<std::string, double> lastJointStates =
        jointMover->getCurrentJointStates();

    auto posNode = std::make_shared<rclcpp::Node>("PoS_node");
    float stability = 1.0;
    auto stabilitySub = posNode->create_subscription<std_msgs::msg::Float32>(
        "/stability", 10,
        [&stability](std_msgs::msg::Float32 m) { stability = m.data; });
    while (stability > 0.01) {
        rclcpp::spin_some(jointMover);
        if (jointMover->getNumberOfMovingJoints() == 0) {
            for (uint i = 0; i < names.size(); i++) {
                const double delta = positions[i] - lastJointStates[names[i]];
                const double newPosition =
                    lastJointStates[names[i]] + delta * 0.05;
                jointMover->update(names[i], newPosition);
                lastJointStates[names[i]] = newPosition;
            }
            jointMover->execute();
            rclcpp::sleep_for(10ms);
        }

        rclcpp::spin_some(posNode);
    }

    rclcpp::shutdown();
    return 0;
}
