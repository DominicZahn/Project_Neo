#include <cstdio>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <nlopt.hpp>

#include "neo_utils/rbdlWrapper.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    neo_utils::RBDLWrapper rbdlWrapper = neo_utils::RBDLWrapper();

    printf("hello world dodge_it package\n");
    rclcpp::shutdown();
    return 0;
}
