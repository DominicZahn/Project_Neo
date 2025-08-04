#include <math.h>
#include <rbdl/rbdl.h>

#include <cstdio>
#include <iostream>
#include <nlopt.hpp>

using namespace RigidBodyDynamics;

typedef struct {
    double a, b;
} ConstraintData;

double objFunc(uint n, const double *x, double *grad, void *objFuncData) {
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

double constraintFunc(uint n, const double *x, double *grad,
                      void *constraintData) {
    ConstraintData *d = (ConstraintData *)constraintData;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a * x[0] + b) * (a * x[0] + b);
        grad[1] = -1.0;
    }
    return ((a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1]);
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    // RBDL check
    rbdl_check_api_version(RBDL_API_VERSION);
    rbdl_print_version();

    // nlopt example
    double lowerBounds[2] = {-HUGE_VAL, 0};  // lower bounds for a and b
    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LD_MMA, 2);
    nlopt_set_lower_bounds(opt, lowerBounds);
    nlopt_set_min_objective(opt, objFunc, NULL);

    ConstraintData constraintData[2] = {{2, 0}, {-1, 1}};
    nlopt_add_inequality_constraint(opt, constraintFunc, &constraintData[0],
                                    1e-8);
    nlopt_add_inequality_constraint(opt, constraintFunc, &constraintData[1],
                                    1e-8);

    nlopt_set_xtol_rel(opt, 1e-4);  // tolerance for objective function

    double x[2] = {1.234, 5.678};  // inital guess for a and b
    double minf;                   // minimum objective value upon return
    if (nlopt_optimize(opt, x, &minf) < 0)
        std::cout << "nlopt failed!!" << std::endl;
    else
        std::cout << "found minimum at (" << x[0] << " | " << x[1]
                  << ") = " << minf << std::endl;
    return 0;
}
