#ifndef SAFETY_FILTER_NODE_CONFIG_H
#define SAFETY_FILTER_NODE_CONFIG_H

#include <array>
#include <vector>
#include <string>
#include "acados_solver_agilex.h"


namespace safety_filter_mpc
{

struct SafetyFilterNodeConstraints {
    // Input Bounds
    std::array<double, AGILEX_NBU> lbu{};
    std::array<double, AGILEX_NBU> ubu{};
};

struct SafetyFilterNodeCollisions {
    double radius{0.0};
    double front_offset{0.0};
    double back_offset{0.0};
};

struct SafetyFilterNodeConfig {
    SafetyFilterNodeCollisions    collisions{};
    SafetyFilterNodeConstraints   constraints{};
    double                        ts{0.0};
    double                        timeout{0.0};
};

}

#endif // SAFETY_FILTER_NODE_CONFIG_H