#pragma once

#include <math.h>
#include <cstdio>
#include <vector>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"

#include "RobotPose1D/random.h" // copied from internal/ceres/random.h

#ifndef CERES_GET_FLAG
#define CERES_GET_FLAG(X) X
#endif

using std::min;

DEFINE_double(corridor_length,
              10.0,
              "Length of the corridor that the robot is travelling down.");

DEFINE_double(pose_separation,
              0.5,
              "The distance that the robot traverses between successive "
              "odometry updates.");

DEFINE_double(odometry_stddev,
              0.1,
              "The standard deviation of odometry error of the robot.");

DEFINE_double(range_stddev,
              0.01,
              "The standard deviation of range readings of the robot.");


namespace rp1 {

    // The stride length of the dynamic_autodiff_cost_function evaluator.
    static constexpr int kStride = 10;

}