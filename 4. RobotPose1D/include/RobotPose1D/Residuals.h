#pragma once

#include "RobotPose1D/Configurations.h"

namespace rp1 { // robot-pose-1d

struct OdometryConstraint 
{
public: 
  typedef ceres::AutoDiffCostFunction<OdometryConstraint, 1, 1> 
      OdometryCostFunction;

public: 
  OdometryConstraint(double odometry_mean, double odometry_stddev)
      : odometry_mean(odometry_mean), odometry_stddev(odometry_stddev) {}

  template <typename T>
  bool operator()(const T* const odometry, T* residual) const 
  {
    *residual = (*odometry - odometry_mean) / odometry_stddev;
    return true;
  }

  static OdometryCostFunction* Create(const double odometry_value) 
  {
    return new OdometryCostFunction(new OdometryConstraint(
        odometry_value, CERES_GET_FLAG(FLAGS_odometry_stddev)));
  }

public: 
  const double odometry_mean;
  const double odometry_stddev;
}; // OdometryConstraint


struct RangeConstraint 
{
public: 
  typedef ceres::DynamicAutoDiffCostFunction<RangeConstraint, rp1::kStride>
      RangeCostFunction;

public: 
  RangeConstraint(int pose_index,
                  double range_reading,
                  double range_stddev,
                  double corridor_length)
      : pose_index(pose_index),
        range_reading(range_reading),
        range_stddev(range_stddev),
        corridor_length(corridor_length) {}

  template <typename T>
  bool operator()(T const* const* relative_poses, T* residuals) const {
    T global_pose(0);
    for (int i = 0; i <= pose_index; ++i) {
      global_pose += relative_poses[i][0];
    }
    residuals[0] = (global_pose + range_reading - corridor_length) / range_stddev;
    return true;
  }

  // Factory method to create a CostFunction from a RangeConstraint to
  // conveniently add to a ceres problem.
  static RangeCostFunction* Create(const int pose_index,
                                   const double range_reading,
                                   std::vector<double>* odometry_values,
                                   std::vector<double*>* parameter_blocks) {
    RangeConstraint* constraint =
        new RangeConstraint(pose_index,
                            range_reading,
                            CERES_GET_FLAG(FLAGS_range_stddev),
                            CERES_GET_FLAG(FLAGS_corridor_length));

    RangeCostFunction* cost_function = new RangeCostFunction(constraint);

    // Add all the parameter blocks that affect this constraint.
    parameter_blocks->clear();
    for (int i = 0; i <= pose_index; ++i) {
      parameter_blocks->push_back(&((*odometry_values)[i]));
      cost_function->AddParameterBlock(1);
    }

    cost_function->SetNumResiduals(1);

    return (cost_function);
  }

public: 
  const int pose_index;
  const double range_reading;
  const double range_stddev;
  const double corridor_length;
}; // RangeConstraint

} // namespace rp1