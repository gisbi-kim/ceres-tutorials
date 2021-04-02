#pragma once

#include "RobotPose1D/Configurations.h"

namespace rp1 { // robot-pose-1d

void SimulateRobot(std::vector<double>* odometry_values, std::vector<double>* range_readings) 
{
  const int num_steps = static_cast<int>( ceil(CERES_GET_FLAG(FLAGS_corridor_length) / CERES_GET_FLAG(FLAGS_pose_separation)) );
  double robot_location = 0.0;   // The robot starts out at the origin.
  for (int i = 0; i < num_steps; ++i)
  {
    const double actual_odometry_value = min( CERES_GET_FLAG(FLAGS_pose_separation), (CERES_GET_FLAG(FLAGS_corridor_length) - robot_location) );
    robot_location += actual_odometry_value;
    const double actual_range = CERES_GET_FLAG(FLAGS_corridor_length) - robot_location;
    const double observed_odometry = rp1::RandNormal() * CERES_GET_FLAG(FLAGS_odometry_stddev) + actual_odometry_value;
    const double observed_range = rp1::RandNormal() * CERES_GET_FLAG(FLAGS_range_stddev) + actual_range;
    odometry_values->push_back(observed_odometry);
    range_readings->push_back(observed_range);
  }
} // func: SimulateRobot


void PrintState(const std::vector<double>& odometry_readings, const std::vector<double>& range_readings) 
{
  CHECK_EQ(odometry_readings.size(), range_readings.size());
  
  double robot_location = 0.0;
  printf("pose: location     odom    range  r.error  o.error\n");
  
  for (int i = 0; i < odometry_readings.size(); ++i) 
  {
    robot_location += odometry_readings[i];
    const double range_error = robot_location + range_readings[i] - CERES_GET_FLAG(FLAGS_corridor_length);
    const double odometry_error = CERES_GET_FLAG(FLAGS_pose_separation) - odometry_readings[i];
    printf("%4d: %8.3f %8.3f %8.3f %8.3f %8.3f\n",
           static_cast<int>(i),
           robot_location,
           odometry_readings[i],
           range_readings[i],
           range_error,
           odometry_error);
  }
} // func: PrintState

} // namespace rp1