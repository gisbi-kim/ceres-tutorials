#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "SimpleBAL/OptionConfig.h"
#include "SimpleBAL/BALManager.h"
#include "SimpleBAL/Residual.h"


int main(int argc, char** argv) 
{
  // prepare the data from here: http://grail.cs.washington.edu/projects/bal/ladybug.html (homepage: http://grail.cs.washington.edu/projects/bal/)
  google::InitGoogleLogging(argv[0]);
  if (argc != 2) {
    std::cerr << "how to use: e.g., $ ./build/main data/problem-49-7776-pre.txt\n"; 
    return 1;
  }

  // about the BAL details, see the Bundle Adjustment in the Large paper (ECCV 2010, http://grail.cs.washington.edu/projects/bal/bal.pdf)
  simplebal::BALManager bal;
  if (!bal.loadFile(argv[1])) {
    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
    return 1;
  }

  std::stringstream ss; ss << argv[1] <<  ".result.txt";
  std::string resultFilePath = ss.str();
  bal.writeResultFile(resultFilePath);
  
  // Create residuals for each observation in the bundle adjustment problem. The parameters for cameras and points are added automatically.
  ceres::Problem problem;
  const double* observations = bal.observations(); // NOTE that it is const 
  for (int i = 0; i < bal.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    auto cost_function = simplebal::genSnavelyReprojectionError(observations[2*i + 0], observations[2*i + 1]);
    problem.AddResidualBlock(cost_function,
                             NULL, /* squared loss or use robust loss: "new ceres::CauchyLoss(0.5)", note but robust kernel would delay the convergence */
                             bal.mutable_camera_for_observation(i),
                             bal.mutable_point_for_observation(i));
  }

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Summary summary;

  ceres::Solver::Options options;
  simplebal::setSolverOptions(options);

  options.update_state_every_iteration = true;
  simplebal::WritingMidResultsCallback my_callback(bal);
  options.callbacks.push_back(&my_callback);

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << "\n";

  return 0;
}