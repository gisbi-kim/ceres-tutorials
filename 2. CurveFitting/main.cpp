#include <iostream>

#include "ceres/ceres.h"
#include "ceres/version.h"
#include "ceres/loss_function.h"
#include "glog/logging.h"

#include "Residuals.h"
#include "MyOptions.h"

#include "data.h"

int main(int argc, char** argv) 
{
  //
  std::cout << "\nUsing Ceres veresion: " << CERES_VERSION_STRING << std::endl;
  google::InitGoogleLogging(argv[0]);


  // load data (in data.h)
  std::cout << "The number of input data measurements: " << kNumObservations << std::endl;


  // init problem 
  double m_init = 1.0; 
  double c_init = 1.0;
  
  double m {m_init};   
  double c {c_init}; 
  ceres::Problem problem;
  for (int i = 0; i < kNumObservations; ++i) 
  {
    auto cost_function = genMyExponentialResidualBlock(data[2 * i], data[2 * i + 1]);
   
    // problem.AddResidualBlock(cost_function, nullptr, &m, &c); // non-robust ver 
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1), &m, &c); // robust ver 
  }


  // set options 
  ceres::Solver::Summary summary;

  ceres::Solver::Options options;
  setSolverOptions(options);

  double x = 0;
  RememberingCallback my_callback(&x);
  options.callbacks.push_back(&my_callback);


  // solve 
  ceres::Solve(options, &problem, &summary);


  // check the results 
  std::cout << summary.BriefReport() << "\n\n";
  std::cout << "Initial m: " << m_init << " c: " << c_init << "\n";
  std::cout << "Final   m: " << m << " c: " << c << "\n";
  std::cout << "GT      m: " << 0.3 << " c: " << 0.1 << "\n";

  for(auto & _elm: my_callback.x_values) {
    cout << _elm << " - num total iterations: " << my_callback.calls << endl;
  }

  return 0;
}
