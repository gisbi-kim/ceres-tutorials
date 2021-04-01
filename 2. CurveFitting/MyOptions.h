#include "ceres/ceres.h"
#include "ceres/loss_function.h"

#include <vector>
#include <string>

using std::cout; 
using std::endl; 
using std::string;
// using ceres::internal::StringPrintf;

// see here for details 
//  http://ceres-solver.org/nnls_solving.html?highlight=options#solver-options
void setSolverOptions(ceres::Solver::Options& _options)
{
  _options.minimizer_type = ceres::TRUST_REGION; // TRUST_REGION or LINE_SEARCH
  _options.max_num_iterations = 100;
  _options.linear_solver_type = ceres::DENSE_QR; // DENSE_QR or SPARSE_NORMAL_CHOLESKY
  _options.function_tolerance = 1e-7;
  _options.minimizer_progress_to_stdout = true;

  // _options.update_state_every_iteration = true;
}

struct RememberingCallback : public ceres::IterationCallback 
{
  explicit RememberingCallback(double* x) : calls(0), x(x) {}
  virtual ~RememberingCallback() {}
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) final {
    x_values.push_back(*x);
    calls++;
    // cout << summary.cost << endl;
    return ceres::SOLVER_CONTINUE;
  }
  int calls;
  double* x;
  std::vector<double> x_values;
};
