#pragma once

#include <vector>
#include <string>

#include "ceres/ceres.h"
#include "ceres/loss_function.h"

#include "SimpleBAL/BALManager.h"

using std::cout; 
using std::endl; 
using std::string;
// using ceres::internal::StringPrintf;

namespace simplebal {

// see here for details 
//  http://ceres-solver.org/nnls_solving.html?highlight=options#solver-options
void setSolverOptions(ceres::Solver::Options& _options)
{
  _options.minimizer_progress_to_stdout = true;

  _options.minimizer_type = ceres::TRUST_REGION; // TRUST_REGION or LINE_SEARCH
  // _options.linear_solver_type = ceres::DENSE_QR; // DENSE_QR or SPARSE_NORMAL_CHOLESKY
  _options.linear_solver_type = ceres::DENSE_SCHUR; // for this BA problem, use DENSE_SCHUR, details: Bundle Adjustment in the Large paper (ECCV 2010, http://grail.cs.washington.edu/projects/bal/bal.pdf)

  _options.max_num_iterations = 200;
  _options.function_tolerance = 1e-7;

  // _options.update_state_every_iteration = true;
}

struct WritingMidResultsCallback : public ceres::IterationCallback 
{
public:
  explicit WritingMidResultsCallback(simplebal::BALManager& _balManager) 
  : balManager(_balManager) 
  { 
    balManager.writeResultFile(); 
  }

  virtual ~WritingMidResultsCallback() {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) final {
    balManager.writeResultFile(iterCounter);
    cout << "     current iteration's solution saved (landmarks points only). " << endl;
    iterCounter++;
    return ceres::SOLVER_CONTINUE;
  }

public:
  simplebal::BALManager& balManager;
  int iterCounter {0};
};


} // namespace simplebal

