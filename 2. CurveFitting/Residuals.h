#include "ceres/ceres.h"
#include "ceres/loss_function.h"

struct MyExponentialResidual {
  MyExponentialResidual(double x, double y) : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T* const m, const T* const c, T* residual) const {
    residual[0] = y_ - exp(m[0] * x_ + c[0]);
    return true;
  }

private:
  const double x_;
  const double y_;
};

ceres::CostFunction* genMyExponentialResidualBlock(double _x, double _y) {
   return ( new ceres::AutoDiffCostFunction<
              MyExponentialResidual, 
              1 /*kNumResiduals*/, 
              1 /* Size of each 1th parameter (i.e., m) block */,  
              1 /* Size of each 2th parameter (i.e., c) block */ >(new MyExponentialResidual(_x, _y)) );
}
