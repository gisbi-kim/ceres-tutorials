#pragma once

#include "ceres/ceres.h"
#include "ceres/loss_function.h"

namespace simplebal {

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    T r2 = xp * xp + yp * yp;
    T distortion = 1.0 + r2 * (l1 + l2 * r2);

    // Compute final projected point position.
    const T& focal = camera[6];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<
          SnavelyReprojectionError, 
          2 /* kNumResiduals */,
          9 /* Size of each 1th parameter block (i.e., camera params: -R, t(x,y,z), f, k1, k2, R is Rodrigues vector, thus 9) */, 
          3 /* Size of each 2th parameter block (i.e., landmark params: x, y, z, thus 3) */ 
          >(new SnavelyReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
}; // struct SnavelyReprojectionError

ceres::CostFunction* genSnavelyReprojectionError(double _x, double _y) {
  //  return ( new ceres::AutoDiffCostFunction<MyExponentialResidual, 1, 1, 1>(new MyExponentialResidual(_x, _y)) );
  // return SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
  return SnavelyReprojectionError::Create(_x, _y);
}

} // namespace simplebal
