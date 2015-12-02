#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <armadillo>


int local_bundle_adjustment(arma::fmat K,
							arma::fmat& poseA,
							arma::fmat& poseB,
							arma::fmat& point2DA,
							arma::fmat& point2DB,
							arma::fmat& point4D);

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
		//  T* R = new T[9];
		// ceres::AngleAxisToRotationMatrix(camera,R);
		//
		// for (size_t i = 0; i < 9; i++) {
		// 	/* code */
		// 	std::cout<<R[i]<<" |";
		// }
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp =  p[0] / p[2];
    T yp =  p[1] / p[2];
    // Compute final projected point position.
    const T& focal = T(1520.4);
    const T& dx = T(302.32);
    const T& dy = T(246.87);
    T predicted_x = T(1520.4) * xp + dx;
    T predicted_y = T(1525.9) * yp + dy;
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
		std::cout<<"error 0: "<<residuals[0]<<std::endl;
		std::cout<<"error 1: "<<residuals[1]<<std::endl;

    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
  }
  double observed_x;
  double observed_y;
};
