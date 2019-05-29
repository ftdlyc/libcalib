/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#pragma once
#ifndef CALIB_CERES_TYPE_H
#define CALIB_CERES_TYPE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

#include "libcalib/calib_def.h"

namespace calib {

CALIB_DLL_LOCAL class SE3Parameterization : public ceres::LocalParameterization {
public:
  bool Plus(const double* x,
            const double* delta,
            double* x_plus_delta) const;

  bool ComputeJacobian(const double* x, double* jacobian) const;

  bool MultiplyByJacobian(const double* x,
                          const int num_rows,
                          const double* global_matrix,
                          double* local_matrix) const;

  int GlobalSize() const;

  int LocalSize() const;
};

CALIB_DLL_LOCAL class ProjectCostFunction : public ceres::CostFunction {
public:
  ProjectCostFunction() = delete;

  ProjectCostFunction(const cv::Point2d& world_point,
                      const cv::Point2d& image_point,
                      bool with_k3 = false);

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const;

private:
  double world_point_[3];
  double image_point_[2];
  bool with_k3_;
};

CALIB_DLL_LOCAL class StereoCostFunction : public ceres::CostFunction {
public:
  StereoCostFunction() = delete;

  StereoCostFunction(const cv::Point2d& world_point,
                     const cv::Point2d& image_point_1,
                     const cv::Point2d& image_point_2,
                     bool with_k3 = false);

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const;

private:
  double world_point_[3];
  double image_point_1_[2];
  double image_point_2_[2];
  bool with_k3_;
};

} // namespace calib

#endif //CALIB_CERES_TYPE_H
