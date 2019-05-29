/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include "libcalib/ceres_type.h"

namespace calib {

bool SE3Parameterization::Plus(const double* x,
                               const double* delta,
                               double* x_plus_delta) const {
  double q[4];
  double q_delta[4];
  double q_plus_delta[4];
  ceres::AngleAxisToQuaternion(x + 3, q);
  ceres::AngleAxisToQuaternion(delta + 3, q_delta);
  ceres::QuaternionProduct(q_delta, q, q_plus_delta);
  ceres::QuaternionToAngleAxis(q_plus_delta, x_plus_delta + 3);

  double t[3];
  ceres::AngleAxisRotatePoint(delta + 3, x, t);

  x_plus_delta[0] = t[0] + delta[0];
  x_plus_delta[1] = t[1] + delta[1];
  x_plus_delta[2] = t[2] + delta[2];

  return true;
}

bool SE3Parameterization::ComputeJacobian(const double* x, double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 6, 6>>(jacobian, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
  return true;
}

bool SE3Parameterization::MultiplyByJacobian(const double* x,
                                             const int num_rows,
                                             const double* global_matrix,
                                             double* local_matrix) const {
  std::copy(global_matrix, global_matrix + num_rows * GlobalSize(), local_matrix);
  return true;
}

int SE3Parameterization::GlobalSize() const {
  return 6;
}

int SE3Parameterization::LocalSize() const {
  return 6;
}

ProjectCostFunction::ProjectCostFunction(const cv::Point2d& world_point,
                                         const cv::Point2d& image_point,
                                         bool with_k3)
    : world_point_{world_point.x, world_point.y, 0.}
    , image_point_{image_point.x, image_point.y}
    , with_k3_(with_k3) {
  set_num_residuals(2);
  auto params = mutable_parameter_block_sizes();
  params->emplace_back(9); // fx, fy, cx, cy, k1, k2, p1, p2, k3
  params->emplace_back(6); // se3(t, phi)
}

bool ProjectCostFunction::Evaluate(double const* const* parameters,
                                   double* residuals,
                                   double** jacobians) const {
  const double& fx  = parameters[0][0];
  const double& fy  = parameters[0][1];
  const double& cx  = parameters[0][2];
  const double& cy  = parameters[0][3];
  const double& k1  = parameters[0][4];
  const double& k2  = parameters[0][5];
  const double& p1  = parameters[0][6];
  const double& p2  = parameters[0][7];
  const double& k3  = parameters[0][8];
  const double* phi = parameters[1] + 3;
  const double* t   = parameters[1];

  // project
  double pc[3];
  ceres::AngleAxisRotatePoint(phi, world_point_, pc);
  double x = pc[0] + t[0];
  double y = pc[1] + t[1];
  double z = pc[2] + t[2];

  if(abs(z) < 1e-3) return false;

  double xp = x / z;
  double yp = y / z;

  double xp2 = xp * xp;
  double yp2 = yp * yp;
  double xyp = xp * yp;
  double r2  = xp2 + yp2;

  double txp = (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) * xp + 2. * p1 * xyp + p2 * (r2 + 2. * xp2);
  double typ = (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) * yp + p1 * (r2 + 2. * yp2) + 2. * p2 * xyp;
  double i   = fx * txp + cx;
  double j   = fy * typ + cy;

  residuals[0] = i - image_point_[0];
  residuals[1] = j - image_point_[1];

  if(jacobians == NULL) return true;

  // jacobians
  double di_dtxp = fx; // di / dtxp
  double dj_dtyp = fy; // dj / dtyp

  if(jacobians[0] != NULL) {
    jacobians[0][0]  = txp;                         // di / dfx
    jacobians[0][1]  = 0.;                          // di / dfy
    jacobians[0][2]  = 1.;                          // di / dcx
    jacobians[0][3]  = 0.;                          // di / dcy
    jacobians[0][4]  = di_dtxp * xp * r2;           // di / dk1
    jacobians[0][5]  = di_dtxp * xp * r2 * r2;      // di / dk2
    jacobians[0][6]  = di_dtxp * 2. * xyp;          // di / dp1
    jacobians[0][7]  = di_dtxp * (r2 + 2. * xp2);   // di / dp2
    jacobians[0][8]  = di_dtxp * xp * r2 * r2 * r2; // di / dk3
    jacobians[0][9]  = 0.;                          // dj / dfx
    jacobians[0][10] = typ;                         // dj / dfy
    jacobians[0][11] = 0.;                          // dj / dcx
    jacobians[0][12] = 1.;                          // dj / dcy
    jacobians[0][13] = dj_dtyp * yp * r2;           // dj / dk1
    jacobians[0][14] = dj_dtyp * yp * r2 * r2;      // dj / dk2
    jacobians[0][15] = dj_dtyp * (r2 + 2. * yp2);   // dj / dp1
    jacobians[0][16] = dj_dtyp * 2. * xyp;          // dj / dp2
    jacobians[0][17] = dj_dtyp * yp * r2 * r2 * r2; // dj / dk3

    if(!with_k3_) {
      jacobians[0][8]  = 0.; // di / dk3
      jacobians[0][17] = 0.; // dj / dk3
    }
  }

  double dtxp_dxp = 1 + k1 * (r2 + 2. * xp2) + k2 * r2 * (r2 + 4. * xp2) + 2. * p1 * yp + 6. * p2 * xp + k3 * r2 * r2 * (r2 + 6. * xp2);
  double dtyp_dyp = 1 + k1 * (r2 + 2. * yp2) + k2 * r2 * (r2 + 4. * yp2) + 6. * p1 * yp + 2. * p2 * xp + k3 * r2 * r2 * (r2 + 6. * yp2);
  double di_dxp   = di_dtxp * dtxp_dxp;
  double dj_dyp   = dj_dtyp * dtyp_dyp;

  double z2    = z * z;
  double di_dx = di_dxp / z;
  double di_dy = 0.;
  double di_dz = -di_dxp * x / z2;
  double dj_dx = 0.;
  double dj_dy = dj_dyp / z;
  double dj_dz = -dj_dyp * y / z2;

  if(jacobians[1] != NULL) {
    jacobians[1][0]  = di_dx;               // di / drho0
    jacobians[1][1]  = di_dy;               // di / drho1
    jacobians[1][2]  = di_dz;               // di / drho2
    jacobians[1][3]  = y * di_dz;           // di / dphi0
    jacobians[1][4]  = di_dxp - x * di_dz;  // di / dphi1
    jacobians[1][5]  = -y * di_dx;          // di / dphi2
    jacobians[1][6]  = dj_dx;               // dj / drho0
    jacobians[1][7]  = dj_dy;               // dj / drho1
    jacobians[1][8]  = dj_dz;               // dj / drho2
    jacobians[1][9]  = -dj_dyp + y * dj_dz; // dj / dphi0
    jacobians[1][10] = -x * dj_dz;          // dj / dphi1
    jacobians[1][11] = x * dj_dy;           // dj / dphi2
  }

  return true;
}

StereoCostFunction::StereoCostFunction(const cv::Point2d& world_point,
                                       const cv::Point2d& image_point_1,
                                       const cv::Point2d& image_point_2,
                                       bool with_k3)
    : world_point_{world_point.x, world_point.y, 0.}
    , image_point_1_{image_point_1.x, image_point_1.y}
    , image_point_2_{image_point_2.x, image_point_2.y}
    , with_k3_(with_k3) {
  set_num_residuals(4);
  auto params = mutable_parameter_block_sizes();
  params->emplace_back(9); // camera left fx, fy, cx, cy, k1, k2, p1, p2, k3
  params->emplace_back(9); // camera right fx, fy, cx, cy, k1, k2, p1, p2, k3
  params->emplace_back(6); // camera left se3(t, phi)
  params->emplace_back(6); // left to right se3(t, phi)
}

bool StereoCostFunction::Evaluate(double const* const* parameters,
                                  double* residuals,
                                  double** jacobians) const {
  const double& fx_1  = parameters[0][0];
  const double& fy_1  = parameters[0][1];
  const double& cx_1  = parameters[0][2];
  const double& cy_1  = parameters[0][3];
  const double& k1_1  = parameters[0][4];
  const double& k2_1  = parameters[0][5];
  const double& p1_1  = parameters[0][6];
  const double& p2_1  = parameters[0][7];
  const double& k3_1  = parameters[0][8];
  const double& fx_2  = parameters[1][0];
  const double& fy_2  = parameters[1][1];
  const double& cx_2  = parameters[1][2];
  const double& cy_2  = parameters[1][3];
  const double& k1_2  = parameters[1][4];
  const double& k2_2  = parameters[1][5];
  const double& p1_2  = parameters[1][6];
  const double& p2_2  = parameters[1][7];
  const double& k3_2  = parameters[1][8];
  const double* phi_1 = parameters[2] + 3;
  const double* t_1   = parameters[2];
  const double* phi   = parameters[3] + 3;
  const double* t     = parameters[3];

  // project
  double pc_1[3];
  ceres::AngleAxisRotatePoint(phi_1, world_point_, pc_1);
  double x_1 = pc_1[0] + t_1[0];
  double y_1 = pc_1[1] + t_1[1];
  double z_1 = pc_1[2] + t_1[2];

  double pc_1_tmp[3] = {x_1, y_1, z_1};
  double pc_2[3];
  ceres::AngleAxisRotatePoint(phi, pc_1_tmp, pc_2);
  double x_2 = pc_2[0] + t[0];
  double y_2 = pc_2[1] + t[1];
  double z_2 = pc_2[2] + t[2];

  if(abs(z_1) < 1e-3 || abs(z_2) < 1e-3) return false;

  double xp_1 = x_1 / z_1;
  double yp_1 = y_1 / z_1;

  double xp2_1 = xp_1 * xp_1;
  double yp2_1 = yp_1 * yp_1;
  double xyp_1 = xp_1 * yp_1;
  double r2_1  = xp2_1 + yp2_1;

  double txp_1 = (1 + r2_1 * (k1_1 + r2_1 * (k2_1 + r2_1 * k3_1))) * xp_1 + 2. * p1_1 * xyp_1 + p2_1 * (r2_1 + 2. * xp2_1);
  double typ_1 = (1 + r2_1 * (k1_1 + r2_1 * (k2_1 + r2_1 * k3_1))) * yp_1 + p1_1 * (r2_1 + 2. * yp2_1) + 2. * p2_1 * xyp_1;
  double i_1   = fx_1 * txp_1 + cx_1;
  double j_1   = fy_1 * typ_1 + cy_1;

  double xp_2 = x_2 / z_2;
  double yp_2 = y_2 / z_2;

  double xp2_2 = xp_2 * xp_2;
  double yp2_2 = yp_2 * yp_2;
  double xyp_2 = xp_2 * yp_2;
  double r2_2  = xp2_2 + yp2_2;

  double txp_2 = (1 + r2_2 * (k1_2 + r2_2 * (k2_2 + r2_2 * k3_2))) * xp_2 + 2. * p1_2 * xyp_2 + p2_2 * (r2_2 + 2. * xp2_2);
  double typ_2 = (1 + r2_2 * (k1_2 + r2_2 * (k2_2 + r2_2 * k3_2))) * yp_2 + p1_2 * (r2_2 + 2. * yp2_2) + 2. * p2_2 * xyp_2;
  double i_2   = fx_2 * txp_2 + cx_2;
  double j_2   = fy_2 * typ_2 + cy_2;

  residuals[0] = i_1 - image_point_1_[0];
  residuals[1] = j_1 - image_point_1_[1];
  residuals[2] = i_2 - image_point_2_[0];
  residuals[3] = j_2 - image_point_2_[1];

  if(jacobians == NULL) return true;

  // jacobians
  double di_dtxp_1 = fx_1; // di_1 / dtxp_1
  double dj_dtyp_1 = fy_1; // dj_1 / dtyp_1
  double di_dtxp_2 = fx_2; // di_2 / dtxp_2
  double dj_dtyp_2 = fy_2; // dj_2 / dtyp_2

  if(jacobians[0] != NULL) {
    jacobians[0][0]  = txp_1;                                 // di_1 / dfx_1
    jacobians[0][1]  = 0.;                                    // di_1 / dfy_1
    jacobians[0][2]  = 1.;                                    // di_1 / dcx_1
    jacobians[0][3]  = 0.;                                    // di_1 / dcy_1
    jacobians[0][4]  = di_dtxp_1 * xp_1 * r2_1;               // di_1 / dk1_1
    jacobians[0][5]  = di_dtxp_1 * xp_1 * r2_1 * r2_1;        // di_1 / dk2_1
    jacobians[0][6]  = di_dtxp_1 * 2. * xyp_1;                // di_1 / dp1_1
    jacobians[0][7]  = di_dtxp_1 * (r2_1 + 2. * xp2_1);       // di_1 / dp2_1
    jacobians[0][8]  = di_dtxp_1 * xp_1 * r2_1 * r2_1 * r2_1; // di_1 / dk3_1
    jacobians[0][9]  = 0.;                                    // dj_1 / dfx_1
    jacobians[0][10] = typ_1;                                 // dj_1 / dfy_1
    jacobians[0][11] = 0.;                                    // dj_1 / dcx_1
    jacobians[0][12] = 1.;                                    // dj_1 / dcy_1
    jacobians[0][13] = dj_dtyp_1 * yp_1 * r2_1;               // dj_1 / dk1_1
    jacobians[0][14] = dj_dtyp_1 * yp_1 * r2_1 * r2_1;        // dj_1 / dk2_1
    jacobians[0][15] = dj_dtyp_1 * (r2_1 + 2. * yp2_1);       // dj_1 / dp1_1
    jacobians[0][16] = dj_dtyp_1 * 2. * xyp_1;                // dj_1 / dp2_1
    jacobians[0][17] = dj_dtyp_1 * yp_1 * r2_1 * r2_1 * r2_1; // dj_1 / dk3_1
    jacobians[0][18] = 0.;                                    // di_2 / dfx_1
    jacobians[0][19] = 0.;                                    // di_2 / dfy_1
    jacobians[0][20] = 0.;                                    // di_2 / dcx_1
    jacobians[0][21] = 0.;                                    // di_2 / dcy_1
    jacobians[0][22] = 0.;                                    // di_2 / dk1_1
    jacobians[0][23] = 0.;                                    // di_2 / dk2_1
    jacobians[0][24] = 0.;                                    // di_2 / dp1_1
    jacobians[0][25] = 0.;                                    // di_2 / dp2_1
    jacobians[0][26] = 0.;                                    // di_2 / dk3_1
    jacobians[0][27] = 0.;                                    // dj_2 / dfx_1
    jacobians[0][28] = 0.;                                    // dj_2 / dfy_1
    jacobians[0][29] = 0.;                                    // dj_2 / dcx_1
    jacobians[0][30] = 0.;                                    // dj_2 / dcy_1
    jacobians[0][31] = 0.;                                    // dj_2 / dk1_1
    jacobians[0][32] = 0.;                                    // dj_2 / dk2_1
    jacobians[0][33] = 0.;                                    // dj_2 / dp1_1
    jacobians[0][34] = 0.;                                    // dj_2 / dp2_1
    jacobians[0][35] = 0.;                                    // dj_2 / dk3_1

    if(!with_k3_) {
      jacobians[0][8]  = 0.; // di_1 / dk3_1
      jacobians[0][17] = 0.; // dj_1 / dk3_1
    }
  }

  if(jacobians[1] != NULL) {
    jacobians[1][0]  = 0.;                                    // di_1 / dfx_2
    jacobians[1][1]  = 0.;                                    // di_1 / dfy_2
    jacobians[1][2]  = 0.;                                    // di_1 / dcx_2
    jacobians[1][3]  = 0.;                                    // di_1 / dcy_2
    jacobians[1][4]  = 0.;                                    // di_1 / dk1_2
    jacobians[1][5]  = 0.;                                    // di_1 / dk2_2
    jacobians[1][6]  = 0.;                                    // di_1 / dp1_2
    jacobians[1][7]  = 0.;                                    // di_1 / dp2_2
    jacobians[1][8]  = 0.;                                    // di_1 / dk3_2
    jacobians[1][9]  = 0.;                                    // dj_1 / dfx_2
    jacobians[1][10] = 0.;                                    // dj_1 / dfy_2
    jacobians[1][11] = 0.;                                    // dj_1 / dcx_2
    jacobians[1][12] = 0.;                                    // dj_1 / dcy_2
    jacobians[1][13] = 0.;                                    // dj_1 / dk1_2
    jacobians[1][14] = 0.;                                    // dj_1 / dk2_2
    jacobians[1][15] = 0.;                                    // dj_1 / dp1_2
    jacobians[1][16] = 0.;                                    // dj_1 / dp2_2
    jacobians[1][17] = 0.;                                    // dj_1 / dk3_2
    jacobians[1][18] = txp_2;                                 // di_2 / dfx_2
    jacobians[1][19] = 0.;                                    // di_2 / dfy_2
    jacobians[1][20] = 1.;                                    // di_2 / dcx_2
    jacobians[1][21] = 0.;                                    // di_2 / dcy_2
    jacobians[1][22] = di_dtxp_2 * xp_2 * r2_2;               // di_2 / dk1_2
    jacobians[1][23] = di_dtxp_2 * xp_2 * r2_2 * r2_2;        // di_2 / dk2_2
    jacobians[1][24] = di_dtxp_2 * 2. * xyp_2;                // di_2 / dp1_2
    jacobians[1][25] = di_dtxp_2 * (r2_2 + 2. * xp2_2);       // di_2 / dp2_2
    jacobians[1][26] = di_dtxp_2 * xp_2 * r2_2 * r2_2 * r2_2; // di_2 / dk3_2
    jacobians[1][27] = 0.;                                    // dj_2 / dfx_2
    jacobians[1][28] = typ_2;                                 // dj_2 / dfy_2
    jacobians[1][29] = 0.;                                    // dj_2 / dcx_2
    jacobians[1][30] = 1.;                                    // dj_2 / dcy_2
    jacobians[1][31] = dj_dtyp_2 * yp_2 * r2_2;               // dj_2 / dk1_2
    jacobians[1][32] = dj_dtyp_2 * yp_2 * r2_2 * r2_2;        // dj_2 / dk2_2
    jacobians[1][33] = dj_dtyp_2 * (r2_2 + 2. * yp2_2);       // dj_2 / dp1_2
    jacobians[1][34] = dj_dtyp_2 * 2. * xyp_2;                // dj_2 / dp2_2
    jacobians[1][35] = dj_dtyp_2 * yp_2 * r2_2 * r2_2 * r2_2; // dj_2 / dk3_2

    if(!with_k3_) {
      jacobians[1][26] = 0.; // di_2 / dk3_2
      jacobians[1][35] = 0.; // dj_2 / dk3_2
    }
  }

  double dtxp_dxp_1 = 1 + k1_1 * (r2_1 + 2. * xp2_1) + k2_1 * r2_1 * (r2_1 + 4. * xp2_1) + 2. * p1_1 * yp_1 + 6. * p2_1 * xp_1 + k3_1 * r2_1 * r2_1 * (r2_1 + 6. * xp2_1);
  double dtyp_dyp_1 = 1 + k1_1 * (r2_1 + 2. * yp2_1) + k2_1 * r2_1 * (r2_1 + 4. * yp2_1) + 6. * p1_1 * yp_1 + 2. * p2_1 * xp_1 + k3_1 * r2_1 * r2_1 * (r2_1 + 6. * yp2_1);
  double di_dxp_1   = di_dtxp_1 * dtxp_dxp_1;
  double dj_dyp_1   = dj_dtyp_1 * dtyp_dyp_1;

  double z2_1    = z_1 * z_1;
  double di_dx_1 = di_dxp_1 / z_1;
  double di_dy_1 = 0.;
  double di_dz_1 = -di_dxp_1 * x_1 / z2_1;
  double dj_dx_1 = 0.;
  double dj_dy_1 = dj_dyp_1 / z_1;
  double dj_dz_1 = -dj_dyp_1 * y_1 / z2_1;

  double dtxp_dxp_2 = 1 + k1_2 * (r2_2 + 2. * xp2_2) + k2_2 * r2_2 * (r2_2 + 4. * xp2_2) + 2. * p1_2 * yp_2 + 6. * p2_2 * xp_2 + k3_2 * r2_2 * r2_2 * (r2_2 + 6. * xp2_2);
  double dtyp_dyp_2 = 1 + k1_2 * (r2_2 + 2. * yp2_2) + k2_2 * r2_2 * (r2_2 + 4. * yp2_2) + 6. * p1_2 * yp_2 + 2. * p2_2 * xp_2 + k3_2 * r2_2 * r2_2 * (r2_2 + 6. * yp2_2);
  double di_dxp_2   = di_dtxp_2 * dtxp_dxp_2;
  double dj_dyp_2   = dj_dtyp_2 * dtyp_dyp_2;

  double z2_2    = z_2 * z_2;
  double di_dx_2 = di_dxp_2 / z_2;
  double di_dy_2 = 0.;
  double di_dz_2 = -di_dxp_2 * x_2 / z2_2;
  double dj_dx_2 = 0.;
  double dj_dy_2 = dj_dyp_2 / z_2;
  double dj_dz_2 = -dj_dyp_2 * y_2 / z2_2;

  double r[9]; // col major
  ceres::AngleAxisToRotationMatrix(phi, r);
  double di_2_dx_1 = di_dx_2 * r[0] + di_dy_2 * r[1] + di_dz_2 * r[2];
  double di_2_dy_1 = di_dx_2 * r[3] + di_dy_2 * r[4] + di_dz_2 * r[5];
  double di_2_dz_1 = di_dx_2 * r[6] + di_dy_2 * r[7] + di_dz_2 * r[8];
  double dj_2_dx_1 = dj_dx_2 * r[0] + dj_dy_2 * r[1] + dj_dz_2 * r[2];
  double dj_2_dy_1 = dj_dx_2 * r[3] + dj_dy_2 * r[4] + dj_dz_2 * r[4];
  double dj_2_dz_1 = dj_dx_2 * r[6] + dj_dy_2 * r[7] + dj_dz_2 * r[8];

  if(jacobians[2] != NULL) {
    jacobians[2][0]  = di_dx_1;                            // di_1 / drho0_1
    jacobians[2][1]  = di_dy_1;                            // di_1 / drho1_1
    jacobians[2][2]  = di_dz_1;                            // di_1 / drho2_1
    jacobians[2][3]  = y_1 * di_dz_1;                      // di_1 / dphi0_1
    jacobians[2][4]  = di_dxp_1 - x_1 * di_dz_1;           // di_1 / dphi1_1
    jacobians[2][5]  = -y_1 * di_dx_1;                     // di_1 / dphi2_1
    jacobians[2][6]  = dj_dx_1;                            // dj_1 / drho0_1
    jacobians[2][7]  = dj_dy_1;                            // dj_1 / drho1_1
    jacobians[2][8]  = dj_dz_1;                            // dj_1 / drho2_1
    jacobians[2][9]  = -dj_dyp_1 + y_1 * dj_dz_1;          // dj_1 / dphi0_1
    jacobians[2][10] = -x_1 * dj_dz_1;                     // dj_1 / dphi1_1
    jacobians[2][11] = x_1 * dj_dy_1;                      // dj_1 / dphi2_1
    jacobians[2][12] = di_2_dx_1;                          // di_2 / drho0_1
    jacobians[2][13] = di_2_dy_1;                          // di_2 / drho1_1
    jacobians[2][14] = di_2_dz_1;                          // di_2 / drho2_1
    jacobians[2][15] = -z_1 * di_2_dy_1 + y_1 * di_2_dz_1; // di_2 / dphi0_1
    jacobians[2][16] = z_1 * di_2_dx_1 - x_1 * di_2_dz_1;  // di_2 / dphi1_1
    jacobians[2][17] = -y_1 * di_2_dx_1 + x_1 * di_2_dy_1; // di_2 / dphi2_1
    jacobians[2][18] = dj_2_dx_1;                          // dj_2 / drho0_1
    jacobians[2][19] = dj_2_dy_1;                          // dj_2 / drho1_1
    jacobians[2][20] = dj_2_dz_1;                          // dj_2 / drho2_1
    jacobians[2][21] = -z_1 * dj_2_dy_1 + y_1 * dj_2_dz_1; // dj_2 / dphi0_1
    jacobians[2][22] = z_1 * dj_2_dx_1 - x_1 * dj_2_dz_1;  // dj_2 / dphi1_1
    jacobians[2][23] = -y_1 * dj_2_dx_1 + x_1 * dj_2_dy_1; // dj_2 / dphi2_1
  }

  if(jacobians[3] != NULL) {
    jacobians[3][0]  = 0.;                        // di_1 / drho0
    jacobians[3][1]  = 0.;                        // di_1 / drho1
    jacobians[3][2]  = 0.;                        // di_1 / drho2
    jacobians[3][3]  = 0.;                        // di_1 / dphi0
    jacobians[3][4]  = 0.;                        // di_1 / dphi1
    jacobians[3][5]  = 0.;                        // di_1 / dphi2
    jacobians[3][6]  = 0.;                        // dj_1 / drho0
    jacobians[3][7]  = 0.;                        // dj_1 / drho1
    jacobians[3][8]  = 0.;                        // dj_1 / drho2
    jacobians[3][9]  = 0.;                        // dj_1 / dphi0
    jacobians[3][10] = 0.;                        // dj_1 / dphi1
    jacobians[3][11] = 0.;                        // dj_1 / dphi2
    jacobians[3][12] = di_dx_2;                   // di_2 / drho0
    jacobians[3][13] = di_dy_2;                   // di_2 / drho1
    jacobians[3][14] = di_dz_2;                   // di_2 / drho2
    jacobians[3][15] = y_2 * di_dz_2;             // di_2 / dphi0
    jacobians[3][16] = di_dxp_2 - x_2 * di_dz_2;  // di_2 / dphi1
    jacobians[3][17] = -y_2 * di_dx_2;            // di_2 / dphi2
    jacobians[3][18] = dj_dx_2;                   // dj_2 / drho0
    jacobians[3][19] = dj_dy_2;                   // dj_2 / drho1
    jacobians[3][20] = dj_dz_2;                   // dj_2 / drho2
    jacobians[3][21] = -dj_dyp_2 + y_2 * dj_dz_2; // dj_2 / dphi0
    jacobians[3][22] = -x_2 * dj_dz_2;            // dj_2 / dphi1
    jacobians[3][23] = x_2 * dj_dy_2;             // dj_2 / dphi2
  }

  return true;
}

} // namespace calib