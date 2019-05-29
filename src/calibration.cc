/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#include <algorithm>
#include <functional>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "libcalib/calibration.h"
#include "libcalib/ceres_type.h"

namespace calib {

void init_camera_params(const std::vector<std::vector<cv::Point2d>>& world_points,
                        const std::vector<std::vector<cv::Point2d>>& image_points,
                        std::vector<double>& K,
                        std::vector<double>& D,
                        std::vector<std::vector<double>>& Rwc,
                        std::vector<std::vector<double>>& Twc) {
  int num_images = world_points.size();
  int num_points = 0;
  for(int i = 0; i < num_images; ++i) {
    num_points += world_points[i].size();
  }

  std::vector<cv::Mat> homo_vec;
  Eigen::MatrixXd mat_v = Eigen::MatrixXd::Zero(2 * num_images, 6);
  for(int i = 0; i < num_images; ++i) {
    cv::Mat homo = cv::findHomography(world_points[i], image_points[i], cv::RANSAC);
    auto homo_at = [&homo](int x, int y) { return homo.at<double>(y - 1, x - 1); };
    mat_v.block(2 * i, 0, 1, 6) << homo_at(1, 1) * homo_at(2, 1),
        homo_at(1, 1) * homo_at(2, 2) + homo_at(1, 2) * homo_at(2, 1),
        homo_at(1, 2) * homo_at(2, 2),
        homo_at(1, 3) * homo_at(2, 1) + homo_at(1, 1) * homo_at(2, 3),
        homo_at(1, 3) * homo_at(2, 2) + homo_at(1, 2) * homo_at(2, 3),
        homo_at(1, 3) * homo_at(2, 3);
    mat_v.block(2 * i + 1, 0, 1, 6) << homo_at(1, 1) * homo_at(1, 1) - homo_at(2, 1) * homo_at(2, 1),
        homo_at(1, 1) * homo_at(1, 2) + homo_at(1, 2) * homo_at(1, 1) -
            homo_at(2, 1) * homo_at(2, 2) - homo_at(2, 2) * homo_at(2, 1),
        homo_at(1, 2) * homo_at(1, 2) - homo_at(2, 2) * homo_at(2, 2),
        homo_at(1, 3) * homo_at(1, 1) + homo_at(1, 1) * homo_at(1, 3) -
            homo_at(2, 3) * homo_at(2, 1) - homo_at(2, 1) * homo_at(2, 3),
        homo_at(1, 3) * homo_at(1, 2) + homo_at(1, 2) * homo_at(1, 3) -
            homo_at(2, 3) * homo_at(2, 2) - homo_at(2, 2) * homo_at(2, 3),
        homo_at(1, 3) * homo_at(1, 3) - homo_at(2, 3) * homo_at(2, 3);
    homo_vec.emplace_back(homo);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat_v.transpose() * mat_v, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 6, 6> mat_b = svd.matrixV().cast<double>();
  double b11                        = mat_b(0, 5);
  double b12                        = mat_b(1, 5);
  double b22                        = mat_b(2, 5);
  double b13                        = mat_b(3, 5);
  double b23                        = mat_b(4, 5);
  double b33                        = mat_b(5, 5);
  double cy                         = (b12 * b13 - b11 * b23) / (b11 * b22 - b12 * b12);
  double lambda                     = b33 - (b13 * b13 + cy * (b12 * b13 - b11 * b23)) / b11;
  double fx                         = sqrt(lambda / b11);
  double fy                         = sqrt(lambda * b11 / (b11 * b22 - b12 * b12));
  double radio                      = -b12 * fx * fx * fy / lambda;
  double cx                         = radio * cy / fx - b13 * fx * fx / lambda;
  K[0]                              = fx;
  K[1]                              = fy;
  K[2]                              = 0.;
  K[3]                              = cx;
  K[4]                              = cy;
  Eigen::Matrix3d K_eigen;
  K_eigen << K[0], K[2], K[3],
      0, K[1], K[4],
      0, 0, 1;

  for(int i = 0; i < num_images; ++i) {
    Eigen::Matrix3d rotation_eigen;
    Eigen::Matrix<double, 3, 1> translation_eigen;
    Eigen::Matrix3d homo_eigen;
    cv::cv2eigen(homo_vec[i], homo_eigen);
    double s              = (K_eigen.inverse() * homo_eigen.col(0)).norm();
    rotation_eigen.col(0) = K_eigen.inverse() * homo_eigen.col(0) / s;
    rotation_eigen.col(1) = K_eigen.inverse() * homo_eigen.col(1) / s;
    rotation_eigen.col(2) = rotation_eigen.col(0).cross(rotation_eigen.col(1));
    translation_eigen     = K_eigen.inverse() * homo_eigen.col(2) / s;

    cv::Mat rotation;
    cv::Mat angle_axis;
    cv::eigen2cv(rotation_eigen, rotation);
    cv::Rodrigues(rotation, angle_axis);

    Rwc[i][0] = angle_axis.at<double>(0);
    Rwc[i][1] = angle_axis.at<double>(1);
    Rwc[i][2] = angle_axis.at<double>(2);
    Twc[i][0] = translation_eigen(0, 0);
    Twc[i][1] = translation_eigen(1, 0);
    Twc[i][2] = translation_eigen(2, 0);
  }

  D[0] = 0.;
  D[1] = 0.;
  D[2] = 0.;
  D[3] = 0.;
  D[4] = 0.;
}

double optimize_params(const std::vector<std::vector<cv::Point2d>>& world_points,
                       const std::vector<std::vector<cv::Point2d>>& image_points,
                       std::vector<double>& K,
                       std::vector<double>& D,
                       std::vector<std::vector<double>>& Rwc,
                       std::vector<std::vector<double>>& Twc) {
  std::vector<double> intrinsics = {K[0], K[1], K[3], K[4], D[0], D[1], D[2], D[3], D[4]};
  std::vector<std::vector<double>> se3(world_points.size(), std::vector<double>(6));
  for(int i = 0; i < world_points.size(); ++i) {
    se3[i][0] = Twc[i][0];
    se3[i][1] = Twc[i][1];
    se3[i][2] = Twc[i][2];
    se3[i][3] = Rwc[i][0];
    se3[i][4] = Rwc[i][1];
    se3[i][5] = Rwc[i][2];
  }

  ceres::Problem problem;
  ceres::HuberLoss* loss_fuction = new ceres::HuberLoss(4.5);
  for(int i = 0; i < world_points.size(); ++i) {
    for(int j = 0; j < world_points[i].size(); ++j) {
      problem.AddResidualBlock(new ProjectCostFunction(world_points[i][j], image_points[i][j]), nullptr, intrinsics.data(), se3[i].data());
    }
    problem.SetParameterization(se3[i].data(), new SE3Parameterization());
  }

  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  K = {intrinsics[0], intrinsics[1], 0., intrinsics[2], intrinsics[3]};
  D = {intrinsics[4], intrinsics[5], intrinsics[6], intrinsics[7], intrinsics[8]};
  for(int i = 0; i < world_points.size(); ++i) {
    Twc[i][0] = se3[i][0];
    Twc[i][1] = se3[i][1];
    Twc[i][2] = se3[i][2];
    Rwc[i][0] = se3[i][3];
    Rwc[i][1] = se3[i][4];
    Rwc[i][2] = se3[i][5];
  }

  double err = sqrt(2 * summary.final_cost / summary.num_residual_blocks);
  return err;
}

double calibrate_camera(const std::vector<std::vector<cv::Point2d>>& world_points,
                        const std::vector<std::vector<cv::Point2d>>& image_points,
                        std::vector<double>& K,
                        std::vector<double>& D,
                        std::vector<std::vector<double>>& Rwc,
                        std::vector<std::vector<double>>& Twc) {
  init_camera_params(world_points, image_points, K, D, Rwc, Twc);
  double err = optimize_params(world_points, image_points, K, D, Rwc, Twc);
  return err;
}

void init_stereo_params(std::vector<std::vector<double>>& Rwc1,
                        std::vector<std::vector<double>>& Rwc2,
                        std::vector<std::vector<double>>& Twc1,
                        std::vector<std::vector<double>>& Twc2,
                        std::vector<double>& R,
                        std::vector<double>& T) {
  auto num_images       = static_cast<int>(Rwc1.size());
  Eigen::MatrixXd mat_a = Eigen::MatrixXd::Zero(9 * num_images, 9);
  Eigen::MatrixXd mat_b = Eigen::MatrixXd::Zero(9 * num_images, 1);
  for(int i = 0; i < num_images; ++i) {
    Eigen::Matrix3d r1_t, r2;
    ceres::AngleAxisToRotationMatrix(Rwc1[i].data(), r1_t.data());
    ceres::AngleAxisToRotationMatrix(Rwc2[i].data(), r2.data());
    r1_t.transposeInPlace();
    mat_a.block(9 * i, 0, 3, 3)     = r1_t;
    mat_a.block(9 * i + 3, 3, 3, 3) = r1_t;
    mat_a.block(9 * i + 6, 6, 3, 3) = r1_t;
    mat_b.block(9 * i, 0, 9, 1) << r2(0, 0), r2(0, 1), r2(0, 2),
        r2(1, 0), r2(1, 1), r2(1, 2),
        r2(2, 0), r2(2, 1), r2(2, 2);
  }
  Eigen::MatrixXd r = (mat_a.transpose() * mat_a).inverse() * mat_a.transpose() * mat_b;
  r.resize(3, 3);
  r.transposeInPlace();

  T[0] = T[1] = T[2] = 0.;
  for(int i = 0; i < num_images; ++i) {
    Eigen::Matrix<double, 3, 1> t, t1(Twc1[i].data()), t2(Twc2[i].data());
    t = t2 - r * t1;
    T[0] += t(0, 0);
    T[1] += t(1, 0);
    T[2] += t(2, 0);
  }
  T[0] /= num_images;
  T[1] /= num_images;
  T[2] /= num_images;
  ceres::RotationMatrixToAngleAxis(r.data(), R.data());
}

double optimize_stereo_params(const std::vector<std::vector<cv::Point2d>>& world_points,
                              const std::vector<std::vector<cv::Point2d>>& image_points_1,
                              const std::vector<std::vector<cv::Point2d>>& image_points_2,
                              std::vector<double>& K1,
                              std::vector<double>& K2,
                              std::vector<double>& D1,
                              std::vector<double>& D2,
                              std::vector<std::vector<double>>& Rwc1,
                              std::vector<std::vector<double>>& Rwc2,
                              std::vector<std::vector<double>>& Twc1,
                              std::vector<std::vector<double>>& Twc2,
                              std::vector<double>& R,
                              std::vector<double>& T) {
  std::vector<double> intrinsics_1 = {K1[0], K1[1], K1[3], K1[4], D1[0], D1[1], D1[2], D1[3], D1[4]};
  std::vector<double> intrinsics_2 = {K2[0], K2[1], K2[3], K2[4], D2[0], D2[1], D2[2], D2[3], D2[4]};
  std::vector<std::vector<double>> se3_1(world_points.size(), std::vector<double>(6));
  std::vector<double> se3 = {T[0], T[1], T[2], R[0], R[1], R[2]};
  for(int i = 0; i < world_points.size(); ++i) {
    se3_1[i][0] = Twc1[i][0];
    se3_1[i][1] = Twc1[i][1];
    se3_1[i][2] = Twc1[i][2];
    se3_1[i][3] = Rwc1[i][0];
    se3_1[i][4] = Rwc1[i][1];
    se3_1[i][5] = Rwc1[i][2];
  }

  ceres::Problem problem;
  ceres::HuberLoss* loss_fuction = new ceres::HuberLoss(4.5);
  for(int i = 0; i < world_points.size(); ++i) {
    for(int j = 0; j < world_points[i].size(); ++j) {
      problem.AddResidualBlock(new StereoCostFunction(world_points[i][j], image_points_1[i][j], image_points_2[i][j]), nullptr,
                               intrinsics_1.data(), intrinsics_2.data(), se3_1[i].data(), se3.data());
    }
    problem.SetParameterization(se3_1[i].data(), new SE3Parameterization());
  }
  problem.SetParameterization(se3.data(), new SE3Parameterization());

  ceres::Solver::Options options;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.trust_region_strategy_type   = ceres::DOGLEG;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  K1 = {intrinsics_1[0], intrinsics_1[1], 0., intrinsics_1[2], intrinsics_1[3]};
  D1 = {intrinsics_1[4], intrinsics_1[5], intrinsics_1[6], intrinsics_1[7], intrinsics_1[8]};
  K2 = {intrinsics_2[0], intrinsics_2[1], 0., intrinsics_2[2], intrinsics_2[3]};
  D2 = {intrinsics_2[4], intrinsics_2[5], intrinsics_2[6], intrinsics_2[7], intrinsics_2[8]};
  R  = {se3[3], se3[4], se3[5]};
  T  = {se3[0], se3[1], se3[2]};

  double q[4], q1[4], q2[4];
  ceres::AngleAxisToQuaternion(R.data(), q);
  for(int i = 0; i < world_points.size(); ++i) {
    Twc1[i][0] = se3_1[i][0];
    Twc1[i][1] = se3_1[i][1];
    Twc1[i][2] = se3_1[i][2];
    Rwc1[i][0] = se3_1[i][3];
    Rwc1[i][1] = se3_1[i][4];
    Rwc1[i][2] = se3_1[i][5];

    ceres::AngleAxisToQuaternion(Rwc1[i].data(), q1);
    ceres::QuaternionProduct(q, q1, q2);
    ceres::QuaternionToAngleAxis(q2, Rwc2[i].data());
    ceres::AngleAxisRotatePoint(R.data(), Twc1[i].data(), Twc2[i].data());
    Twc2[i][0] += T[0];
    Twc2[i][1] += T[1];
    Twc2[i][2] += T[2];
  }

  double err = sqrt(2 * summary.final_cost / summary.num_residual_blocks / 2);
  return err;
}

double calibrate_stereo_camera(const std::vector<std::vector<cv::Point2d>>& world_points,
                               const std::vector<std::vector<cv::Point2d>>& image_points_1,
                               const std::vector<std::vector<cv::Point2d>>& image_points_2,
                               std::vector<double>& K1,
                               std::vector<double>& K2,
                               std::vector<double>& D1,
                               std::vector<double>& D2,
                               std::vector<std::vector<double>>& Rwc1,
                               std::vector<std::vector<double>>& Rwc2,
                               std::vector<std::vector<double>>& Twc1,
                               std::vector<std::vector<double>>& Twc2,
                               std::vector<double>& R,
                               std::vector<double>& T) {
  init_stereo_params(Rwc1, Rwc2, Twc1, Twc2, R, T);
  double err = optimize_stereo_params(world_points, image_points_1, image_points_2, K1, K2, D1, D2, Rwc1, Rwc2, Twc1, Twc2, R, T);
  return err;
}

} // namespace calib
