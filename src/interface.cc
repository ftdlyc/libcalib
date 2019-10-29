/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#include <stdio.h>

#include <vector>

#include <opencv2/opencv.hpp>

#include "libcalib/calibration.h"
#include "libcalib/config.h"
#include "libcalib/extract.h"
#include "libcalib/interface.h"

namespace calib {

int calibrate(const std::vector<cv::Mat>& images,
              const std::string& config_file,
              std::vector<double>& K,
              std::vector<double>& D,
              std::vector<std::vector<double>>& Rwc,
              std::vector<std::vector<double>>& Twc,
              std::vector<int>& extract_res,
              double& error) {
  // load config
  Params params;
  if(get_config(config_file, params) < 0) {
    printf("Failed to load config from %s\n", config_file.c_str());
    return -1;
  }

  // extract corners
  std::vector<std::vector<cv::Point2d>> image_points, world_points;
  extract_res = std::move(extract_corners(images, image_points, world_points, params));
  if(image_points.size() < 4) {
    return -1;
  }

  // calibrate camera
  K.resize(5);
  D.resize(5);
  Rwc   = std::move(std::vector<std::vector<double>>(image_points.size(), std::vector<double>(3, 0)));
  Twc   = std::move(std::vector<std::vector<double>>(image_points.size(), std::vector<double>(3, 0)));
  error = calibrate_camera(world_points, image_points, K, D, Rwc, Twc);

  return 0;
}

int calibrate(const std::vector<cv::Mat>& images,
              const std::string& config_file,
              std::vector<double>& K,
              std::vector<double>& D,
              std::vector<std::vector<double>>& Rwc,
              std::vector<std::vector<double>>& Twc,
              std::vector<int>& extract_res) {
  double error;
  return calibrate(images, config_file, K, D, Rwc, Twc, extract_res, error);
}

int calibrate_stereo(const std::vector<cv::Mat>& images_1,
                     const std::vector<cv::Mat>& images_2,
                     const std::string& config_file,
                     std::vector<double>& K1,
                     std::vector<double>& K2,
                     std::vector<double>& D1,
                     std::vector<double>& D2,
                     std::vector<std::vector<double>>& Rwc1,
                     std::vector<std::vector<double>>& Rwc2,
                     std::vector<std::vector<double>>& Twc1,
                     std::vector<std::vector<double>>& Twc2,
                     std::vector<double>& R,
                     std::vector<double>& T,
                     std::vector<int>& extract_res,
                     std::vector<double>& error) {
  // load config
  Params params;
  if(get_config(config_file, params) < 0) {
    printf("Failed to load config from %s\n", config_file.c_str());
    return -1;
  }

  // extract corners
  std::vector<std::vector<cv::Point2d>> image_points_1, image_points_2, world_points;
  extract_res = std::move(extract_corners_stereo(images_1, images_2, image_points_1, image_points_2, world_points, params));
  if(world_points.size() < 4) {
    return -1;
  }

  // calibrate camera
  K1.resize(5);
  K2.resize(5);
  D1.resize(5);
  D2.resize(5);
  Rwc1 = std::move(std::vector<std::vector<double>>(world_points.size(), std::vector<double>(3, 0)));
  Rwc2 = std::move(std::vector<std::vector<double>>(world_points.size(), std::vector<double>(3, 0)));
  Twc1 = std::move(std::vector<std::vector<double>>(world_points.size(), std::vector<double>(3, 0)));
  Twc2 = std::move(std::vector<std::vector<double>>(world_points.size(), std::vector<double>(3, 0)));
  R.resize(3);
  T.resize(3);

  error.resize(3);
  error[0] = calibrate_camera(world_points, image_points_1, K1, D1, Rwc1, Twc1);
  error[1] = calibrate_camera(world_points, image_points_2, K2, D2, Rwc2, Twc2);
  error[2] = calibrate_stereo_camera(world_points, image_points_1, image_points_2, K1, K2, D1, D2, Rwc1, Rwc2, Twc1, Twc2, R, T);

  return 0;
}

int calibrate_stereo(const std::vector<cv::Mat>& images_1,
                     const std::vector<cv::Mat>& images_2,
                     const std::string& config_file,
                     std::vector<double>& K1,
                     std::vector<double>& K2,
                     std::vector<double>& D1,
                     std::vector<double>& D2,
                     std::vector<std::vector<double>>& Rwc1,
                     std::vector<std::vector<double>>& Rwc2,
                     std::vector<std::vector<double>>& Twc1,
                     std::vector<std::vector<double>>& Twc2,
                     std::vector<double>& R,
                     std::vector<double>& T,
                     std::vector<int>& extract_res) {
  std::vector<double> error;
  return calibrate_stereo(images_1, images_2, config_file, K1, K2, D1, D2, Rwc1, Rwc2, Twc1, Twc2, R, T, extract_res, error);
}

} // namespace calib
