/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#pragma once
#ifndef CALIB_CALIBRATION_H
#define CALIB_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "libcalib/calib_def.h"

namespace calib {

CALIB_DLL_LOCAL double calibrate_camera(const std::vector<std::vector<cv::Point2d>>& world_points,
                                        const std::vector<std::vector<cv::Point2d>>& image_points,
                                        std::vector<double>& K,
                                        std::vector<double>& D,
                                        std::vector<std::vector<double>>& Rwc,
                                        std::vector<std::vector<double>>& Twc);

CALIB_DLL_LOCAL double calibrate_stereo_camera(const std::vector<std::vector<cv::Point2d>>& world_points,
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
                                               std::vector<double>& T);

} // namespace calib

#endif //CALIB_CALIBRATION_H
