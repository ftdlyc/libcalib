/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#pragma once
#ifndef CALIB_EXTRACT_H
#define CALIB_EXTRACT_H

#include <opencv2/opencv.hpp>

#include "libcalib/config.h"

namespace calib {

CALIB_DLL_LOCAL std::vector<int> extract_corners(const std::vector<cv::Mat>& images,
                                                 std::vector<std::vector<cv::Point2d>>& image_points,
                                                 std::vector<std::vector<cv::Point2d>>& world_points,
                                                 Params& params);

CALIB_DLL_LOCAL int extract_corners(const cv::Mat& images,
                                    std::vector<cv::Point2d>& image_points,
                                    std::vector<cv::Point2d>& world_points,
                                    Params& params);

CALIB_DLL_LOCAL std::vector<int> extract_corners_stereo(const std::vector<cv::Mat>& images_1,
                                                        const std::vector<cv::Mat>& images_2,
                                                        std::vector<std::vector<cv::Point2d>>& image_points_1,
                                                        std::vector<std::vector<cv::Point2d>>& image_points_2,
                                                        std::vector<std::vector<cv::Point2d>>& world_points,
                                                        Params& params);

CALIB_DLL_LOCAL int extract_corners_stereo(const cv::Mat& images_1,
                                           const cv::Mat& images_2,
                                           std::vector<cv::Point2d>& image_points_1,
                                           std::vector<cv::Point2d>& image_points_2,
                                           std::vector<cv::Point2d>& world_points,
                                           Params& params);

} // namespace calib

#endif //CALIB_EXTRACT_H
