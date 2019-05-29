/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#ifndef CALIB_INTERFACE_H
#define CALIB_INTERFACE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "libcalib/calib_def.h"

namespace calib {

CALIB_DLL_DECL int calibrate(const std::vector<cv::Mat>& images,
                             const std::string& config_file,
                             std::vector<double>& K,
                             std::vector<double>& D,
                             std::vector<std::vector<double>>& Rwc,
                             std::vector<std::vector<double>>& Twc,
                             std::vector<int>& extract_res,
                             double& error);

CALIB_DLL_DECL int calibrate(const std::vector<cv::Mat>& images,
                             const std::string& config_file,
                             std::vector<double>& K,
                             std::vector<double>& D,
                             std::vector<std::vector<double>>& Rwc,
                             std::vector<std::vector<double>>& Twc,
                             std::vector<int>& extract_res);

CALIB_DLL_DECL int calibrate_stereo(const std::vector<cv::Mat>& images_1,
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
                                    std::vector<double>& error);

CALIB_DLL_DECL int calibrate_stereo(const std::vector<cv::Mat>& images_1,
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
                                    std::vector<int>& extract_res);

} // namespace calib

#endif //CALIB_INTERFACE_H
