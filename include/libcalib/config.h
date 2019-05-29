/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#pragma once
#ifndef CALIB_CONFIG_H
#define CALIB_CONFIG_H

#include <vector>

#include "libcalib/calib_def.h"
#include "libcbdetect/config.h"

namespace calib {

typedef struct DetectParams {
  cbdetect::Params detect_params;
  bool show_cornres = false;
} DetectParams;

typedef struct Params {
  double pattern_size;
  int num_boards;
  int pattern_type;
  DetectParams camera1;
  DetectParams camera2;

  Params()
      : pattern_size(24)
      , num_boards(1)
      , pattern_type(0){};
} Params;

CALIB_DLL_LOCAL int get_config(const std::string& filename, Params& params);

} // namespace calib

#endif //CALIB_CONFIG_H
