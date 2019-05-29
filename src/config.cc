/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#include <fstream>
#include <libcalib/config.h>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

#include "libcalib/config.h"
#include "libcbdetect/config.h"

namespace calib {

int get_config(const std::string& filename, Params& params) {
  std::ifstream ifs(filename);
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document dom;
  dom.ParseStream(isw);
  ifs.close();
  if(dom.HasParseError()) {
    return -1;
  }

  params.pattern_type = dom["pattern_type"].GetInt();
  params.pattern_size = dom["pattern_size"].GetDouble();
  params.num_boards   = dom["num_boards"].GetInt();

  if(dom.HasMember("camera1")) {
    auto dom1                           = dom["camera1"].GetObject();
    cbdetect::Params& detect_params     = params.camera1.detect_params;
    detect_params.detect_method         = static_cast<cbdetect::DetectMethod>(dom1["detect_method"].GetInt());
    detect_params.norm                  = dom1["norm"].GetBool();
    detect_params.norm_half_kernel_size = dom1["norm_half_kernel_size"].GetInt();
    detect_params.radius.clear();
    for(const auto& r : dom1["radius"].GetArray()) {
      detect_params.radius.emplace_back(r.GetInt());
    }
    detect_params.init_loc_thr                    = dom1["init_loc_thr"].GetDouble();
    detect_params.polynomial_fit_half_kernel_size = dom1["polynomial_fit_half_kernel_size"].GetInt();
    detect_params.score_thr                       = dom1["score_thr"].GetDouble();
    detect_params.show_processing                 = false;
    if(params.pattern_type == 0) {
      detect_params.corner_type = cbdetect::SaddlePoint;
    }
    params.camera1.show_cornres = dom1["show_cornres"].GetBool();
  }

  if(dom.HasMember("camera2")) {
    auto dom2                           = dom["camera2"].GetObject();
    cbdetect::Params& detect_params     = params.camera2.detect_params;
    detect_params.detect_method         = static_cast<cbdetect::DetectMethod>(dom2["detect_method"].GetInt());
    detect_params.norm                  = dom2["norm"].GetBool();
    detect_params.norm_half_kernel_size = dom2["norm_half_kernel_size"].GetInt();
    detect_params.radius.clear();
    for(const auto& r : dom2["radius"].GetArray()) {
      detect_params.radius.emplace_back(r.GetInt());
    }
    detect_params.init_loc_thr                    = dom2["init_loc_thr"].GetDouble();
    detect_params.polynomial_fit_half_kernel_size = dom2["polynomial_fit_half_kernel_size"].GetInt();
    detect_params.score_thr                       = dom2["score_thr"].GetDouble();
    detect_params.show_processing                 = false;
    if(params.pattern_type == 0) {
      detect_params.corner_type = cbdetect::SaddlePoint;
    }
    params.camera2.show_cornres = dom2["show_cornres"].GetBool();
  }

  return 0;
}

} // namespace calib
