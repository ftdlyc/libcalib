#include <stdio.h>

#include <chrono>
#include <vector>
using namespace std::chrono;

#include "libcalib/fileio.h"
#include "libcalib/interface.h"

int main() {
  std::vector<double> K;
  std::vector<double> D;
  std::vector<std::vector<double>> Rwc;
  std::vector<std::vector<double>> Twc;
  std::vector<int> extract_res;

  std::vector<cv::Mat> images;
  std::vector<std::string> files;
  get_all_file("../example/full_match/left", ".*png", files);
  for(int i = 0; i < files.size(); ++i) {
    cv::Mat image = cv::imread(files[i].c_str(), cv::IMREAD_GRAYSCALE);
    images.emplace_back(image);
  }

  auto t1 = high_resolution_clock::now();
  double err;
  calib::calibrate(images, "../example/full_match/config_template.json", K, D, Rwc, Twc, extract_res, err);
  auto t2 = high_resolution_clock::now();
  printf("Took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0);

  printf("err = %lf\n", err);
  printf("%f %f %f %f %f\n", K[0], K[1], K[2], K[3], K[4]);
  printf("%f %f %f %f %f\n", D[0], D[1], D[2], D[3], D[4]);

  return 0;
}
