#include <stdio.h>

#include <chrono>
#include <vector>
using namespace std::chrono;

#include "libcalib/fileio.h"
#include "libcalib/interface.h"

int main() {
  std::vector<double> K1, K2, D1, D2;
  std::vector<std::vector<double>> Rwc1, Rwc2, Twc1, Twc2;
  std::vector<double> R, T;
  std::vector<int> extract_res;

  std::vector<cv::Mat> images_1, images_2;
  std::vector<std::string> files_1, files_2;
  get_all_file("../example/full_match/left", ".*png", files_1);
  get_all_file("../example/full_match/right", ".*png", files_2);
  for(int i = 0; i < files_1.size(); ++i) {
    cv::Mat image = cv::imread(files_1[i].c_str(), cv::IMREAD_GRAYSCALE);
    images_1.emplace_back(image);
  }
  for(int i = 0; i < files_2.size(); ++i) {
    cv::Mat image = cv::imread(files_2[i].c_str(), cv::IMREAD_GRAYSCALE);
    images_2.emplace_back(image);
  }

  auto t1 = high_resolution_clock::now();
  std::vector<double> err;
  calib::calibrate_stereo(images_1, images_2, "../example/full_match/config_template.json", K1, K2, D1, D2, Rwc1, Rwc2, Twc1, Twc2, R, T, extract_res, err);
  auto t2 = high_resolution_clock::now();
  printf("Took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0);

  printf("err = %lf, err = %lf, err = %lf\n", err[0], err[1], err[2]);
  printf("K1: %f %f %f %f %f\n", K1[0], K1[1], K1[2], K1[3], K1[4]);
  printf("D1: %f %f %f %f %f\n", D1[0], D1[1], D1[2], D1[3], D1[4]);
  printf("K2: %f %f %f %f %f\n", K2[0], K2[1], K2[2], K2[3], K2[4]);
  printf("D2: %f %f %f %f %f\n", D2[0], D2[1], D2[2], D2[3], D2[4]);
  printf("R: %f %f %f\n", R[0], R[1], R[2]);
  printf("T: %f %f %f\n", T[0], T[1], T[2]);

  cv::Mat rotation;
  cv::Mat angle_axis = (cv::Mat_<double>(3, 1) << R[0], R[1], R[2]);
  cv::Rodrigues(angle_axis, rotation);
  std::cout << rotation << "\n";

  return 0;
}
