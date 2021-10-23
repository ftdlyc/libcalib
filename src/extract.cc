/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#include <algorithm>
#include <tuple>
#include <vector>

#include <opencv2/opencv.hpp>

#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_boards.h"

#include "libcalib/config.h"
#include "libcalib/extract.h"

namespace calib {

// pattern
// angle0     angle1     angle2     angle3
// -2            -2 -2   -2 -2      -2 -2 -2
// -2 -2      -2 -2 -2   -2 -2      -2 -2
// -2 -2                    -2
int find_pattern(const std::vector<std::vector<int>>& chessboard,
                 int& x, int& y, int& angle) {
  std::vector<std::tuple<int, int, int>> pos;
  for(int j = 2; j < chessboard.size() - 2; ++j) {
    for(int i = 2; i < chessboard[0].size() - 2; ++i) {
      if(chessboard[j][i] != -2 ||
         chessboard[j + 1][i] != -2 ||
         chessboard[j][i + 1] != -2 ||
         chessboard[j + 1][i + 1] != -2 ||
         chessboard[j - 1][i + 1] < 0 ||
         chessboard[j + 1][i + 2] < 0 ||
         chessboard[j + 2][i] < 0 ||
         chessboard[j][i - 1] < 0) continue;

      int n = (chessboard[j - 1][i] == -2) + (chessboard[j][i + 2] == -2) + (chessboard[j + 2][i + 1] == -2) + (chessboard[j + 1][i - 1] == -2);
      if(n != 1) continue;

      if(chessboard[j - 1][i] == -2) { // angle0
        if(chessboard[j - 2][i] < 0 || chessboard[j - 1][i - 1] < 0) continue;
        pos.emplace_back(std::make_tuple(i, j - 1, 0));
      } else if(chessboard[j + 1][i - 1] == -2) { // angle1
        if(chessboard[j + 2][i - 1] < 0 || chessboard[j + 1][i - 2] < 0) continue;
        pos.emplace_back(std::make_tuple(i - 1, j + 1, 1));
      } else if(chessboard[j + 2][i + 1] == -2) { // angle2
        if(chessboard[j + 2][i + 2] < 0 || chessboard[j + 3][i + 1] < 0) continue;
        pos.emplace_back(std::make_tuple(i + 1, j + 2, 2));
      } else if(chessboard[j][i + 2] == -2) { // angle3
        if(chessboard[j - 1][i + 2] < 0 || chessboard[j][i + 3] < 0) continue;
        pos.emplace_back(std::make_tuple(i + 2, j, 3));
      }
    }
  }

  if(pos.size() != 1) return -1;
  x = std::get<0>(pos[0]);
  y = std::get<1>(pos[0]);
  angle = std::get<2>(pos[0]);
  return 0;
}

int rotate_chessboard(const std::vector<std::vector<int>>& src,
                      std::vector<std::vector<int>>& dst,
                      int& x, int& y, int& angle) {
  switch(angle) {
  case 0: {
    dst = src;
    break;
  }
  case 1: {
    dst = std::move(std::vector<std::vector<int>>(src[0].size(), std::vector<int>(src.size())));
    for(int j = 0; j < src.size(); ++j) {
      for(int i = 0; i < src[0].size(); ++i) {
        dst[i][src.size() - 1 - j] = src[j][i];
      }
    }
    int tmp = x;
    x = src.size() - 1 - y;
    y = tmp;
    break;
  }
  case 2: {
    dst = std::move(std::vector<std::vector<int>>(src.size(), std::vector<int>(src[0].size())));
    for(int j = 0; j < src.size(); ++j) {
      for(int i = 0; i < src[0].size(); ++i) {
        dst[src.size() - 1 - j][src[0].size() - 1 - i] = src[j][i];
      }
    }
    x = src[0].size() - 1 - x;
    y = src.size() - 1 - y;
    break;
  }
  case 3: {
    dst = std::move(std::vector<std::vector<int>>(src[0].size(), std::vector<int>(src.size())));
    for(int j = 0; j < src.size(); ++j) {
      for(int i = 0; i < src[0].size(); ++i) {
        dst[src[0].size() - 1 - i][j] = src[j][i];
      }
    }
    int tmp = x;
    x = y;
    y = src[0].size() - 1 - tmp;
    break;
  }
  }

  angle = 0;
  return 0;
}

int match_two_chessboards(const std::vector<std::vector<int>>& chessboard_1,
                          const std::vector<std::vector<int>>& chessboard_2,
                          const std::vector<cv::Point2d>& points_1,
                          const std::vector<cv::Point2d>& points_2,
                          std::vector<cv::Point2d>& image_points_1,
                          std::vector<cv::Point2d>& image_points_2,
                          std::vector<cv::Point2d>& world_points,
                          double pattern_size) {
  int x1, y1, angle1, x2, y2, angle2;
  if(find_pattern(chessboard_1, x1, y1, angle1) < 0) return -1;
  if(find_pattern(chessboard_2, x2, y2, angle2) < 0) return -1;

  std::vector<std::vector<int>> norm_board_1, norm_board_2;
  rotate_chessboard(chessboard_1, norm_board_1, x1, y1, angle1);
  rotate_chessboard(chessboard_2, norm_board_2, x2, y2, angle2);

  for(int j = 0; j < norm_board_1.size(); ++j) {
    for(int i = 0; i < norm_board_1[0].size(); ++i) {
      int ii = i - x1 + x2;
      int jj = j - y1 + y2;
      if(ii < 0 || ii >= norm_board_1[0].size() ||
         jj < 0 || jj >= norm_board_1.size() ||
         norm_board_1[j][i] < 0 || norm_board_2[jj][ii] < 0) continue;

      image_points_1.emplace_back(points_1[norm_board_1[j][i]]);
      image_points_2.emplace_back(points_2[norm_board_2[jj][ii]]);
      world_points.emplace_back(cv::Point2d(pattern_size * (i - 1), pattern_size * (j - 1)));
    }
  }

  return 0;
}

std::vector<int> extract_corners(const std::vector<cv::Mat>& images,
                                 std::vector<std::vector<cv::Point2d>>& image_points,
                                 std::vector<std::vector<cv::Point2d>>& world_points,
                                 Params& params) {
  std::vector<int> res(images.size(), 0);
  if(params.camera1.detect_params.corner_type == cbdetect::SaddlePoint) {
    params.camera1.detect_params.overlay = true;
    for(int i = 0; i < images.size(); ++i) {
      cbdetect::Corner chessboad_corners;
      std::vector<cbdetect::Board> chessboards;

      cbdetect::find_corners(images[i], chessboad_corners, params.camera1.detect_params);
      cbdetect::boards_from_corners(images[i], chessboad_corners, chessboards, params.camera1.detect_params);
      if(chessboards.empty()) {
        continue;
      }
      if(params.camera1.show_cornres) {
        cbdetect::plot_boards(images[i], chessboad_corners, chessboards, params.camera1.detect_params);
      }

      std::sort(chessboards.begin(), chessboards.end(),
                [](const cbdetect::Board& b1, const cbdetect::Board& b2) {
                  return b1.num > b2.num;
                });

      res[i] = std::min(params.num_boards, (int)chessboards.size());
      for(int j = 0; j < res[i]; ++j) {
        std::vector<std::vector<int>>& chessboard = chessboards[j].idx;
        std::vector<cv::Point2d> pi_buf, pw_buf;

        for(int jj = 0; jj < chessboard.size(); ++jj) {
          for(int ii = 0; ii < chessboard[0].size(); ++ii) {
            if(chessboard[jj][ii] < 0) continue;
            pi_buf.emplace_back(chessboad_corners.p[chessboard[jj][ii]]);
            pw_buf.emplace_back(cv::Point2d(params.pattern_size * (ii - 1), params.pattern_size * (jj - 1)));
          }
        }
        image_points.emplace_back(pi_buf);
        world_points.emplace_back(pw_buf);
      }
    }
  }
  if(params.camera1.detect_params.corner_type == cbdetect::MonkeySaddlePoint) {
    for(int i = 0; i < images.size(); ++i) {
      cbdetect::Corner deltille_corners;
      std::vector<cbdetect::Board> deltilles;

      cbdetect::find_corners(images[i], deltille_corners, params.camera1.detect_params);
      cbdetect::boards_from_corners(images[i], deltille_corners, deltilles, params.camera1.detect_params);
      if(deltilles.empty()) {
        continue;
      }
      if(params.camera1.show_cornres) {
        cbdetect::plot_boards(images[i], deltille_corners, deltilles, params.camera1.detect_params);
      }

      std::sort(deltilles.begin(), deltilles.end(),
                [](const cbdetect::Board& d1, const cbdetect::Board& d2) {
                  return d1.num > d2.num;
                });

      res[i] = std::min(params.num_boards, (int)deltilles.size());
      double dx = params.pattern_size;
      double dy = params.pattern_size / 2.0 * std::sqrt(3);
      for(int j = 0; j < res[i]; ++j) {
        cbdetect::Board& deltille = deltilles[j];
        std::vector<cv::Point2d> pi_buf, pw_buf;

        for(int jj = 0; jj < deltille.idx.size(); ++jj) {
          double shift = -jj * dx / 2.0;
          for(int ii = 0; ii < deltille.idx[0].size(); ++ii) {
            if(deltille.idx[jj][ii] >= 0) {
              pi_buf.emplace_back(deltille_corners.p[deltille.idx[jj][ii]]);
              pw_buf.emplace_back(cv::Point2d(shift + dx * ii, dy * jj));
            }
          }
        }
        image_points.emplace_back(pi_buf);
        world_points.emplace_back(pw_buf);
      }
    }
  }

  return res;
}

int extract_corners(const cv::Mat& images,
                    std::vector<cv::Point2d>& image_points,
                    std::vector<cv::Point2d>& world_points,
                    Params& params) {
  params.camera1.detect_params.overlay = true;

  cbdetect::Corner corners;
  std::vector<cbdetect::Board> boards;
  cbdetect::find_corners(images, corners, params.camera1.detect_params);
  cbdetect::boards_from_corners(images, corners, boards, params.camera1.detect_params);

  if(params.camera1.show_cornres) {
    cbdetect::plot_boards(images, corners, boards, params.camera1.detect_params);
  }

  return 0;
}

std::vector<int> extract_corners_stereo(const std::vector<cv::Mat>& images_1,
                                        const std::vector<cv::Mat>& images_2,
                                        std::vector<std::vector<cv::Point2d>>& image_points_1,
                                        std::vector<std::vector<cv::Point2d>>& image_points_2,
                                        std::vector<std::vector<cv::Point2d>>& world_points,
                                        Params& params) {
  int num_images = images_1.size();
  std::vector<int> res(num_images, 0);
  if(images_1.size() != images_2.size() ||
     params.camera1.detect_params.corner_type != cbdetect::SaddlePoint ||
     params.camera1.detect_params.corner_type != cbdetect::SaddlePoint) return res;

  params.camera1.detect_params.overlay = true;
  params.camera2.detect_params.overlay = true;

  for(int i = 0; i < num_images; ++i) {
    cbdetect::Corner chessboad_corners_1, chessboad_corners_2;
    std::vector<cbdetect::Board> chessboards_1, chessboards_2;
    cbdetect::find_corners(images_1[i], chessboad_corners_1, params.camera1.detect_params);
    cbdetect::boards_from_corners(images_1[i], chessboad_corners_1, chessboards_1, params.camera1.detect_params);
    cbdetect::find_corners(images_2[i], chessboad_corners_2, params.camera2.detect_params);
    cbdetect::boards_from_corners(images_2[i], chessboad_corners_2, chessboards_2, params.camera2.detect_params);

    if(chessboards_1.empty() || chessboards_2.empty()) {
      continue;
    }

    if(params.num_boards == 1) {
      std::sort(chessboards_1.begin(), chessboards_1.end(),
                [](const cbdetect::Board& b1, const cbdetect::Board& b2) {
                  return b1.num > b2.num;
                });
      std::sort(chessboards_2.begin(), chessboards_2.end(),
                [](const cbdetect::Board& b1, const cbdetect::Board& b2) {
                  return b1.num > b2.num;
                });
    } else {
      std::sort(chessboards_1.begin(), chessboards_1.end(), [&](const cbdetect::Board& b1, const cbdetect::Board& b2) -> bool {
        if(b1.num == 0) {
          return false;
        } else if(b2.num == 0) {
          return true;
        } else {
          const auto& chessboard_1 = b1.idx;
          const auto& chessboard_2 = b2.idx;
          cv::Point2d p1(0., 0.), p2(0., 0.);
          for(int j = 0; j < chessboard_1.size(); ++j) {
            for(int i = 0; i < chessboard_1[0].size(); ++i) {
              if(chessboard_1[j][i] < 0) continue;
              p1 += chessboad_corners_1.p[chessboard_1[j][i]];
            }
          }
          for(int j = 0; j < chessboard_2.size(); ++j) {
            for(int i = 0; i < chessboard_2[0].size(); ++i) {
              if(chessboard_2[j][i] < 0) continue;
              p2 += chessboad_corners_1.p[chessboard_2[j][i]];
            }
          }
          p1 /= b1.num;
          p2 /= b2.num;
          return (2 * p1.y + p1.x) < (2 * p2.y + p2.x);
        }
      });
      std::sort(chessboards_2.begin(), chessboards_2.end(), [&](const cbdetect::Board& b1, const cbdetect::Board& b2) -> bool {
        if(b1.num == 0) {
          return false;
        } else if(b2.num == 0) {
          return true;
        } else {
          const auto& chessboard_1 = b1.idx;
          const auto& chessboard_2 = b2.idx;
          cv::Point2d p1(0., 0.), p2(0., 0.);
          for(int j = 0; j < chessboard_1.size(); ++j) {
            for(int i = 0; i < chessboard_1[0].size(); ++i) {
              if(chessboard_1[j][i] < 0) continue;
              p1 += chessboad_corners_2.p[chessboard_1[j][i]];
            }
          }
          for(int j = 0; j < chessboard_2.size(); ++j) {
            for(int i = 0; i < chessboard_2[0].size(); ++i) {
              if(chessboard_2[j][i] < 0) continue;
              p2 += chessboad_corners_2.p[chessboard_2[j][i]];
            }
          }
          p1 /= b1.num;
          p2 /= b2.num;
          return (2 * p1.y + p1.x) < (2 * p2.y + p2.x);
        }
      });
    }

    if(params.camera1.show_cornres) {
      cbdetect::plot_boards(images_1[i], chessboad_corners_1, chessboards_1, params.camera1.detect_params);
    }
    if(params.camera2.show_cornres) {
      cbdetect::plot_boards(images_2[i], chessboad_corners_2, chessboards_2, params.camera1.detect_params);
    }

    int n = std::min<int>(chessboards_1.size(), chessboards_2.size());
    n = std::min<int>(n, params.num_boards);
    res[i] = 0;

    if(params.pattern_type == 0) { // simple full match
      for(int j = 0; j < n; ++j) {
        if(chessboards_1[j].num != chessboards_2[j].num || chessboards_1[j].idx.size() != chessboards_2[j].idx.size()) continue;

        std::vector<std::vector<int>>& chessboard_1 = chessboards_1[j].idx;
        std::vector<std::vector<int>>& chessboard_2 = chessboards_2[j].idx;
        std::vector<cv::Point2d> pi_buf_1, pi_buf_2, pw_buf;

        for(int jj = 0; jj < chessboard_1.size(); ++jj) {
          for(int ii = 0; ii < chessboard_1[0].size(); ++ii) {
            if(chessboard_1[jj][ii] < 0 || chessboard_2[jj][ii] < 0) continue;

            pi_buf_1.emplace_back(chessboad_corners_1.p[chessboard_1[jj][ii]]);
            pi_buf_2.emplace_back(chessboad_corners_2.p[chessboard_2[jj][ii]]);
            pw_buf.emplace_back(cv::Point2d(params.pattern_size * (ii - 1), params.pattern_size * (jj - 1)));
          }
        }
        image_points_1.emplace_back(pi_buf_1);
        image_points_2.emplace_back(pi_buf_2);
        world_points.emplace_back(pw_buf);

        ++res[i];
      }
    } else { // pattern match
      for(int j = 0; j < n; ++j) {
        std::vector<cv::Point2d> pi_buf_1, pi_buf_2, pw_buf;

        int ret = match_two_chessboards(chessboards_1[j].idx, chessboards_2[j].idx,
                                        chessboad_corners_1.p, chessboad_corners_2.p,
                                        pi_buf_1, pi_buf_2, pw_buf,
                                        params.pattern_size);
        if(ret < 0) continue;

        image_points_1.emplace_back(pi_buf_1);
        image_points_2.emplace_back(pi_buf_2);
        world_points.emplace_back(pw_buf);
        ++res[i];
      }
    }
  }

  return res;
}

int extract_corners_stereo(const cv::Mat& images_1,
                           const cv::Mat& images_2,
                           std::vector<cv::Point2d>& image_points_1,
                           std::vector<cv::Point2d>& image_points_2,
                           std::vector<cv::Point2d>& world_points,
                           Params& params) {
  if(params.camera1.detect_params.corner_type != cbdetect::SaddlePoint ||
     params.camera1.detect_params.corner_type != cbdetect::SaddlePoint) return -1;

  params.camera1.detect_params.overlay = true;
  params.camera2.detect_params.overlay = true;

  cbdetect::Corner chessboad_corners_1, chessboad_corners_2;
  std::vector<cbdetect::Board> chessboards_1, chessboards_2;
  cbdetect::find_corners(images_1, chessboad_corners_1, params.camera1.detect_params);
  cbdetect::boards_from_corners(images_1, chessboad_corners_1, chessboards_1, params.camera1.detect_params);
  cbdetect::find_corners(images_2, chessboad_corners_2, params.camera2.detect_params);
  cbdetect::boards_from_corners(images_2, chessboad_corners_2, chessboards_2, params.camera2.detect_params);

  if(params.camera1.show_cornres) {
    cbdetect::plot_boards(images_1, chessboad_corners_1, chessboards_1, params.camera1.detect_params);
  }
  if(params.camera2.show_cornres) {
    cbdetect::plot_boards(images_2, chessboad_corners_2, chessboards_2, params.camera1.detect_params);
  }

  if(chessboards_1.size() != 1 || chessboards_2.size() != 1) return -1;

  if(params.pattern_type == 0) { // simple full match
    if(chessboards_1[0].num != chessboards_2[0].num || chessboards_1[0].idx.size() != chessboards_2[0].idx.size()) return -1;

    std::vector<std::vector<int>>& chessboard_1 = chessboards_1[0].idx;
    std::vector<std::vector<int>>& chessboard_2 = chessboards_2[0].idx;

    for(int jj = 0; jj < chessboard_1.size(); ++jj) {
      for(int ii = 0; ii < chessboard_1[0].size(); ++ii) {
        if(chessboard_1[jj][ii] < 0 || chessboard_2[jj][ii] < 0)
          continue;

        image_points_1.emplace_back(chessboad_corners_1.p[chessboard_1[jj][ii]]);
        image_points_2.emplace_back(chessboad_corners_2.p[chessboard_2[jj][ii]]);
        world_points.emplace_back(cv::Point2d(params.pattern_size * (ii - 1), params.pattern_size * (jj - 1)));
      }
    }
  } else { // pattern match
    int ret = match_two_chessboards(chessboards_1[0].idx, chessboards_2[0].idx,
                                    chessboad_corners_1.p, chessboad_corners_2.p,
                                    image_points_1, image_points_2, world_points,
                                    params.pattern_size);
    if(ret < 0) return -1;
  }

  return 0;
}

} // namespace calib
