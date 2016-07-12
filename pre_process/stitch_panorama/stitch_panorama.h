#ifndef STITCH_PANORAMA_H_
#define STITCH_PANORAMA_H_

#include <cstdio>
#include <iostream>
#include <fstream>
#include <numeric>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct Input {
  std::vector<Eigen::Matrix3d> intrinsics;
  std::vector<Eigen::Matrix3d> rotations;

  std::vector<cv::Mat> images;
};

struct Output {
  std::vector<Eigen::Matrix3d> refined_intrinsics;
  std::vector<Eigen::Matrix3d> refined_rotations;
  cv::Mat blended_image;
}


#endif  // STITCH_PANORAMA_H_
