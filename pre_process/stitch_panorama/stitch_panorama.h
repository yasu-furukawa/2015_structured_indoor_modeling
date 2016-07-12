#ifndef STITCH_PANORAMA_H_
#define STITCH_PANORAMA_H_

#include <cstdio>
#include <iostream>
#include <fstream>
#include <numeric>
#include <set>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace pre_process {

struct Input {
  std::string directory;
  // Pixel resolution.
  int out_width;
  int out_height;

  int margin;
  int subsample;
};

struct Patch {
  // Top left corner.
  int x;
  int y;
  int size;

  std::set<int> indexes;
};

class StitchPanorama {
 public:
  bool Stitch(const Input& input);

  const cv::Mat GetStitchedImage() const { return stitched_image; }
  
 private:
  bool Init();
  bool SetMasks();

  bool RefineCameras();
  void SamplePatches();
  bool Blend();

  Eigen::Vector3d ScreenToRay(const Eigen::Vector2d& screen) const;
  Eigen::Vector2d RayToScreen(const Eigen::Vector3d& ray) const;
  // Return (x, y, depth).
  Eigen::Vector3d Project(const int camera, const Eigen::Vector3d& ray) const;
  
  // Input.
  std::string directory;
  int out_width;
  int out_height;
  int margin;
  int subsample;

  int num_cameras;
  Eigen::Matrix3d intrinsics;
  std::vector<Eigen::Matrix3d> rotations;
  std::vector<Eigen::Matrix3d> projections;
  std::vector<cv::Mat> images;

  // Intermediate data.
  std::vector<cv::Mat> masks;
  std::vector<Patch> patches;
  
  // Output.
  std::vector<Eigen::Matrix3d> refined_intrinsics;
  std::vector<Eigen::Matrix3d> refined_rotations;
  
  cv::Mat stitched_image;
};

}  // namespace pre_process
 
#endif  // STITCH_PANORAMA_H_
