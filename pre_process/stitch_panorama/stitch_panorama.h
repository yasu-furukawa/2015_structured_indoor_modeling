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

class PatchCorrelationResidual;

struct Input {
  std::string directory;
  // Pixel resolution.
  int out_width;
  int out_height;
  int level;
  int margin;
  int subsample;

  std::vector<Eigen::Matrix3d> initial_rotations;
};

struct Patch {
  // Top left corner.
  int x;
  int y;
  int size;

  std::vector<int> indexes;
};

class StitchPanorama {
 public:
  bool Stitch(const Input& input);

  const cv::Mat GetStitchedImage() const { return stitched_image; }

  const std::vector<Eigen::Matrix3d>& GetRefinedRotations() const { return refined_rotations; }

 private:
  bool Init(const std::vector<Eigen::Matrix3d>& initial_rotations);
  bool SetMasks();

  bool RefineCameras();
  void SamplePatches();
  bool Blend(const std::string& filename);

  Eigen::Vector3d ScreenToRay(const Eigen::Vector2d& screen) const;
  Eigen::Vector2d RayToScreen(const Eigen::Vector3d& ray) const;
  // Return (x, y, depth).
  Eigen::Vector3d Project(const int camera, const Eigen::Vector3d& ray) const;
  
  // Input.
  std::string directory;
  int out_width;
  int out_height;
  int level;
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

  friend class PatchCorrelationResidual;
};

class PatchCorrelationResidual {
 public:
 PatchCorrelationResidual(const pre_process::StitchPanorama& stitch_panorama,
                          const int x,
                          const int y,
                          const int size,
                          const int index0,
                          const int index1);

    template <typename T> bool operator()(const T* const param0, const T* const param1, T* residual) const;

    bool GrabPatch(const int index,
                   const Eigen::Matrix3d& projection,
                   std::vector<Eigen::Vector3f>& patch) const;
    
 private:
    const pre_process::StitchPanorama& stitch_panorama_;
    int x_;
    int y_;
    int size_;
    int index0_;
    int index1_;
};
 
}  // namespace pre_process
 
#endif  // STITCH_PANORAMA_H_
