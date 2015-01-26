#ifndef PANORAMA_H_
#define PANORAMA_H_

/*
  Panorama has both RGB and D information. Both information are stored
  as a 2D array with a simple cylindrical mapping. The RGB and D
  cameras do not share the same optical center, but the D information
  was mapped to the RGB camera, and one needs not worry about this
  issue in using this class.

  One needs to consider 

global/local/image/depth


 */

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "file_io.h"

namespace structured_indoor_modeling {

class Panorama {
public:
  Panorama();
  bool Init(const FileIO& file_io, const int panorama);
  bool InitWithoutLoadingImages(const FileIO& file_io, const int panorama);

  Eigen::Vector2d Project(const Eigen::Vector3d& global) const;
  Eigen::Vector3d Unproject(const Eigen::Vector2d& pixel,
                            const double distance) const;

  Eigen::Vector2d ProjectToDepth(const Eigen::Vector3d& global) const;

  Eigen::Vector3d GlobalToLocal(const Eigen::Vector3d& global) const;
  Eigen::Vector3d LocalToGlobal(const Eigen::Vector3d& local) const;

  const Eigen::Vector3d& GetCenter() const { return center; }

  double GetAverageDistance() const { return average_distance; }
  Eigen::Vector2d RGBToDepth(const Eigen::Vector2d& pixel) const;
  Eigen::Vector2d DepthToRGB(const Eigen::Vector2d& depth_pixel) const;

  void MakeOnlyBackgroundBlack();

  Eigen::Vector3f GetRGB(const Eigen::Vector2d& pixel) const;
  double GetDepth(const Eigen::Vector2d& depth_pixel) const;
  
  int Width() const { return width; }
  int Height() const { return height; }
  int DepthWidth() const { return depth_width; }
  int DepthHeight() const { return depth_height; }

  bool IsInsideRGB(const Eigen::Vector2d& pixel) const;
  bool IsInsideDepth(const Eigen::Vector2d& depth_pixel) const;

  void ResizeRGB(const Eigen::Vector2i& size);
  
private:
  void InitDepthImage(const FileIO& file_io, const int panorama);
  void InitCameraParameters(const FileIO& file_io, const int panorama);

  Eigen::Matrix4d global_to_local;
  Eigen::Matrix4d local_to_global;
  Eigen::Vector3d center;
  double phi_range;

  cv::Mat rgb_image;
  std::vector<double> depth_image;

  // Redundant but useful values.
  int width;
  int height;
  int depth_width;
  int depth_height;

  double phi_per_pixel;
  double phi_per_depth_pixel;

  double average_distance;
};

}  // namespace structured_indoor_modeling

#endif  // PANORAMA_H_
