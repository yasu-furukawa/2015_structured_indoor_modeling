#ifndef CONFIGURATION_H__
#define CONFIGURATION_H__

#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

struct PanoramaConfiguration {
  std::string rgb_image_filename;
  std::string depth_image_filename;

  // Camera center.
  Eigen::Vector3d center;
  // Camera x, y, z axes. Each column is an axis. So this is a transformation from local to global.
  Eigen::Matrix3d local_to_global;
  // phi (in radian) per image y.
  double phi_range;
};

struct Configuration {
  std::string data_directory;
  std::vector<PanoramaConfiguration> panorama_configurations;
};

void ReadPanoramaConfiguration(const std::string& data_directory,
                               const int pano_index,
                               PanoramaConfiguration* panorama_configuration);
void ReadConfiguration(const std::string filename, Configuration* configuration);

#endif  // CONFIGURATION_H__
