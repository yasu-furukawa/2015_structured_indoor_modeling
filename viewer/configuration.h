#ifndef CONFIGURATION_H__
#define CONFIGURATION_H__

#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "../base/file_io.h"

namespace structured_indoor_modeling {

  /*
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
  */
  
struct Configuration {
  std::string data_directory;
  // std::vector<PanoramaConfiguration> panorama_configurations;

  // Parameters.
  double air_angle;
  double air_field_of_view_degrees;
  double floorplan_angle;
  double floorplan_field_of_view_degrees;
};

/*
void ReadPanoramaConfiguration(const std::string& data_directory,
                               const int pano_index,
                               PanoramaConfiguration* panorama_configuration);
void ReadConfiguration(const std::string filename, Configuration* configuration);
*/

}  // namespace structured_indoor_modeling

#endif  // CONFIGURATION_H__
