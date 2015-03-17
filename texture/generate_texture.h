#pragma once

#include <Eigen/Dense>
#include <vector>
#include "../base/point_cloud.h"

namespace structured_indoor_modeling {

class FileIO;
class Panorama;

void ReadPanoramaToGlobals(const FileIO& file_io,
                           std::vector<Eigen::Matrix4d>* panorama_to_globals);

void Invert(const std::vector<Eigen::Matrix4d>& panorama_to_globals,
            std::vector<Eigen::Matrix4d>* global_to_panoramas);
 
}  // namespace structured_indoor_modeling
 
