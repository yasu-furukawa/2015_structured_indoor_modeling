#pragma once

#include <Eigen/Dense>
#include <vector>
#include "../base/point_cloud.h"

namespace structured_indoor_modeling {

class FileIO;
class Panorama;

int GetEndPanorama(const FileIO& file_io, const int start_panorama);
 
void ReadPanoramas(const FileIO& file_io,
                   const int start_panorama,
                   const int end_panorama,
                   const int num_pyramid_levels,
                   std::vector<std::vector<Panorama> >* panoramas);

void ReadPanoramaToGlobals(const FileIO& file_io,
                           const int start_panorama,
                           const int end_panorama,
                           std::vector<Eigen::Matrix4d>* panorama_to_globals);

void ReadPointClouds(const FileIO& file_io,
                     const int start_panorama,
                     const int end_panorama,
                     std::vector<PointCloud>* point_clouds);
 
void Invert(const std::vector<Eigen::Matrix4d>& panorama_to_globals,
            std::vector<Eigen::Matrix4d>* global_to_panoramas);
 
}  // namespace structured_indoor_modeling
 
