#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <opencv2/opencv.hpp>


namespace structured_indoor_modeling{

class FileIO;
class PointCloud;
class Panorama;

class DepthFilling{
 public:
  DepthFilling(){}
  void Init(const FileIO &file_io, int id);
  void Init(const PointCloud &point_cloud, const Panorama &panorama);
  void fill_hole();
  
  //just for debugging
  void SaveDepthmap(std::string path);
 private:
  std::vector <double> depthmap;
  int depthwidth;
  int depthheight;
  double max_depth;
  double min_depth;
};

} // namespace
