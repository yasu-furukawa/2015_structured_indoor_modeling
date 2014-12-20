#ifndef GENERATE_TEXTURE_H_
#define GENERATE_TEXTURE_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace texture {

struct Data {
  std::vector<std::vector<cv::Mat> > panoramas;
  std::vector<Eigen::Matrix4d> panorama_to_globals;
  std::vector<Eigen::Matrix4d> global_to_panoramas;
  Floorplan floorplan;
};

// Domain where texture is generated.
struct Domain {
  Eigen::Vector3d vertices[4];

};
 

}  // namespace
 
#endif  // GENERATE_TEXTURE_H_
