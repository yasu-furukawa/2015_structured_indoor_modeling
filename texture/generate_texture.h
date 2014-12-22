#ifndef GENERATE_TEXTURE_H_
#define GENERATE_TEXTURE_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace texture {

// Domain where texture is generated.
struct Domain {
  Eigen::Vector3d vertices[4];

};
 

}  // namespace
 
#endif  // GENERATE_TEXTURE_H_
