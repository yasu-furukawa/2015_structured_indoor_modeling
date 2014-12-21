#ifndef FLOORED_TRANSFORM_H_
#define FLOORED_TRANSFORM_H_

#include <Eigen/Dense>
#include <vector>
#include "data.h"

namespace floored {

void ReadTransforms(const std::string directory, const int num, const int reference_num,
                    std::vector<Eigen::Matrix4f>* transforms);
  
Eigen::Vector3f TransformPoint(const Eigen::Matrix4f& transform, const Eigen::Vector3f& point);

void TransformSweeps(const std::vector<Eigen::Matrix4f>& transforms,
                     std::vector<Sweep>* sweeps);

void ConvertSweepsToFrame(const Frame& frame, std::vector<Sweep>* sweeps);

}  // namespace floored

#endif  // FLOORED_TRANSFORM_H_
