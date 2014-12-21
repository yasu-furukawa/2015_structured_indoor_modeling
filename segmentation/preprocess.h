#ifndef FLOORED_PREPROCESS_H_
#define FLOORED_PREPROCESS_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "data.h"

namespace floored {

void ReadSweeps(const std::string directory, const int num,
                std::vector<Sweep>* sweeps);

void FilterSweeps(const float lower_height,
                  const float upper_height,
                  std::vector<Sweep>* sweeps);
 
void ComputeFrame(const std::string directory,
                  const std::vector<Sweep>& sweeps,
                  const float average_distance,
                  Frame* frame);

void RansacPlane(const std::vector<OrientedPoint>& points,
                 const Eigen::Vector3f& axis,
                 const int iteration,
                 const float threshold,
                 Eigen::Vector3f* orthogonal_axis,
                 std::vector<OrientedPoint>* inliers);

void WriteSweepsAsPly(const std::vector<Sweep>& sweeps,
                     const std::string filename);

float ComputeAverageDistance(const std::vector<Sweep>& sweeps);
 
}  // namespace floored

#endif  // FLOORED_PREPROCESS_H_
