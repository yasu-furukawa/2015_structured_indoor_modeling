#ifndef FLOORED_DYNAMIC_PROGRAMMING_H_
#define FLOORED_DYNAMIC_PROGRAMMING_H_

#include <fstream>
#include <iostream>
#include <Eigen/Dense>

#include <string>
#include <vector>

#include "data.h"

namespace floored {

  void Reconstruct2DDP(const Frame& frame,
                       const std::vector<float>& point_evidence,
                       const std::vector<float>& free_space_evidence,
                       const std::string& directory,
                       Floorplan* floorplan);
  
}  // namespace floored

#endif  // FLOORED_DYNAMIC_PROGRAMMING_H_
