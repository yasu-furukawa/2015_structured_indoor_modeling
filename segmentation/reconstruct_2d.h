#ifndef FLOORED_RECONSTRUCT_2D_H_
#define FLOORED_RECONSTRUCT_2D_H_

#include <string>
#include <vector>

#include "data.h"

namespace floored {

void Reconstruct2D(const Frame& frame,
                   const std::vector<float>& point_evidence,
                   const std::vector<float>& free_space_evidence,
                   const std::string& directory,
                   Floorplan* floorplan);

}  // namespace floored

#endif  // FLOORED_RECONSTRUCT_2D_H_
