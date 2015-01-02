#ifndef DEPTHMAP_REFINER_H__
#define DEPTHMAP_REFINER_H__

#include <vector>

namespace structured_indoor_modeling {

void FillHolesAndSmooth(const int width, const int height, const double invalid,
                        std::vector<double>* depths);

}  // namespace structured_indoor_modeling
 
#endif  // DEPTHMAP_REFINER_H__
