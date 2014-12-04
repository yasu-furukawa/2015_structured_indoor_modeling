#ifndef DEPTHMAP_REFINER_H__
#define DEPTHMAP_REFINER_H__

#include <vector>

void FillHolesAndSmooth(const int width, const int height, const double invalid,
                        std::vector<double>* depths);

#endif  // DEPTHMAP_REFINER_H__
