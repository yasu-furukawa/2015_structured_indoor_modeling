#ifndef REFINE_SEGMENTATION_H_
#define REFINE_SEGMENTATION_H_

#include <Eigen/Dense>
#include <string>
#include <vector>
#include "data.h"

void NormalizeEvidence(const double min_sigma,
                       const double max_sigma,
                       std::vector<float>* evidence);

void DrawEvidence(const int width, const int height,
                  const std::vector<float>& evidence,
                  const std::string& filename,
                  const double scale);

void DrawEvidence3(const int width, const int height,
                   const Eigen::Vector3f& x_axis,
                   const Eigen::Vector3f& y_axis,
                   const std::vector<Eigen::Vector3d>& evidence,
                   const std::string& filename);
                  
void RefineSegmentation(const floored::Frame& frame,
                        const std::vector<float>& point_evidence,
                        const std::vector<float>& free_space_evidence,
                        const std::vector<Eigen::Vector3d>& normal_evidence,
                        const std::vector<Eigen::Vector2i>& centers,
                        const std::vector<std::vector<Eigen::Vector2i> >& clusters,
                        const std::vector<std::vector<std::pair<int, float> > > visibility,
                        std::vector<int>* segmentation);

void WriteSegmentation(const std::string& output_file,
                       const std::vector<int>& segmentation,
                       const int width,
                       const int height,
                       const int num_label);

void ExpandVisibility(const int width, const int height,
                      std::vector<std::vector<std::pair<int, float> > >* visibility);

#endif  // REFINE_SEGMENTATAION_H_
