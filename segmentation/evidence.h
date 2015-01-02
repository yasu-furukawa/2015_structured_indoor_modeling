#ifndef SEGMENTATION_EVIDENCE_H_
#define SEGMENTATION_EVIDENCE_H_

#include <string>
#include <vector>

#include "data.h"

namespace structured_indoor_modeling {

void LoadEvidence(const std::string& filename, std::vector<float>* evidence);
void WriteEvidence(const std::string& filename, const std::vector<float>& evidence);

void LoadEvidence3(const std::string& filename, std::vector<Eigen::Vector3d>* evidence);
void WriteEvidence3(const std::string& filename, const std::vector<Eigen::Vector3d>& evidence);

void SetPointEvidence(const std::vector<Sweep>& sweeps,
                      const Frame& frame,
                      const std::string& directory,
                      std::vector<float>* point_evidence);

void SetPointEvidence(const std::vector<Sweep>& sweeps,
                      const Frame& frame,
                      const std::string& directory,
                      std::vector<float>* point_evidence,
                      std::vector<Eigen::Vector3d>* normal_evidence);
 
void SetFreeSpaceEvidence(const std::vector<Sweep>& sweeps,
                          const Frame& frame,
                          const std::string& directory,
                          std::vector<float>* free_space_evidence);
 
void DrawEvidenceToImage(const Frame& frame,
                         const std::string directory,
                         const std::vector<float>& point_evidence,
                         const std::vector<float>& free_space_evidence);

double ScalePointEvidence(const double input);
 
double ScaleFreeSpaceEvidence(const double input);

void ConvertEvidence(const int width,
                     const int height,
                     const std::vector<float>& evidence,
                     const double scale,
                     std::vector<unsigned char>* evidence_int);

}  // namespace structured_indoor_modeling

#endif  // SEGMENTATION_EVIDENCE_H_
