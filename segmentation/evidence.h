#ifndef FLOORED_EVIDENCE_H_
#define FLOORED_EVIDENCE_H_

#include <string>
#include <vector>

#include "data.h"

namespace floored {

void LoadEvidence(const std::string& filename, std::vector<float>* evidence);

void WriteEvidence(const std::string& filename, const std::vector<float>& evidence);

void SetPointEvidence(const std::vector<Sweep>& sweeps,
                      const Frame& frame,
                      const std::string& directory,
                      std::vector<float>* point_evidence);

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
 
 
}  // namespace floored

#endif  // FLOORED_EVIDENCE_H_
