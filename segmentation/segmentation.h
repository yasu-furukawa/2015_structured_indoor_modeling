#include <Eigen/Dense>
#include <string>
#include <vector>
#include "data.h"
#include "../calibration/file_io.h"

namespace segmentation {

void ReadSweeps(const file_io::FileIO& file_io,
                std::vector<Sweep>* sweeps);
  
void NormalizeIntensity(const double min_sigma,
                        const double max_sigma,
                        Sweep* sweep);

void ConvertSweepFromLocalToGlobal(const Eigen::Matrix4d& local_to_global,
                                   Sweep* sweep);

void RotateSweep(const double angle, Sweep* sweep);

float ComputeAverageDistance(const std::vector<Sweep>& sweeps);

void InitializeFrame(const bool recompute,
                     const std::string directory,
                     const std::vector<Sweep>& sweeps,
                     const float average_distance,
                     Frame* frame);

void ComputeFrame(const std::string directory,
                  const std::vector<Sweep>& sweeps,
                  const float average_distance,
                  Frame* frame);
 
void SetRanges(const std::vector<Sweep>& sweeps,
               const float average_distance,
               Frame* frame);

void InitializeEvidence(const bool recompute,
                        const std::vector<Sweep>& sweeps,
                        const Frame& frame,
                        const std::string directory,
                        std::vector<float>* point_evidence,
                        std::vector<float>* free_space_evidence,
                        std::vector<Eigen::Vector3d>* normal_evidence);
  
}  // namespace segmentation
