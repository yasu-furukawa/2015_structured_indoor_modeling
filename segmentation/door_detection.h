#ifndef ROOM_SEGMENTATION_DOOR_DETECTION_H_
#define ROOM_SEGMENTATION_DOOR_DETECTION_H_

#include <Eigen/Dense>
#include "../base/ply/points.h"
#include "data.h"

namespace room_segmentation {
  void ConvertPointsToSweep(const ply::Points& points, floored::Sweep* sweep);

  void SetRanges(const std::vector<floored::Sweep>& sweeps,
                 const float average_distance,
                 floored::Frame* frame);

  void ComputeFrame(const std::string directory,
                    const std::vector<floored::Sweep>& sweeps,
                    const float average_distance,
                    floored::Frame* frame);

  float ComputeAverageDistance(const std::vector<floored::Sweep>& sweeps);

  void DetectDoors(const std::vector<floored::Sweep>& sweeps,
                   const floored::Frame& frame,
                   const std::string directory,
                   const std::vector<float>& point_evidence,
                   const std::vector<float>& free_space_evidence,
                   std::vector<float>* door_detection);

  void WriteClustering(const int width,
                       const int height,
                       const std::vector<int>& centers,
                       const std::vector<std::vector<int> >& clusters,
                       const int subsample,
                       const std::string filename);

  void LoadClustering(const std::string filename,
                      std::vector<Eigen::Vector2i>* centers,
                      std::vector<std::vector<Eigen::Vector2i> >* clusters);

  float VisibilityDistance(const std::vector<std::pair<int, float> >& lhs,
                           const std::vector<std::pair<int, float> >& rhs);

  void WriteVisibility(const std::string& filename,
                       const std::vector<std::vector<std::pair<int, float> > >& visibility,
                       const int width,
                       const int height,
                       const int subsample);
  void LoadVisibility(const std::string& filename,
                      std::vector<std::vector<std::pair<int, float> > >* visibility,
                      int* width,
                      int* height,
                      int* subsample);
    
  
}  // namespace room_segmentation

#endif // ROOM_SEGMENTATION_DOOR_DETECTION_H_
