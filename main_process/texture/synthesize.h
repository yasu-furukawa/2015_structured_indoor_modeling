#ifndef STRUCTURED_INDOOR_MODELING_SYNTHESIZE_H_
#define STRUCTURED_INDOOR_MODELING_SYNTHESIZE_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace structured_indoor_modeling {

struct SynthesisData {
SynthesisData(const std::vector<cv::Mat>& projected_textures,
	      const std::vector<double>& weights) :
  projected_textures(projected_textures), weights(weights) {
  }
  
  const std::vector<cv::Mat>& projected_textures;
  const std::vector<double>& weights;
  
  int num_cg_iterations;
  Eigen::Vector2i texture_size;
  int patch_size;
  int margin;
  std::vector<bool> mask;
};

void CollectCandidatePatches(const SynthesisData& synthesis_data,
                             std::vector<cv::Mat>* patches,
                             std::vector<Eigen::Vector2i>* patch_positions);

void SynthesizePoisson(const SynthesisData& synthesis_data,
                       const std::vector<cv::Mat>& patches,
                       const std::vector<Eigen::Vector2i>& patch_positions,
                       const bool vertical_constraint,
                       cv::Mat* floor_texture);
 
}  // namespace structured_indoor_modeling

#endif  // STRUCTURED_INDOOR_MODELING_SYNTHESIZE_H_
