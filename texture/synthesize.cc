#include "synthesize.h"
#include "../base/numeric/sparseMat.h"
#include "../base/numeric/sparseMatSolve.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

double AverageAbsoluteDifference(const cv::Mat& texture,
                                 const cv::Mat& patch,
                                 const Eigen::Vector2i& x_range,
                                 const Eigen::Vector2i& y_range) {
  const cv::Vec3b kHole(0, 0, 0);
  const int kNumChannels = 3;
  double sum = 0;
  int denom = 0;
  for (int y = y_range[0]; y < y_range[1]; ++y) {
    for (int x = x_range[0]; x < x_range[1]; ++x) {
      if (texture.at<cv::Vec3b>(y, x) == kHole)
        continue;

      for (int c = 0; c < kNumChannels; ++c) 
        sum += abs(((int)texture.at<cv::Vec3b>(y, x)[c]) -
                   ((int)patch.at<cv::Vec3b>(y - y_range[0], x - x_range[0])[c]));
      denom += kNumChannels;
    }
  }

  if (denom == 0) {
    cerr << "Impossible in AAD" << endl;
    exit (1);
  }

  return sum / denom;
}

void CopyPatch(const std::vector<bool>& mask,
               const cv::Mat& patch,
               const Eigen::Vector2i& x_range,
               const Eigen::Vector2i& y_range,
               cv::Mat* floor_texture) {
  const cv::Vec3b kHole(0, 0, 0);
  const int kNumChannels = 3;
  const int width  = floor_texture->cols;
  const int height = floor_texture->rows;

  for (int y = y_range[0]; y < y_range[1]; ++y) {
    for (int x = x_range[0]; x < x_range[1]; ++x) {
      if (mask[y * width + x])
        floor_texture->at<cv::Vec3b>(y, x) = patch.at<cv::Vec3b>(y - y_range[0], x - x_range[0]);
    }
  }
}
  
bool FindGridWithMostValidSomeHoles(const SynthesisData& synthesis_data,
                                    const cv::Mat& floor_texture,
                                    Eigen::Vector2i* grid) {
  const cv::Vec3b kHole(0, 0, 0);
  const int width      = synthesis_data.texture_size[0];
  const int height     = synthesis_data.texture_size[1];
  const int patch_size = synthesis_data.patch_size;
  const int margin     = synthesis_data.margin;

  const int grid_width  = width / (patch_size - margin) + 1;
  const int grid_height = height / (patch_size - margin) + 1;

  int best_count = 0;
  for (int j = 0; j < grid_height; ++j) {
    const int min_y = j * (patch_size - margin);
    const int max_y = min(height, min_y + patch_size);
    
    for (int i = 0; i < grid_width; ++i) {
      const int min_x = i * (patch_size - margin);
      const int max_x = min(width, min_x + patch_size);

      // Count the number of non-black pixels.
      int num_valids = 0;
      int num_holes = 0;
      for (int y = min_y; y < max_y; ++y) {
        for (int x = min_x; x < max_x; ++x) {
          const int index = y * width + x;
          if (!synthesis_data.mask[index])
            continue;

          if (floor_texture.at<cv::Vec3b>(y, x) == kHole)
            ++num_holes;
          else
            ++num_valids;
        }
      }
      if (num_holes != 0 && num_valids > best_count) {
        *grid = Vector2i(i, j);
        best_count = num_valids;
      }
    }
  }

  if (best_count == 0)
    return false;
  else
    return true;
}  

void SetDataForBlending(const int width,
                        const std::vector<bool>& mask,
                        const cv::Mat& patch,
                        const Eigen::Vector2i& x_range,
                        const Eigen::Vector2i& y_range,
                        std::vector<std::vector<Eigen::Vector3d> >* laplacians,
                        std::vector<std::vector<Eigen::Vector3d> >* values) {
  const int kNumChannels = 3;
  for (int y = y_range[0] + 1; y < y_range[1] - 1; ++y) {
    const int patch_y = y - y_range[0];
    for (int x = x_range[0] + 1; x < x_range[1] - 1; ++x) {
      const int patch_x = x - x_range[0];
      const int index = y * width + x;
      if (!mask[index])
        continue;

      Vector3d laplacian;
      for (int c = 0; c < kNumChannels; ++c) {
        int count = 0;
        if (mask[index - 1]) {
          laplacian[c] -= patch.at<cv::Vec3b>(patch_y, patch_x - 1)[c];
          ++count;
        }
        if (mask[index + 1]) {
          laplacian[c] -= patch.at<cv::Vec3b>(patch_y, patch_x + 1)[c];
          ++count;
        }
        if (mask[index - width]) {
          laplacian[c] -= patch.at<cv::Vec3b>(patch_y - 1, patch_x)[c];
          ++count;
        }
        if (mask[index + width]) {
          laplacian[c] -= patch.at<cv::Vec3b>(patch_y + 1, patch_x)[c];
          ++count;
        }

        laplacian[c] += count * patch.at<cv::Vec3b>(patch_y, patch_x)[c];
      }
      laplacians->at(index).push_back(laplacian);
    }
  }


  for (int y = y_range[0]; y < y_range[1]; ++y) {
    const int patch_y = y - y_range[0];
    for (int x = x_range[0]; x < x_range[1]; ++x) {
      const int patch_x = x - x_range[0];
      const int index = y * width + x;
      if (!mask[index])
        continue;
      Vector3d value;
      for (int c = 0; c < kNumChannels; ++c) {
        value[c] = patch.at<cv::Vec3b>(patch_y, patch_x)[c];
      }
      values->at(index).push_back(value);
    }
  }
}

void PoissonBlend(const SynthesisData& synthesis_data,
                  const vector<vector<Vector3d> >& laplacians,
                  const vector<vector<Vector3d> >& values,
                  cv::Mat* floor_texture) {
  const int width  = synthesis_data.texture_size[0];
  const int height = synthesis_data.texture_size[1];
  vector<Vector3d> average_laplacian(width * height, Vector3d(0, 0, 0));
  int index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      if (laplacians[index].empty())
        continue;
      for (const auto rgb : laplacians[index])
        average_laplacian[index] += rgb;
      average_laplacian[index] /= laplacians[index].size();
    }
  }

  // variable to index.
  vector<Vector2i> indexes;
  map<pair<int, int>, int> inverse_indexes;
  index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      if (synthesis_data.mask[index]) {
        inverse_indexes[make_pair(x, y)] = indexes.size();
        indexes.push_back(Vector2i(x, y));
      }
    }
  }

  // For each channel.
  const int kNumChannels = 3;
  for (int c = 0; c < kNumChannels; ++c) {
    vector<vector<double> > A;
    vector<double> b;
    // Values constraints.
    int index = 0;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x, ++index) {
        // Add a constraint, only when a single value is specified.
        if (values[index].size() != 1)
          continue;
        const int variable_index = inverse_indexes[make_pair(x, y)];
        vector<double> a(indexes.size(), 0);
        a[variable_index] = 1.0;
        A.push_back(a);
        b.push_back(values[index][0][c]);
      }
    }

    // Laplacian constraints.
    for (int y = 1; y < height - 1; ++y) {
      for (int x = 1; x < width - 1; ++x) {
        const int index = y * width + x;
        if (laplacians[index].empty())
          continue;
        vector<double> a(indexes.size(), 0);
        int count = 0;
        if (synthesis_data.mask[index - 1]) {
          a[inverse_indexes[pair<int, int>(x - 1, y)]] = -1;
          ++count;
        }
        if (synthesis_data.mask[index + 1]) {
          a[inverse_indexes[pair<int, int>(x + 1, y)]] = -1;
          ++count;
        }
        if (synthesis_data.mask[index - width]) {
          a[inverse_indexes[pair<int, int>(x, y - 1)]] = -1;
          ++count;
        }
        if (synthesis_data.mask[index + width]) {
          a[inverse_indexes[pair<int, int>(x, y + 1)]] = -1;
          ++count;
        }
        a[inverse_indexes[pair<int, int>(x, y)]] = count;
        b.push_back(average_laplacian[index][c]);
      }
    }

    // If no constraint, add value constraints.
    for (int v = 0; v < (int)indexes.size(); ++v) {
      bool zero = true;
      for (int r = 0; r < (int)A.size(); ++r) {
        if (A[v][r] != 0) {
          zero = false;
          break;
        }
      }

      if (zero) {
        const int index = indexes[v][1] * width + indexes[v][0];
        if (values[index].empty()) {
          cerr << "Impossible here." << endl;
          exit (1);
        }
        double average = 0.0;
        for (const auto& value : values[index])
          average += value[c];
        average /= values[index].size();

        vector<double> a(indexes.size(), 0);
        a[v] = 1.0;
        b.push_back(average);
      }
    }
    
    CsparseMat<double> A2(A);
    Cvec<double> b2(b.size());
    for (int i = 0; i < (int)b.size(); ++i)
      b2[i] = b[i];

    Cvec<double> x(indexes.size());
    const int kIteration = 50;
    GaussSeidel<double>(A2, b2, x, kIteration);
    // Richardson<double>(A2, b2, x, kIteration, 0.2);

    for (int v = 0; v < (int)x.size(); ++v) {
      floor_texture->at<cv::Vec3b>(indexes[v][1], indexes[v][0])[c] = 
        static_cast<unsigned char>(x[v]);
    }
  }
}
  
}  // namespace

void CollectCandidatePatches(const SynthesisData& synthesis_data,
                             std::vector<cv::Mat>* patches) {
  const std::vector<bool>& mask = synthesis_data.mask;
  const Eigen::Vector2i& texture_size = synthesis_data.texture_size;
  const int patch_size = synthesis_data.patch_size;
  
  for (int y = 0; y < texture_size[1] - patch_size; ++y) {
    for (int x = 0; x < texture_size[0] - patch_size; ++x) {
      // Checks if the patch at (x, y) is in a mask.
      bool fail = false;
      for (int j = 0; j < patch_size; ++j) {
        if (fail)
          break;
        const int ytmp = y + j;
        for (int i = 0; i < patch_size; ++i) {
          const int xtmp = x + i;
          if (!mask[ytmp * texture_size[0] + xtmp]) {
            fail = true;
            break;
          }
        }
      }
      if (fail)
        continue;

      for (const auto& projected_texture : synthesis_data.projected_textures) {
        cv::Mat patch(patch_size, patch_size, CV_8UC3);
        bool fail_texture = false;
        for (int j = 0; j < patch_size; ++j) {
          if (fail_texture)
            break;
          const int ytmp = y + j;
          for (int i = 0; i < patch_size; ++i) {
            const int xtmp = x + i;
            const cv::Vec3b rgb = projected_texture.at<cv::Vec3b>(ytmp, xtmp);
            if (rgb == cv::Vec3b(0, 0, 0)) {
              fail_texture = true;
              break;
            }
            patch.at<cv::Vec3b>(j, i) = rgb;
          }
        }
        if (!fail_texture) {
          patches->push_back(patch);
        }
      }
    }
  }
}

void Synthesize(const SynthesisData& synthesis_data,
                const std::vector<cv::Mat>& patches,
                cv::Mat* floor_texture) {
  // First identify the projected texture with the most area.
  const cv::Vec3b kHole(0, 0, 0);
  const int kInvalid = -1;
  int best_index = kInvalid;
  int best_count = 0;
  for (int i = 0; i < synthesis_data.projected_textures.size(); ++i) {
    const cv::Mat& image = synthesis_data.projected_textures[i];
    // Count the number of non-black pixels.
    int count = 0;
    for (int y = 0; y < image.rows; ++y) {
      for (int x = 0; x < image.cols; ++x) {
        if (image.at<cv::Vec3b>(y, x) != kHole)
          ++count;
      }
    }
    if (best_index == kInvalid || count > best_count) {
      best_index = i;
      best_count = count;
    }
  }

  // Simple case. Use one image to synthesize.
  cv::imshow("source", synthesis_data.projected_textures[best_index]);
  // cv::waitKey(0);

  //----------------------------------------------------------------------    
  const int width      = synthesis_data.texture_size[0];
  const int height     = synthesis_data.texture_size[1];
  const int patch_size = synthesis_data.patch_size;
  const int margin     = synthesis_data.margin;
  
  // Copy reference to the output.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const cv::Vec3b color = synthesis_data.projected_textures[best_index].at<cv::Vec3b>(y, x);
      const int index = y * width + x;
      if (synthesis_data.mask[index] && color != kHole) {
        floor_texture->at<cv::Vec3b>(y, x) = color;        
      }
    }
  }

  const double kMarginResidual = 2.5;
  // Try patch synthesis at grid points
  // i * (patch_size - margin), j * (patch_size - margin).
  const int grid_width  = (width - patch_size - 1) / (patch_size - margin) + 1;
  const int grid_height = (height - patch_size - 1) / (patch_size - margin) + 1;

  // Constraints for poisson blending later.
  vector<vector<Vector3d> > laplacians(width * height);
  vector<vector<Vector3d> > values(width * height);
  
  while (true) {
    // Find a grid position with the most constraints.
    Vector2i best_grid;
    if (!FindGridWithMostValidSomeHoles(synthesis_data, *floor_texture, &best_grid))
      break;

    // Put a texture at best_grid from candidates.
    const int min_x = best_grid[0] * (patch_size - margin);
    const int max_x = min(width, min_x + patch_size);
    const int min_y = best_grid[1] * (patch_size - margin);
    const int max_y = min(height, min_y + patch_size);
    const Vector2i x_range(min_x, max_x);
    const Vector2i y_range(min_y, max_y);

    vector<double> residuals(patches.size(), 0);
    for (int p = 0; p < patches.size(); ++p) {
      residuals[p] = AverageAbsoluteDifference(*floor_texture, patches[p], x_range, y_range);
    }
    const double min_residual = *min_element(residuals.begin(), residuals.end());
    const double threshold = min_residual + kMarginResidual;
    vector<int> candidates;
    for (int i = 0; i < (int)residuals.size(); ++i) {
      if (residuals[i] <= threshold)
        candidates.push_back(i);
    }

    // cerr << "Candidate: " << (int)candidates.size() << '/' << residuals.size() << ' '
    // << min_residual << ' ' << threshold;
    const int patch_id = candidates[rand() % candidates.size()];
    // cerr << "  patch: " << patch_id << endl;
    CopyPatch(synthesis_data.mask, patches[patch_id], x_range, y_range, floor_texture);

    SetDataForBlending(width, synthesis_data.mask, patches[patch_id], x_range, y_range, &laplacians, &values);
  }

  // Poisson blend.
  PoissonBlend(synthesis_data, laplacians, values, floor_texture);  
}

}  // namespace structured_indoor_modeling
