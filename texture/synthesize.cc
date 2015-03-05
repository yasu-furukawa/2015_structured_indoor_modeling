#include <Eigen/Sparse>
#include <fstream>
#include "synthesize.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

double AverageAbsoluteDifference(const cv::Mat& texture,
                                 const cv::Mat& patch,
                                 const Eigen::Vector2i& x_range,
                                 const Eigen::Vector2i& y_range,
                                 const double current_threshold) {
  const cv::Vec3b kHole(0, 0, 0);
  const int kNumChannels = 3;
  double sum = 0;
  int denom = 0;

  const double kLarge = 10000.0;

  for (int y = y_range[0]; y < y_range[1]; ++y) {
    for (int x = x_range[0]; x < x_range[1]; ++x) {
      if (texture.at<cv::Vec3b>(y, x) == kHole)
        continue;

      for (int c = 0; c < kNumChannels; ++c) 
        sum += abs(((int)texture.at<cv::Vec3b>(y, x)[c]) -
                   ((int)patch.at<cv::Vec3b>(y - y_range[0], x - x_range[0])[c]));
      denom += kNumChannels;

      // Threshold check.
      if (denom > 30 && current_threshold * denom < sum)
        return kLarge;
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
  
bool FindGridWithMostValid(const SynthesisData& synthesis_data,
                           const cv::Mat& floor_texture,
                           const std::set<std::pair<int, int> >& visited_grids,
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

      if (visited_grids.find(pair<int, int>(i, j)) != visited_grids.end())
        continue;

      // Count the number of non-black pixels.
      int num_valids = 0;
      int num_holes  = 0;
      for (int y = min_y; y < max_y; ++y) {
        for (int x = min_x; x < max_x; ++x) {
          const int index = y * width + x;
          if (!synthesis_data.mask[index])
            continue;

          if (floor_texture.at<cv::Vec3b>(y, x) != kHole)
            ++num_valids;
          else
            ++num_holes;
        }
      }
      if (num_valids > best_count) {
        *grid = Vector2i(i, j);
        best_count = num_valids;
      }
      if (num_holes == 0 && num_valids != 0) {
        return true;
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
                        const std::vector<bool>& initial_mask,
                        const std::vector<bool>& pixel_guarded,
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
      Vector3d laplacian(0, 0, 0);

      if (initial_mask[index] != initial_mask[index - 1] ||
          initial_mask[index] != initial_mask[index + 1] ||
          initial_mask[index] != initial_mask[index - width] ||
          initial_mask[index] != initial_mask[index + width]) {
        laplacians->at(index).push_back(laplacian);
        continue;
      }
      
      Vector3i counts(0, 0, 0);
      for (int c = 0; c < kNumChannels; ++c) {
        if (mask[index - 1]) {
          laplacian[c] -= patch.at<cv::Vec3b>(patch_y, patch_x - 1)[c];
          ++counts[c];
        }
        if (mask[index + 1]) {
          laplacian[c] -= patch.at<cv::Vec3b>(patch_y, patch_x + 1)[c];
          ++counts[c];
        }
        if (mask[index - width]) {
          laplacian[c] -= patch.at<cv::Vec3b>(patch_y - 1, patch_x)[c];
          ++counts[c];
        }
        if (mask[index + width]) {
          laplacian[c] -= patch.at<cv::Vec3b>(patch_y + 1, patch_x)[c];
          ++counts[c];
        }
        
        laplacian[c] += counts[c] * patch.at<cv::Vec3b>(patch_y, patch_x)[c];
      }
      if (counts[0] != 0)
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

      if (pixel_guarded[index])
        continue;

      // if (!initial_mask[index])
      // continue;
      
      Vector3d value;
      for (int c = 0; c < kNumChannels; ++c) {
        value[c] = patch.at<cv::Vec3b>(patch_y, patch_x)[c];
      }
      values->at(index).push_back(value);
    }
  }
}

void PoissonBlendSub(const SynthesisData& synthesis_data,
                     const vector<vector<Vector3d> >& laplacians,
                     const vector<vector<Vector3d> >& values,
                     const vector<Vector3d>& average_laplacian,
                     const vector<Vector2i>& indexes,
                     map<pair<int, int>, int>& inverse_indexes,
                     const int channel,
                     cv::Mat* floor_texture) {
  const int width      = synthesis_data.texture_size[0];
  const int height     = synthesis_data.texture_size[1];

  vector<Eigen::SparseVector<double> > A;
  vector<double> b;
  // Values constraints.
  int index = 0;
  int value_count = 0;
  vector<bool> constraints(indexes.size(), false);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      // Add a constraint, only when a single value is specified.
      if (values[index].size() != 1)
        continue;
      ++value_count;
      const int variable_index = inverse_indexes[make_pair(x, y)];
      Eigen::SparseVector<double> a(indexes.size());
      a.coeffRef(variable_index) = 1.0;
      constraints[variable_index] = true;
      A.push_back(a);
      b.push_back(values[index][0][channel]);
    }
  }
  
  // Laplacian constraints.
  int laplacian_count = 0;
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      const int index = y * width + x;
      if (laplacians[index].empty())
        continue;
      Eigen::SparseVector<double> a(indexes.size());
      int count = 0;
      vector<Vector2i> neighbors;
      neighbors.push_back(Vector2i(x - 1, y));
      neighbors.push_back(Vector2i(x + 1, y));
      neighbors.push_back(Vector2i(x, y - 1));
      neighbors.push_back(Vector2i(x, y + 1));
      for (const auto neighbor : neighbors) {
        const int index = neighbor[1] * width + neighbor[0];
        if (synthesis_data.mask[index]) {
          if (inverse_indexes.find(pair<int, int>(neighbor[0], neighbor[1])) ==
              inverse_indexes.end()) {
            cout << "w1" << endl;
            exit (1);
          }
          const int variable_index = inverse_indexes[pair<int, int>(neighbor[0], neighbor[1])];
          a.coeffRef(variable_index) = -1;
          constraints[variable_index] = true;
          ++count;
        }
      }
      {
        if (inverse_indexes.find(pair<int, int>(x, y)) == inverse_indexes.end()){
          cout << "w5" << endl;
          exit (1);
        }
        const int variable_index = inverse_indexes[pair<int, int>(x, y)];
        a.coeffRef(variable_index) = count;
        constraints[variable_index] = true;
      }
      if (count != 0) {
        A.push_back(a);
        b.push_back(average_laplacian[index][channel]);
      }
    }
  }  
  
  // If no constraint, add value constraints.
  /*
  int no_constraint_count = 0;
  for (int v = 0; v < (int)indexes.size(); ++v) {
    if (constraints[v])
      continue;
    ++no_constraint_count;
    
    const int index = indexes[v][1] * width + indexes[v][0];
    if (values[index].empty()) {
      cerr << "Impossible here." << endl;
      exit (1);
    }
    double average = 0.0;
    for (const auto& value : values[index])
      average += value[channel];
    average /= values[index].size();
    
    Eigen::SparseVector<double> a(indexes.size());
    a.coeffRef(v) = 1.0;
    A.push_back(a);
    b.push_back(average);
  }
  */  

  vector<Eigen::Triplet<double> > triplets;
  for (int y = 0; y < A.size(); ++y) {
    for (SparseVector<double>::InnerIterator it(A[y]); it; ++it)
      triplets.push_back(Eigen::Triplet<double>(y, it.index(), it.value()));
  }
  
  Eigen::SparseMatrix<double> A2(A.size(), indexes.size());
  A2.setFromTriplets(triplets.begin(), triplets.end());
  
  Eigen::VectorXd b2(b.size());
  for (int i = 0; i < (int)b.size(); ++i)
    b2[i] = b[i];
  
  Eigen::SparseMatrix<double> ATA = A2.transpose() * A2;
  Eigen::VectorXd ATb = A2.transpose() * b2;
  
  ConjugateGradient<SparseMatrix<double> > cg;
  cg.setMaxIterations(synthesis_data.num_cg_iterations);
  cg.compute(ATA);
  cerr << "CG solve..." << flush;

  Eigen::VectorXd x0(indexes.size());
  for (int v = 0; v < indexes.size(); ++v)
    x0[v] = floor_texture->at<cv::Vec3b>(indexes[v][1], indexes[v][0])[channel];
        
  VectorXd x = cg.solveWithGuess(ATb, x0);
  cerr << "done. " << flush;
  for (int v = 0; v < (int)indexes.size(); ++v) {
    floor_texture->at<cv::Vec3b>(indexes[v][1], indexes[v][0])[channel] = 
      static_cast<unsigned char>(max(0.0, min(255.0, x[v])));
  }  
}    

void PoissonBlendSubNew(const SynthesisData& synthesis_data,
                        const vector<vector<Vector3d> >& laplacians,
                        const vector<vector<Vector3d> >& values,
                        const vector<Vector3d>& average_laplacian,
                        const vector<Vector2i>& indexes,
                        map<pair<int, int>, int>& inverse_indexes,
                        const int channel,
                        cv::Mat* floor_texture) {
  const int width      = synthesis_data.texture_size[0];
  const int height     = synthesis_data.texture_size[1];

  SparseMatrix<double> A(indexes.size(), indexes.size());
  VectorXd b(indexes.size());

  vector<Eigen::Triplet<double> > triplets;

  // Laplacian constraints.
  int laplacian_count = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = y * width + x;
      if (synthesis_data.mask[index])
        continue;

      const int ref_variable_index = inverse_indexes[pair<int, int>(x, y)];
      
      int count = 0;
      vector<Vector2i> neighbors;
      if (x != 0)
        neighbors.push_back(Vector2i(x - 1, y));
      if (x != width - 1)
        neighbors.push_back(Vector2i(x + 1, y));
      if (y != 0)
        neighbors.push_back(Vector2i(x, y - 1));
      if (y != height - 1)
        neighbors.push_back(Vector2i(x, y + 1));

      for (const auto neighbor : neighbors) {
        const int index = neighbor[1] * width + neighbor[0];
        if (synthesis_data.mask[index]) {
          if (inverse_indexes.find(pair<int, int>(neighbor[0], neighbor[1])) ==
              inverse_indexes.end()) {
            cout << "w1" << endl;
            exit (1);
          }
          const int variable_index = inverse_indexes[pair<int, int>(neighbor[0], neighbor[1])];
          triplets.push_back(Triplet<double>(ref_variable_index, variable_index, -1));
          ++count;
        }
      }
      if (count != 0) {
        if (inverse_indexes.find(pair<int, int>(x, y)) == inverse_indexes.end()){
          cout << "w5" << endl;
          exit (1);
        }
        triplets.push_back(Triplet<double>(ref_variable_index, ref_variable_index, count));
      }

      b[ref_variable_index] = average_laplacian[index][channel];      
    }
  }  
  
  A.setFromTriplets(triplets.begin(), triplets.end());

  // Data terms.
  const double kDataWeight = 2.0;
  for (int i = 0; i < indexes.size(); ++i) {
    const int x = indexes[i][0];
    const int y = indexes[i][1];
    const int index = y * width + x;
    if (values[index].size() != 1)
        continue;
    A.coeffRef(i, i) += kDataWeight;
    b[i] += kDataWeight * values[index][0][channel];
  }
  
  SimplicialCholesky<SparseMatrix<double> > chol(A);
  VectorXd x = chol.solve(b);

  for (int v = 0; v < (int)indexes.size(); ++v) {
    floor_texture->at<cv::Vec3b>(indexes[v][1], indexes[v][0])[channel] = 
      static_cast<unsigned char>(max(0.0, min(255.0, x[v])));
  }  
}    
  
void PoissonBlend(const SynthesisData& synthesis_data,
                  const vector<vector<Vector3d> >& laplacians,
                  const vector<vector<Vector3d> >& values,
                  cv::Mat* floor_texture) {
  cerr << "Poissonblending..." << flush;
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
    PoissonBlendSubNew(synthesis_data, laplacians, values, average_laplacian,
                       indexes, inverse_indexes, c, floor_texture);

  }
  cerr << " all done." << endl;
}

void InitializeTexture(const SynthesisData& synthesis_data, cv::Mat* floor_texture,
                       vector<bool>* pixel_guarded) {
  const cv::Vec3b kHole(0, 0, 0);
  const int kInvalid = -1;

  const int width      = synthesis_data.texture_size[0];
  const int height     = synthesis_data.texture_size[1];
  const vector<bool>& mask = synthesis_data.mask;

  const int guard_radius = synthesis_data.margin;
  vector<bool> texture_used(synthesis_data.projected_textures.size(), false);
  pixel_guarded->clear();
  pixel_guarded->resize(width * height, false);

  while (true) {
    int best_index = kInvalid;
    int best_count = 0;
    
    for (int i = 0; i < synthesis_data.projected_textures.size(); ++i) {
      if (texture_used[i])
        continue;
      const cv::Mat& image = synthesis_data.projected_textures[i];

      // Count the number of non-black pixels that are not used.
      int count = 0;
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const int index = y * width + x;
          if (mask[index] && image.at<cv::Vec3b>(y, x) != kHole && !pixel_guarded->at(index))
            ++count;
        }
      }
      if (best_index == kInvalid || count > best_count) {
        best_index = i;
        best_count = count;
      }
    }

    if (best_index == kInvalid || best_count == 0)
      break;

    //----------------------------------------------------------------------      
    // Copy reference to the output.
    //----------------------------------------------------------------------
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const cv::Vec3b color = synthesis_data.projected_textures[best_index].at<cv::Vec3b>(y, x);
        const int index = y * width + x;
        if (mask[index] && color != kHole && !pixel_guarded->at(index)) {
          floor_texture->at<cv::Vec3b>(y, x) = color;
        }
      }
    }

    texture_used[best_index] = true;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const cv::Vec3b color = synthesis_data.projected_textures[best_index].at<cv::Vec3b>(y, x);
        const int index = y * width + x;
        if (mask[index] && color != kHole) {
          for (int j = -guard_radius; j <= guard_radius; ++j) {
            const int ytmp = y + j;
            if (ytmp < 0 || height <= ytmp)
              continue;
            for (int i = -guard_radius; i <= guard_radius; ++i) {
              const int xtmp = x + i;
              if (xtmp < 0 || width <= xtmp)
                continue;
              pixel_guarded->at(ytmp * width + xtmp) = true;
            }
          }
        }
      }
    }
  }

  int index = 0;
  for (int y = 0; y < floor_texture->rows; ++y) {
    for (int x = 0; x < floor_texture->cols; ++x, ++index) {
      if (floor_texture->at<cv::Vec3b>(y, x) != kHole) {
        pixel_guarded->at(index) = false;
      }
    }
  }
  
  /*
  const cv::Vec3b kHole(0, 0, 0);
  const int kInvalid = -1;

  const int width      = synthesis_data.texture_size[0];
  const int height     = synthesis_data.texture_size[1];
  int best_index = kInvalid;
  int best_count = 0;
  const vector<bool>& mask = synthesis_data.mask;
  for (int i = 0; i < synthesis_data.projected_textures.size(); ++i) {
    const cv::Mat& image = synthesis_data.projected_textures[i];
    // Count the number of non-black pixels.
    int count = 0;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (mask[y * width + x] && image.at<cv::Vec3b>(y, x) != kHole)
          ++count;
      }
    }
    if (best_index == kInvalid || count > best_count) {
      best_index = i;
      best_count = count;
    }
  }

  //----------------------------------------------------------------------      
  // Copy reference to the output.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const cv::Vec3b color = synthesis_data.projected_textures[best_index].at<cv::Vec3b>(y, x);
      const int index = y * width + x;
      if (mask[index] && color != kHole) {
        floor_texture->at<cv::Vec3b>(y, x) = color;        
      }
    }
  }

  // Copy the others with some distance from the reference.
  for (int i = 0; i < synthesis_data.projected_textures.size(); ++i) {
    if (i == best_index)
      continue;

    int index = 0;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x, ++index) {
        if (!mask[index])
          continue;
        if (floor_texture->at<cv::Vec3b>(y, x) != kHole)
          continue;

        floor_texture->at<cv::Vec3b>(y, x) =
          synthesis_data.projected_textures[i].at<cv::Vec3b>(y, x);
      }
    }
  }
  */
}
  
}  // namespace

void CollectCandidatePatches(const SynthesisData& synthesis_data,
                             std::vector<cv::Mat>* patches,
                             std::vector<Eigen::Vector2i>* patch_positions) {
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
          patch_positions->push_back(Vector2i(x, y));
        }
      }
    }
  }
}

void SynthesizePoisson(const SynthesisData& synthesis_data,
                       const std::vector<cv::Mat>& patches,
                       const std::vector<Eigen::Vector2i>& patch_positions,
                       const bool vertical_constraint,
                       cv::Mat* floor_texture) {
  // First identify the projected texture with the most area.
  const cv::Vec3b kHole(0, 0, 0);
  // Pixels that are right around the initial floor_texture. "Value"
  // constraint should not be enforced here for smooth transition.
  vector<bool> pixel_guarded;
  InitializeTexture(synthesis_data, floor_texture, &pixel_guarded);

  // Simple case. Use one image to synthesize.
  cv::imshow("source", *floor_texture);
  // cv::imwrite("source.png", *floor_texture);
  // cv::waitKey(0);
  
  vector<bool> initial_mask(floor_texture->rows * floor_texture->cols, false);
  int index = 0;
  for (int y = 0; y < floor_texture->rows; ++y) {
    for (int x = 0; x < floor_texture->cols; ++x, ++index) {
      if (floor_texture->at<cv::Vec3b>(y, x) != kHole) {
        initial_mask[index] = true;
      }
    }
  }
  
  const double kMarginResidualScale = 1.25;
  const int width      = synthesis_data.texture_size[0];
  const int height     = synthesis_data.texture_size[1];
  const int patch_size = synthesis_data.patch_size;
  const int margin     = synthesis_data.margin;
  // Try patch synthesis at grid points
  // i * (patch_size - margin), j * (patch_size - margin).
  const int grid_width  = (width - patch_size - 1) / (patch_size - margin) + 1;
  const int grid_height = (height - patch_size - 1) / (patch_size - margin) + 1;

  // Constraints for poisson blending later.
  vector<vector<Vector3d> > laplacians(width * height);
  vector<vector<Vector3d> > values(width * height);

  cerr << "stitch " << flush;
  set<pair<int, int> > visited_grids;
  while (true) {
    // Find a grid position with the most constraints.
    Vector2i best_grid;
    if (!FindGridWithMostValid(synthesis_data, *floor_texture, visited_grids, &best_grid))
      break;
    visited_grids.insert(pair<int, int>(best_grid[0], best_grid[1]));
    
    // Put a texture at best_grid from candidates.
    const int min_x = best_grid[0] * (patch_size - margin);
    const int max_x = min(width, min_x + patch_size);
    const int min_y = best_grid[1] * (patch_size - margin);
    const int max_y = min(height, min_y + patch_size);
    const Vector2i x_range(min_x, max_x);
    const Vector2i y_range(min_y, max_y);

    int num_holes = 0;
    {
      for (int y = min_y; y < max_y; ++y) {
        for (int x = min_x; x < max_x; ++x) {
          const int index = y * width + x;
          if (!synthesis_data.mask[index])
            continue;
          if (floor_texture->at<cv::Vec3b>(y, x) == kHole)
            ++num_holes;
        }
      }
    }

    cv::Mat patch_with_initial_texture(patch_size, patch_size, CV_8UC3, cv::Scalar(0));
    if (num_holes == 0) {
      for (int y = min_y; y < max_y; ++y) {
        for (int x = min_x; x < max_x; ++x) {
          const int index = y * width + x;
          patch_with_initial_texture.at<cv::Vec3b>(y - min_y, x - min_x) =
            floor_texture->at<cv::Vec3b>(y, x);
        }
      }
    } else {
      vector<double> residuals(patches.size(), 0);
      const double kLarge = 10000.0;
      double current_min = kLarge;
      vector<int> candidates;
      if (vertical_constraint) {
        // First search along vertical.
        bool found = false;
        for (int p = 0; p < patch_positions.size(); ++p) {
          if (patch_positions[p][0] == min_x) {
            residuals[p] = AverageAbsoluteDifference(*floor_texture, patches[p], x_range, y_range,
                                                     current_min * kMarginResidualScale);
            current_min = min(current_min, residuals[p]);
            found = true;
          } else {
            residuals[p] = kLarge;
          }
        }
        if (!found) {
          for (int p = 0; p < patches.size(); ++p) {
            residuals[p] = AverageAbsoluteDifference(*floor_texture, patches[p], x_range, y_range,
                                                     current_min * kMarginResidualScale);
            current_min = min(current_min, residuals[p]);
          }
        }          
        const double min_residual = *min_element(residuals.begin(), residuals.end());
        const double threshold = min_residual * kMarginResidualScale;
        for (int i = 0; i < (int)residuals.size(); ++i) {
          if (residuals[i] <= threshold)
            candidates.push_back(i);
        }
      } else {
        for (int p = 0; p < patches.size(); ++p) {
          residuals[p] = AverageAbsoluteDifference(*floor_texture, patches[p], x_range, y_range,
                                                   current_min * kMarginResidualScale);
          current_min = min(current_min, residuals[p]);
        }
        
        const double min_residual = *min_element(residuals.begin(), residuals.end());
        const double threshold = min_residual * kMarginResidualScale;
        for (int i = 0; i < (int)residuals.size(); ++i) {
          if (residuals[i] <= threshold)
            candidates.push_back(i);
        }
      }
      // cerr << "Candidate: " << (int)candidates.size() << '/' << residuals.size() << endl;
      // << min_residual << ' ' << threshold;
      const int patch_id = candidates[rand() % candidates.size()];
      // cerr << "  patch: " << patch_id << endl;
      patch_with_initial_texture = patches[patch_id];
      // overwrite with floor_texture and initial_mask.
      for (int y = y_range[0]; y < y_range[1]; ++y) {
        for (int x = x_range[0]; x < x_range[1]; ++x) {
          if (synthesis_data.mask[y * width + x] && initial_mask[y * width + x])
            patch_with_initial_texture.at<cv::Vec3b>(y - y_range[0], x - x_range[0]) =
              floor_texture->at<cv::Vec3b>(y, x);
        }
      }
      CopyPatch(synthesis_data.mask, patch_with_initial_texture, x_range, y_range, floor_texture);

      cv::imshow("next", *floor_texture);
      //cv::waitKey(0);
    }
    // cout << "setdataforblending" << endl;
    SetDataForBlending(width,
                       synthesis_data.mask,
                       initial_mask,
                       pixel_guarded,
                       patch_with_initial_texture,
                       x_range,
                       y_range,
                       &laplacians,
                       &values);
    // cout << "done" << endl;
  }
  // cerr << "blend" << endl;
  // Poisson blend.
  cv::imshow("before", *floor_texture);
  
  cerr << "blend" << flush;
  PoissonBlend(synthesis_data, laplacians, values, floor_texture);
  // cerr << "done" << endl;
}

  /*
void SynthesizeQuilt(const SynthesisData& synthesis_data,
                     const std::vector<cv::Mat>& patches,
                     cv::Mat* floor_texture) {
  // First identify the projected texture with the most area.
  const cv::Vec3b kHole(0, 0, 0);
  InitializeTexture(synthesis_data, floor_texture);

  // Simple case. Use one image to synthesize.
  cv::imshow("source", *floor_texture);
  // cv::waitKey(0);
  
  const double kMarginResidual = 2.5;
  const int width      = synthesis_data.texture_size[0];
  const int height     = synthesis_data.texture_size[1];
  const int patch_size = synthesis_data.patch_size;
  const int margin     = synthesis_data.margin;
  // Try patch synthesis at grid points
  // i * (patch_size - margin), j * (patch_size - margin).
  const int grid_width  = (width - patch_size - 1) / (patch_size - margin) + 1;
  const int grid_height = (height - patch_size - 1) / (patch_size - margin) + 1;

  // Constraints for poisson blending later.
  vector<vector<Vector3d> > laplacians(width * height);
  vector<vector<Vector3d> > values(width * height);

  set<pair<int, int> > visited_grids;
  while (true) {
    // Find a grid position with the most constraints.p
    Vector2i best_grid;
    if (!FindGridWithMostValid(synthesis_data, *floor_texture, visited_grids, &best_grid))
      break;
    visited_grids.insert(pair<int, int>(best_grid[0], best_grid[1]));
    // cerr << "best grid " << best_grid[0] << ' ' << best_grid[1] << endl;

    // Put a texture at best_grid from candidates.
    const int min_x = best_grid[0] * (patch_size - margin);
    const int max_x = min(width, min_x + patch_size);
    const int min_y = best_grid[1] * (patch_size - margin);
    const int max_y = min(height, min_y + patch_size);
    const Vector2i x_range(min_x, max_x);
    const Vector2i y_range(min_y, max_y);

    vector<double> residuals(patches.size(), 0);
    const double kLarge = 10000.0;
    double current_min = kLarge;
    for (int p = 0; p < patches.size(); ++p) {
      residuals[p] = AverageAbsoluteDifference(*floor_texture, patches[p], x_range, y_range,
                                               current_min + kMarginResidual);
      current_min = min(current_min, residuals[p]);
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

    // cout << "setdataforblending" << endl;
    SetDataForBlending(width,
                       synthesis_data.mask,
                       patches[patch_id],
                       x_range,
                       y_range,
                       &laplacians,
                       &values);
    // cout << "done" << endl;
  }
  // cerr << "blend" << endl;
  // Poisson blend.
  cv::imshow("before", *floor_texture);

  PoissonBlend(synthesis_data, laplacians, values, floor_texture);
  // cerr << "done" << endl;
}
  */  

}  // namespace structured_indoor_modeling
