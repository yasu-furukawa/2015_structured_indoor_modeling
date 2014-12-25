#include "door_detection.h"

#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <Eigen/Dense>

#include "data.h"
#include "evidence.h"
#include "../base/imageProcess/morphological_operation.h"

using namespace Eigen;
using namespace floored;
using namespace std;
using namespace room_segmentation;

namespace {
const float kInvalidScore = numeric_limits<float>::max();


// elegant-prize
const int kSeedStep = 10;
const int kMaxDistanceRadius = 10;
const double kDoorDetectionScale = 0.01;
  const float kGoodFreeSpaceEvidence = 30; // 50;//100.0;
const float kBoundarySubsampleRatio = 0.2;

const int kClusteringSubsample = 4;
const float kMarginFromBoundaryForVisibility = 5;
const int kVisibilityMargin = 10;

const int kInitialClusterNum = 20;

const float kMergeThreshold = 0.3;
 

struct ShortestPathNode {
  float current_score;
  pair<int, int> previous_node;
};

void ForegroundPath(const vector<bool>& mask,
                    const vector<float>& distance_to_boundary,
                      const int width,
                    const int height,
                    const pair<int, int>& seed,
                    vector<ShortestPathNode>* nodes) {
  // Start from the seed.
  priority_queue<pair<float, pair<int, int> > > water_front;
  water_front.push(make_pair(-0.0, seed));
  const int start_index = seed.second * width + seed.first;
  nodes->at(start_index).current_score = 0.0;

  while (!water_front.empty()) {
    const auto node = water_front.top();
    water_front.pop();
    const int x = node.second.first;
    const int y = node.second.second;
    // Propagate to neighboring pixels within a mask. mask is 0 at the boundary.
    for (int j = -1; j <= 1; ++j) {
      const int ytmp = y + j;
      for (int i = -1; i <= 1; ++i) {
        const int xtmp = x + i;
        if (i == 0 && j == 0)
          continue;
        const int new_index = ytmp * width + xtmp;
        if (!mask[new_index])
          continue;

        const float weight = distance_to_boundary[y * width + x];
        
        const float new_score = (-node.first) + weight * sqrt(i * i + j * j);
        if (new_score < nodes->at(new_index).current_score) {
          nodes->at(new_index).current_score = new_score;
          nodes->at(new_index).previous_node = node.second;
          water_front.push(make_pair(-new_score, make_pair(xtmp, ytmp)));
        }
      }
    }
  }
}

void TraceBack(const vector<ShortestPathNode>& nodes,
               const int width,
               const pair<int, int>& seed,
               const pair<int, int>& target,
               vector<pair<int, int> >* path) {
  const int index = target.second * width + target.first;
  if (nodes[index].current_score == kInvalidScore)
    return;
  
  pair<int, int> pixel = target;
  while (pixel != seed) {
    path->push_back(pixel);
    const int index = pixel.second * width + pixel.first;
    pixel = nodes[index].previous_node;
  }  
}
  
void FindShortestPaths(const vector<bool>& mask,
                       const vector<float>& distance_to_boundary,
                       const int width,
                       const int height,
                       const pair<int, int>& seed,
                       const vector<pair<int, int> >& seeds,
                       vector<float>* path_counts) {
  path_counts->clear();
  path_counts->resize(width * height, 0.0);

  vector<ShortestPathNode> nodes(width * height);
  for (auto& node : nodes) {
    node.current_score = kInvalidScore;
    node.previous_node = make_pair(-1, -1);
  }

  // Foreground.
  ForegroundPath(mask, distance_to_boundary, width, height, seed, &nodes);

  // Trace back from other seeds.
  for (const auto& target : seeds) {
    vector<pair<int, int> > path;
    TraceBack(nodes, width, seed, target, &path);

    for (const auto& pixel : path) {
      const int index = pixel.second * width + pixel.first;
      path_counts->at(index) += 1;
    }
  }

  /*
  ofstream ofstr;
  ofstr.open("sp.ppm");
  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;
  for (const auto& node : nodes) {
    if (node.current_score == numeric_limits<float>::max())
      ofstr << "255 0 255 ";
    else {
      const int itmp = min(255, static_cast<int>(node.current_score / 3.0));
      ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
    }
  }
  ofstr.close();
  exit (1);
  */
}

void BlurField(const int width,
               const int height,
               const vector<bool>& mask,
               const double sigma,
               vector<float>* field) {
  const double sigma2 = 2 * sigma * sigma;
  const int half_size = static_cast<int>(ceil(2 * sigma));
  const int size = 2 * half_size + 1;
  vector<float> kernel(size * size);
  for (int y = -half_size; y <= half_size; ++y) {
    for (int x = -half_size; x <= half_size; ++x) {
      kernel[(y + half_size) * size + (x + half_size)] =
        exp(- (x * x + y * y) / sigma2);
    }
  }

  vector<float> old_field = *field;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = y * width + x;
      if (!mask[index])
        continue;

      float numer = 0.0;
      float denom = 0.0;
      for (int j = -half_size; j <= half_size; ++j) {
        const int ytmp = y + j;
        if (ytmp < 0 || height <= ytmp)
          continue;
        for (int i = -half_size; i <= half_size; ++i) {
          const int xtmp = x + i;
          if (xtmp < 0 || width <= xtmp)
            continue;
          const int index_tmp = ytmp * width + xtmp;
          if (!mask[index_tmp])
            continue;
          const int kernel_index = (j + half_size) * size + (i + half_size);
          numer += old_field[index_tmp] * kernel[kernel_index];
          denom += kernel[kernel_index];
        }
      }
      if (denom == 0.0) {
        cerr << "Impossible." << endl;
        exit (1);
      }
      field->at(index) = numer / denom;
    }
  }  
}  

void DrawDoorDetection(const int width,
                       const int height,
                       const vector<bool>& mask,
                       const vector<float>& door_detection,
                       const std::string& directory) {
  vector<unsigned char> door_detection_int;
  ConvertEvidence(width,
                  height,
                  door_detection,
                  kDoorDetectionScale,
                  &door_detection_int);

  ofstream ofstr;
  ofstr.open((directory + "door_detection.ppm").c_str());
  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;
  for (const auto& intensity : door_detection_int) {
    ofstr << static_cast<int>(intensity) << ' ';
  }
  ofstr.close();
}

void SetMask(const Frame& frame,
             const vector<float>& free_space_evidence,
             vector<bool>* mask) {
  mask->clear();
  mask->resize(free_space_evidence.size(), false);

  for (int y = 0; y < frame.size[1]; ++y) {
    for (int x = 0; x < frame.size[0]; ++x) {
      if (y == 0 || y == frame.size[1] - 1 || x == 0 || x == frame.size[0] - 1)
        continue;
      const int index = y * frame.size[0] + x;
      if (free_space_evidence[index] > kGoodFreeSpaceEvidence)
        mask->at(index) = true;
    }
  }
}

void SetDistanceToBoundary(const Frame& frame,
                           const vector<bool>& mask,
                           vector<float>* distance_to_boundary) {
  const int width  = frame.size[0];
  const int height = frame.size[1];
  distance_to_boundary->clear();
  distance_to_boundary->resize(width * height, numeric_limits<float>::max());
  priority_queue<pair<float, pair<int, int> > > water_front;
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      const int index = y * width + x;
      if (mask[index] == true &&
          (mask[index - 1] == false ||
           mask[index + 1] == false ||
           mask[index - width] == false ||
           mask[index + width] == false)) {
        distance_to_boundary->at(index) = 0.0;
        water_front.push(make_pair(0.0, make_pair(x, y)));
      }
    }
  }

  while (!water_front.empty()) {
    const auto node = water_front.top();
    water_front.pop();

    const float current_score = -node.first;
    const int x = node.second.first;
    const int y = node.second.second;

    for (int j = -1; j <= 1; ++j) {
      const int ytmp = y + j;
      for (int i = -1; i <= 1; ++i) {
        const int xtmp = x + i;
        if (i == 0 && j == 0)
          continue;
        const int new_index = ytmp * width + xtmp;
        if (!mask[new_index])
          continue;
        
        const float new_score = current_score + sqrt(i * i + j * j);
        if (new_score < distance_to_boundary->at(new_index)) {
          distance_to_boundary->at(new_index) = new_score;
          water_front.push(make_pair(-new_score, make_pair(xtmp, ytmp)));
        }
      }
    }
  }
  /*
  ofstream ofstr;
  ofstr.open("bb.ppm");
  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;

  for (int i = 0; i < distance_to_boundary->size(); ++i) {
    if (mask[i]) {
      const int itmp = min(255, static_cast<int>(3 * distance_to_boundary->at(i)));
      ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
    } else
      ofstr << "255 0 255 ";
  }
  
  ofstr.close();
  exit (1);
  */

  /*
  // Take the maximum distance within a radius, then set the weight.
  vector<float> vftmp = *distance_to_boundary;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      float maximum_distance = 0.0;
      const int index = y * width + x;
      if (!mask[index])
        continue;

      for (int j = -kMaxDistanceRadius; j <= kMaxDistanceRadius; ++j) {
        const int ytmp = y + j;
        if (ytmp < 0 || height <= ytmp)
          continue;
        for (int i = -kMaxDistanceRadius; i <= kMaxDistanceRadius; ++i) {
          const int xtmp = x + i;
          if (xtmp < 0 || width <= xtmp)
            continue;

          maximum_distance = max(maximum_distance,
                                 vftmp[ytmp * width + xtmp]);
        }
      }
      const float sigma = 0.5 * maximum_distance;
      distance_to_boundary->at(index) =
        exp(- vftmp[index] * vftmp[index] / (2 * sigma * sigma));
    }
  }

  ofstream ofstr;
  ofstr.open("weight.ppm");
  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;

  for (int i = 0; i < distance_to_boundary->size(); ++i) {
    if (mask[i]) {
      const int itmp = min(255, static_cast<int>(255 * distance_to_boundary->at(i)));
      ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
    } else
      ofstr << "255 0 255 ";
  }
  
  ofstr.close();
  */
}

int CountMask(const vector<bool>& mask) {
  int count = 0;
  for (const auto& bit : mask)
    if (bit)
      ++count;
  return count;
}

void WriteMask(const int width, const int height,
	       const vector<bool>& mask, const string& filename) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P2" << endl
	<< width << ' ' << height << endl
	<< 255 << endl;
  for (const auto& value : mask)
    if (value)
      ofstr << "0 ";
    else
      ofstr << "255 ";
  ofstr.close();
}

void FindBoundary(const int width, 
		  const int height,
		  const vector<bool>& mask,
		  vector<pair<int, int> >* boundary) {
  boundary->clear();
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      const int index = y * width + x;
      if (!mask[index])
	continue;
      if ((!mask[index - 1]) ||
	  (!mask[index + 1]) ||
	  (!mask[index - width]) ||
	  (!mask[index + width])) {
	boundary->push_back(make_pair(x, y));
      }
    }
  }
}

bool IsVisible(const int width,
	       const int height,
	       const vector<bool>& mask,
	       const pair<int, int>& source,
	       const pair<int, int>& target) {
  // Skip first kMargin pixels to avoid noisy boundary.
  Vector2f step =
    Vector2f(target.first, target.second) - Vector2f(source.first, source.second);
  const int num_steps = static_cast<int>(floor(step.norm())) * 2 + 1;
  step /= num_steps;

  const Vector2f start(source.first, source.second);
  for (int i = kVisibilityMargin; i < num_steps; ++i) {
    const Vector2f position = start + i * step;
    const int x = static_cast<int>(round(position[0]));
    const int y = static_cast<int>(round(position[1]));
    if (x < 0 || width <= x || y < 0 || height <= y) {
      cerr << "Impossible: " << x << ' ' << y << ' ' << width << ' ' << height << endl;
      exit (1);
    }

    if (!mask[y * width + x])
      return false;
  }
  return true;
}

void ComputeVisibility(const int width,
		       const int height,
		       const int subsample,
		       const vector<pair<int, int> >& boundary,
		       const vector<bool>& mask,
		       const vector<float>& distance_to_boundary,
		       vector<vector<int> >* visibility) {  
  const int subsampled_width = width / subsample;
  const int subsampled_height = height / subsample;
  visibility->clear();
  visibility->resize(subsampled_width * subsampled_height);

  int visibility_index = 0;
  for (int subsampled_y = 0; subsampled_y < subsampled_height; ++subsampled_y) {
    const int y = subsampled_y * subsample;
    for (int subsampled_x = 0; subsampled_x < subsampled_width; ++subsampled_x, ++visibility_index) {
      const int x = subsampled_x * subsample;

      // Compute visibility only for a pixel inside a mask.
      if (!mask[y * width + x]) {
	continue;
      }
      if (distance_to_boundary[y * width + x] < kMarginFromBoundaryForVisibility) {
	continue;
      }

      // Test against all the boundary points.
      for (int b = 0; b < boundary.size(); ++b) {        
	if (IsVisible(width, height, mask, make_pair(x, y), boundary[b])) {
	  visibility->at(visibility_index).push_back(b);
        }
      }

      /*
      ofstream ofstr;
      ofstr.open("vis.svg");
      ofstr << "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http//www.w3.org/1999/xlink\">" << endl
	    << "<rect x=\"0\" y=\"0\" width=\"" << width
	    << "\" height=\"" << height << "\" style=\"fill: #FFFFFF\"/>" << endl;
      ofstr << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"2\" stroke=\"black\" stroke-width=\"1\" fill=\"red\" />" << endl;
      for (const auto& b : visibility->at(visibility_index)) {
	ofstr << "<circle cx=\"" << boundary[b].first << "\" cy=\"" << boundary[b].second << "\" r=\"2\" stroke=\"green\" stroke-width=\"1\" fill=\"cyan\" />" << endl;
      }
      ofstr << "</svg>" << endl;
      ofstr.close();

      cout << "first" << endl;
      char a;
      cin >> a;
      */
    }
  }
}

void AssociateWeightToVisibility(const int width,
                                 const int height,
                                 const int subsample,
                                 const vector<pair<int, int> >& boundary,
                                 const vector<vector<int> >& visibility,
                                 vector<vector<pair<int, float> > >* weighted_visibility) {
  // Weight is inversely proportional to the distance.
  weighted_visibility->clear();
  weighted_visibility->resize(visibility.size());

  const int subsampled_width = width / subsample;
  const int subsampled_height = height / subsample;

  int visibility_index = 0;
  for (int subsampled_y = 0; subsampled_y < subsampled_height; ++subsampled_y) {
    const int y = subsampled_y * subsample;
    for (int subsampled_x = 0; subsampled_x < subsampled_width; ++subsampled_x, ++visibility_index) {
      const int x = subsampled_x * subsample;

      if (visibility[visibility_index].empty())
        continue;

      const vector<int>& vis = visibility[visibility_index];
      weighted_visibility->at(visibility_index).resize(vis.size());

      for (int i = 0; i < vis.size(); ++i) {
        const Vector2f pos0(x, y);
        const Vector2f pos1(boundary[vis[i]].first, boundary[vis[i]].second);
        const float distance = (pos0 - pos1).norm();
        weighted_visibility->at(visibility_index)[i] = make_pair(vis[i], distance);
      }
      float weight_sum = 0.0;
      for (int i = 0; i < vis.size(); ++i) {
        const float weight =
          1.0 / (weighted_visibility->at(visibility_index)[i].second + 1.0);
        weighted_visibility->at(visibility_index)[i].second = weight;
        weight_sum += weight;
      }
      for (int i = 0; i < vis.size(); ++i) {
        weighted_visibility->at(visibility_index)[i].second /= weight_sum;
      }
    }
  }     
}

int IdentifyClosestCenterIndex(const vector<vector<pair<int, float> > >& weighted_visibility,
                               const int index,
                               const vector<int>& centers) {
  const int kInvalid = -1;
  int closest_index = kInvalid;
  float closest_distance = numeric_limits<float>::max();
  for (int i = 0; i < centers.size(); ++i) {
    const float distance = VisibilityDistance(weighted_visibility[index],
                                              weighted_visibility[centers[i]]);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }
  if (closest_index == kInvalid) {
    cerr << "Impossible." << endl;
    exit (1);
  }
  return closest_index;
}

void UpdateCenters(const int width,
                   const int height,
                   const int subsample,
                   const vector<vector<pair<int, float> > >& weighted_visibility,
                   const vector<vector<int> >& clusters,
                   vector<int>* centers) {
  /* k-medoid
  centers->clear();
  centers->resize(clusters.size());
  for (int c = 0; c < clusters.size(); ++c) {
    const vector<int>& cluster = clusters[c];
    if (cluster.empty())
      continue;
    // Sum of squared distances to the other members.
    vector<float> distances(cluster.size(), 0.0);
    for (int i = 0; i < cluster.size(); ++i) {
      for (int j = i + 1; j < cluster.size(); ++j) {
        const float distance = VisibilityDistance(weighted_visibility[cluster[i]],
                                                  weighted_visibility[cluster[j]]);
        distances[i] += distance * distance;
        distances[j] += distance * distance;
      }
    }
    // Find the smallest one.
    const int min_index =
      min_element(distances.begin(), distances.end()) - distances.begin();
    centers->at(c) = cluster[min_index];
  }
  */

  const int subsampled_width = width / subsample;
  const int subsampled_height = height / subsample;
  
  centers->clear();
  centers->resize(clusters.size());
  for (int c = 0; c < clusters.size(); ++c) {
    const vector<int>& cluster = clusters[c];
    if (cluster.empty())
      continue;
    Vector2d subsampled_sum(0, 0);
    for (int i = 0; i < cluster.size(); ++i) {
      const int subsampled_x = cluster[i] % subsampled_width;
      const int subsampled_y = cluster[i] / subsampled_width;
      subsampled_sum[0] += subsampled_x;
      subsampled_sum[1] += subsampled_y;
    }
    subsampled_sum /= cluster.size();

    // Find the closest one.
    int best_index = -1;
    double best_distance = 0.0;
    for (int i = 0; i < cluster.size(); ++i) {
      const int subsampled_x = cluster[i] % subsampled_width;
      const int subsampled_y = cluster[i] / subsampled_width;
      const double distance =
        (subsampled_x - subsampled_sum[0]) * (subsampled_x - subsampled_sum[0]) +
        (subsampled_y - subsampled_sum[1]) * (subsampled_y - subsampled_sum[1]);
      if (best_index == -1 || distance < best_distance) {
        best_index = cluster[i];
        best_distance = distance;
      }
    }
    centers->at(c) = best_index;
  }
}
 
void DrawCluster(const int width,
                 const int height,
                 const int subsample,
                 const string& filename,
                 const vector<int>& centers,
                 const vector<vector<int> >& clusters) {
  vector<Vector3i> rgbs;
  rgbs.resize(width * height, Vector3i(255, 255, 255));

  const int margin = subsample / 2;

  const int subsampled_width = width / subsample;
  const int subsampled_height = height / subsample;

  for (int i = 0; i < clusters.size(); ++i) {
    const Vector3i rgb(rand() % 255, rand() % 255, rand() % 255);
    for (const auto& index : clusters[i]) {
      const int subsampled_x = index % subsampled_width;
      const int subsampled_y = index / subsampled_width;
      const int x = subsample * subsampled_x;
      const int y = subsample * subsampled_y;

      for (int j = -margin; j <= margin; ++j) {
        const int ytmp = y + j;
        if (ytmp < 0 || height <= ytmp)
          continue;
        for (int i = -margin; i <= margin; ++i) {
          const int xtmp = x + i;
          if (xtmp < 0 || width <= xtmp)
            continue;
          rgbs[ytmp * width + xtmp] = rgb;
        }
      }
    }

    {
      const int subsampled_x = centers[i] % subsampled_width;
      const int subsampled_y = centers[i] / subsampled_width;
      const int x = subsample * subsampled_x;
      const int y = subsample * subsampled_y;

      const int kCenterSize = 2;
      for (int j = -kCenterSize; j <= kCenterSize; ++j) {
        const int ytmp = y + j;
        if (ytmp < 0 || height <= ytmp)
          continue;
        for (int i = -kCenterSize; i <= kCenterSize; ++i) {
          const int xtmp = x + i;
          if (xtmp < 0 || width <= xtmp)
            continue;
          rgbs[ytmp * width + xtmp] = Vector3i(255, 0, 0);
        }
      }
    }
  }
  
  
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;
  for (const auto& rgb : rgbs) {
    ofstr << rgb[0] << ' ' << rgb[1] << ' ' << rgb[2] << ' ';
  }
  ofstr.close();
}

void Cluster(const vector<pair<int, int> >& boundary_array,
             const int width,
             const int height,
             const int subsample,
             const vector<vector<pair<int, float> > >& weighted_visibility,
             vector<int>* centers,
             vector<vector<int> >* clusters) {
  const int subsampled_width = width / subsample;
  const int subsampled_height = height / subsample;
  if (clusters->empty()) {
    // K-means clustering.
    clusters->clear();
    clusters->resize(centers->size());

    // Typically converges after 10 iterations.
    const int kTimes = 10;
    for (int t = 0; t < kTimes; ++t) {
      // Based on the current centers, associate elements to centers.
      clusters->clear();
      clusters->resize(centers->size());
      for (int i = 0; i < weighted_visibility.size(); ++i) {
        if (weighted_visibility[i].empty())
          continue;
        
        clusters->at(IdentifyClosestCenterIndex(weighted_visibility, i, *centers)).push_back(i);
      }
      // Update centers based on the assignments.
      UpdateCenters(width, height, subsample, weighted_visibility, *clusters, centers);
    }
    return;
  }
  cerr << "Real routine." << endl;

  // Based on the current centers, associate elements to centers.
  // From boundary.
  map<int, map<int, double> > boundary_to_pixel;
  {
    for (int p = 0; p < weighted_visibility.size(); ++p) {
      for (int i = 0; i < weighted_visibility[p].size(); ++i) {
        const int boundary = weighted_visibility[p][i].first;
        const double weight = weighted_visibility[p][i].second;
        boundary_to_pixel[boundary][p] += weight;
      }
    }
  }

  static int count = 0;
  
  // Typically converges after 10 iterations.
  const int kTimes = 5; // ???10;
  for (int t = 0; t < kTimes; ++t) {
    {    
      char buffer[1024];
      sprintf(buffer, "cluster_%02d.ppm", count++);
      DrawCluster(width, height, subsample,
                  buffer, *centers, *clusters);
    }

    cerr << "T " << t << ' ' << kTimes << endl;
    const vector<vector<int> > clusters_org = *clusters;
    map<int, int> pixel_to_cluster;
    {
      for (int c = 0; c < clusters_org.size(); ++c) {
        for (int i = 0; i < clusters_org[c].size(); ++i) {
          pixel_to_cluster[clusters_org[c][i]] = c;
        }
      }
    }
    
    clusters->clear();
    clusters->resize(centers->size());
    for (int p = 0; p < weighted_visibility.size(); ++p) {
      if (p % 100 == 0)
        cerr << p << ' ' << flush;
      if (weighted_visibility[p].empty())
        continue;
      
      // clusters->at(IdentifyClosestCenterIndex(weighted_visibility, i, *centers)).push_back(i);
      // Find the best cluster.

      // Angle sampling.
      const int kNumAngleSamples = 60;
      vector<pair<int, double> > angle_to_cluster_weight(kNumAngleSamples, make_pair(-1, 0));

      const int subsampled_x = p % subsampled_width;
      const int subsampled_y = p / subsampled_width;
      const int x = subsampled_x * subsample;
      const int y = subsampled_y * subsample;


      bool flag = false;
      /*
      if (count > 2 && abs(162 - x) <= subsample && abs(95 - y) <= subsample) {
        flag = true;
        cout << "flag true " << p << endl;
      }
      */
      
      for (int i = 0; i < weighted_visibility[p].size(); ++i) {
        const int boundary = weighted_visibility[p][i].first;

        if (flag) {
          cout << "Boundary: " << boundary_array[boundary].first << ' ' << boundary_array[boundary].second << ' ';
        }

        if (boundary_to_pixel.find(boundary) == boundary_to_pixel.end())
          continue;

        const int bx = boundary_array[boundary].first;
        const int by = boundary_array[boundary].second;

        double angle_d = atan2(by - y, bx - x) / M_PI * 180;
        if (angle_d < 0.0)
          angle_d += 2 * M_PI;
        const int angle_i = max(0, min(kNumAngleSamples - 1,
                                       (int)(angle_d / 360.0 * kNumAngleSamples)));

        // Get the best pixel_weight.
        int best_cluster = -1;
        double best_weight = 0.0;
        int best_pixel = -1;
        for (const auto& item : boundary_to_pixel[boundary]) {
          const int pixel = item.first;
          const double weight = item.second;
          const int cluster = pixel_to_cluster[pixel];
          
          if (weight > best_weight) {
            best_weight = weight;
            best_cluster = cluster;
            best_pixel = pixel;
          }
        }
        
        if (flag) {
          cout << "P " << best_pixel % subsampled_width * subsample << ' ' << best_pixel / subsampled_width * subsample << ' ' << best_cluster << ' ' << best_weight << endl;
        }
        
        if (best_weight > angle_to_cluster_weight[angle_i].second) {
          angle_to_cluster_weight[angle_i].second = best_weight;
          angle_to_cluster_weight[angle_i].first= best_cluster;
        }
      }

      if (flag) {
        for (int i = 0; i < angle_to_cluster_weight.size(); ++i)
          if (angle_to_cluster_weight[i].first != -1)
            cout << i * 2 << ' ' << angle_to_cluster_weight[i].first << ' '
                 << angle_to_cluster_weight[i].second << endl;
      }
      

      map<int, double> cluster_weight;
      for (const auto& item : angle_to_cluster_weight) {
        const int cluster = item.first;
        if (cluster != -1) {
          cluster_weight[cluster] += item.second;
        }
      }
      
      int best_cluster = -1;
      double best_weight = 0.0;
      for (auto& item : cluster_weight) {
        if (best_cluster == -1 || item.second > best_weight) {
          best_weight = item.second;
          best_cluster = item.first;
        }
      }
      
      clusters->at(best_cluster).push_back(p);
    }
  }
  // Update centers based on the assignments.
  UpdateCenters(width, height, subsample, weighted_visibility, *clusters, centers);
}

bool Merge(const int width,
           const int height,
           const int subsample,
           const vector<vector<pair<int, float> > >& weighted_visibility,
           const float kMergeThreshold,
           vector<int>* centers,
           vector<vector<int> >* clusters) {
  // Distance matrix.
  vector<vector<float> > distances(centers->size());
  for (int i = 0; i < distances.size(); ++i)
    distances[i].resize(centers->size(), numeric_limits<float>::max());

  cout << "merge" << endl;
  for (int i = 0; i < centers->size(); ++i) {
    for (int j = i + 1; j < centers->size(); ++j) {
      distances[i][j] = VisibilityDistance(weighted_visibility[centers->at(i)],
                                           weighted_visibility[centers->at(j)]);
      cout << distances[i][j] << ' ';
    }
    cout << endl;
  }

  vector<pair<int, int> > pairs_to_merge;
  while (true) {
    // Find the pair with the closest distance.
    pair<int, int> closest_pair;
    float closest_distance = numeric_limits<float>::max();
    for (int i = 0; i < centers->size(); ++i) {
      for (int j = i + 1; j < centers->size(); ++j) {
        if (distances[i][j] < closest_distance) {
          closest_distance = distances[i][j];
          closest_pair = make_pair(i, j);
        }
      }
    }

    if (closest_distance < kMergeThreshold) {
      pairs_to_merge.push_back(closest_pair);
      // Avoid merging these 2 clusters any more.
      for (int i = 0; i < centers->size(); ++i) {
        distances[closest_pair.first][i]  = numeric_limits<float>::max();
        distances[closest_pair.second][i] = numeric_limits<float>::max();
        distances[i][closest_pair.first]  = numeric_limits<float>::max();
        distances[i][closest_pair.second] = numeric_limits<float>::max();
      }
    } else {
      break;
    }
  }

  if (pairs_to_merge.empty()) {
    return false;
  }

  // Update clusters.
  vector<int> erase_ids;
  for (const auto& pair : pairs_to_merge) {
    vector<int>& cluster0 = clusters->at(pair.first);
    const vector<int>& cluster1 = clusters->at(pair.second);
    cluster0.insert(cluster0.end(), cluster1.begin(), cluster1.end());
    erase_ids.push_back(pair.second);
  }

  sort(erase_ids.rbegin(), erase_ids.rend());
  for (int i = 0; i < erase_ids.size(); ++i)
    clusters->erase(clusters->begin() + erase_ids[i]);
  
  // Update centers.
  UpdateCenters(width, height, subsample, weighted_visibility, *clusters, centers);

  return true;
}
 
void ClusterMerge(const vector<pair<int, int> >& boundary,
                  const int width,
                  const int height,
                  const int subsample,
                  const vector<vector<pair<int, float> > >& weighted_visibility,
                  vector<int>* centers,
                  vector<vector<int> >* clusters) {
  // Repeat cluster and merge for 3 times.
  const int kTimes = 5;
  for (int t = 0; t < kTimes; ++t) {
    cerr << "Cluster" << endl;
    Cluster(boundary, width, height, subsample, weighted_visibility,
            centers, clusters);
    cerr << "Merge" << endl;
    const bool merged =
      Merge(width, height, subsample, weighted_visibility, kMergeThreshold, centers, clusters);

    if (!merged)
      break;
  }
  // Last one should be Cluster instead of Merge.
  Cluster(boundary, width, height, subsample, weighted_visibility,
          centers, clusters);
} 

}  // namespace

namespace room_segmentation {

  
void ConvertPointsToSweep(const ply::Points& points, Sweep* sweep) {
  if (points.empty()) {
    cerr << "Empty ply::Points" << endl;
    exit (1);
  }
  sweep->center = points[0].position;

  sweep->points.resize(points.size() - 1);
  for (int i = 1; i < points.size(); ++i) {
    sweep->points[i - 1].position = points[i].position;
    sweep->points[i - 1].normal   = points[i].normal;
    sweep->points[i - 1].weight   = points[i].intensity / 255.0;
  }
}
  
void SetRanges(const vector<Sweep>& sweeps,
               const float average_distance,
               Frame* frame) {
  // Do not take into account point weights.
  vector<double> histogram[3];

  for (const auto& sweep : sweeps) {
    for (const auto& point : sweep.points) {
      for (int a = 0; a < 3; ++a) {
        histogram[a].push_back(point.position.dot(frame->axes[a]));
      }
    }
  }

  for (int a = 0; a < 3; ++a) {
    sort(histogram[a].begin(), histogram[a].end());

    // Take the 5 and 95 percentiles.
    const auto pos_5  = histogram[a].begin() + (histogram[a].size() * 1 / 100);
    const auto pos_95 = histogram[a].begin() + (histogram[a].size() * 99 / 100);

    nth_element(histogram[a].begin(), pos_5,  histogram[a].end());
    nth_element(histogram[a].begin(), pos_95, histogram[a].end());

    const double diff = (*pos_95) - (*pos_5);
    // Allow a margin of 5 percents.
    const double margin = diff * 5 / 100;

    frame->ranges[a][0] = (*pos_5)  - margin;
    frame->ranges[a][1] = (*pos_95) + margin;


    //?????
    // if (a == 0)
    // frame->ranges[a][0] -= 1000;
  }

  //----------------------------------------------------------------------
  // Initial guess of unit.
  //???????
  //double unit = average_distance / 150.0;
  // double unit = average_distance / 80.0;
  double unit = average_distance / 50.0;
  // Compute resolution.
  const int width  = static_cast<int>(round((frame->ranges[0][1] - frame->ranges[0][0]) / unit));
  const int height = static_cast<int>(round((frame->ranges[1][1] - frame->ranges[1][0]) / unit));
  // const int depth  = static_cast<int>(round((frame->max_z - frame->min_z) / unit));
  const int kMaxResolution = 600;  // autodesk
  //const int kMaxResolution = 1024;
  // Don't use depth.
  const int max_current_resolution = max(width, height);
  if (kMaxResolution < max_current_resolution) {
    unit *= max_current_resolution / kMaxResolution;
  }

  frame->unit = unit;

  for (int a = 0; a < 3; ++a) {
    frame->size[a] = static_cast<int>(round((frame->ranges[a][1] - frame->ranges[a][0]) / unit));
  }
}
  
void ComputeFrame(const std::string directory,
                  const vector<Sweep>& sweeps,
                  const float average_distance,
                  Frame* frame) {
  frame->axes[0] = Vector3f(1, 0, 0);
  frame->axes[1] = Vector3f(0, 1, 0);
  frame->axes[2] = Vector3f(0, 0, 1);

  // Set frame and apply ToLocal.
  SetRanges(sweeps, average_distance, frame);
}

float ComputeAverageDistance(const std::vector<Sweep>& sweeps) {
  float average_distance = 0.0;
  int count = 0;

  for (const auto& sweep : sweeps) {
    for (const auto& point : sweep.points) {
      average_distance += (point.position - sweep.center).norm();
      ++count;
    }
  }
  
  if (count == 0) {
    cerr << "No points?" << endl;
    return 1.0;
  }
  
  return average_distance / count;
}

void LoadClustering(const std::string filename,
                    std::vector<Eigen::Vector2i>* centers,
                    std::vector<std::vector<Eigen::Vector2i> >* clusters) {
  ifstream ifstr;
  ifstr.open(filename.c_str());

  string header;
  int width, height;
  ifstr >> header >> width >> height;
  int num_cluster;
  ifstr >> num_cluster;
  centers->clear();
  clusters->clear();

  centers->resize(num_cluster);
  clusters->resize(num_cluster);
  for (int c = 0; c < num_cluster; ++c) {
    ifstr >> centers->at(c)[0] >> centers->at(c)[1];
  }

  for (int c = 0; c < num_cluster; ++c) {
    int num_point;
    ifstr >> num_point;
    clusters->at(c).resize(num_point);
    for (int p = 0; p < num_point; ++p) {
      ifstr >> clusters->at(c)[p][0] >> clusters->at(c)[p][1];
    }
  }  
  ifstr.close();
}
  
void WriteClustering(const int width,
                     const int height,
                     const std::vector<int>& centers,
                     const std::vector<std::vector<int> >& clusters,
                     const int subsample,
                     const string filename) {
  const int subsampled_width  = width / subsample;
  const int subsampled_height = height / subsample;

  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "INITIAL_CLUSTER" << endl
        << width << ' ' << height << endl;

  ofstr << (int)centers.size() << endl;
  for (int i = 0; i < (int)centers.size(); ++i) {
    const int subsampled_x = centers[i] % subsampled_width;
    const int subsampled_y = centers[i] / subsampled_width;
    const int x = kClusteringSubsample * subsampled_x;
    const int y = kClusteringSubsample * subsampled_y;
    ofstr << x << ' ' << y << endl;
  }
  
  for (int i = 0; i < (int)clusters.size(); ++i) {
    ofstr << (int)clusters[i].size() << endl;
    for (int j = 0; j < (int)clusters[i].size(); ++j) {
      const int index = clusters[i][j];
      const int subsampled_x = index % subsampled_width;
      const int subsampled_y = index / subsampled_width;
      const int x = kClusteringSubsample * subsampled_x;
      const int y = kClusteringSubsample * subsampled_y;
      ofstr << x << ' ' << y << endl;
    }
  }
  ofstr.close();
}    

void WriteVisibility(const string& filename,
                     const std::vector<std::vector<std::pair<int, float> > >& visibility,
                     const int width,
                     const int height,
                     const int subsample) {
  const int subsampled_width = width / subsample;
  const int subsampled_height = height / subsample;

  ofstream ofstr;
  ofstr.open(filename.c_str());

  ofstr << "VISIBILITY" << endl
        << width << ' ' << height << ' ' << subsample << endl;
  int index = 0;
  for (int y = 0; y < subsampled_height; ++y) {
    for (int x = 0; x < subsampled_width; ++x, ++index) {
      ofstr << (int)visibility[index].size() << endl;
      for (const auto& value : visibility[index]) {
        ofstr << value.first << ' ' << value.second << ' ';
      }
    }
  }
  
  ofstr.close();
}

void LoadVisibility(const string& filename,
                    std::vector<std::vector<std::pair<int, float> > >* visibility_no_subsampling,
                    int* width,
                    int* height,
                    int* subsample) {
  ifstream ifstr;
  ifstr.open(filename.c_str());

  string header;
  ifstr >> header >> *width >> *height >> *subsample;

  const int subsampled_width = (*width) / (*subsample);
  const int subsampled_height = (*height) / (*subsample);
  visibility_no_subsampling->resize((*width) * (*height));

  for (int y = 0; y < subsampled_height; ++y) {
    const int y_no_subsampling = y * (*subsample);
    for (int x = 0; x < subsampled_width; ++x) {
      const int x_no_subsampling = x * (*subsample);

      const int index_no_subsampling = y_no_subsampling * (*width) + x_no_subsampling;
      int length;
      ifstr >> length;
      visibility_no_subsampling->at(index_no_subsampling).resize(length);
      for (int i = 0; i < length; ++i)
        ifstr >> visibility_no_subsampling->at(index_no_subsampling)[i].first
              >> visibility_no_subsampling->at(index_no_subsampling)[i].second;
    }
  } 
  ifstr.close();
}

void DetectDoors(const vector<Sweep>& sweeps,
                 const Frame& frame,
                 const std::string directory,
                 const vector<float>& point_evidence,
                 const vector<float>& free_space_evidence,
                 vector<float>* door_detection) {
  // Hard thresholding, and randomly pick a pair of points that are farther away.
  vector<bool> mask;
  SetMask(frame, free_space_evidence, &mask);
  {
    char buffer[1024];
    sprintf(buffer, "%smask_before_open.pgm", directory.c_str());
    WriteMask(frame.size[0], frame.size[1], mask, buffer);
  }

  const int kKernelWidth = 9;
  for (int i = 0; i < 40; ++i)
    image_process::Open(frame.size[0], frame.size[1], kKernelWidth, &mask);

  {
    char buffer[1024];
    sprintf(buffer, "%smask_after_open.pgm", directory.c_str());
    WriteMask(frame.size[0], frame.size[1], mask, buffer);
  }

  cerr << "FindBoundary..." << flush;
  // Find boundary pixels.
  vector<pair<int, int> > boundary;
  FindBoundary(frame.size[0], frame.size[1], mask, &boundary);

  // Subsample.
  random_shuffle(boundary.begin(), boundary.end());
  boundary.resize(static_cast<int>(boundary.size() * kBoundarySubsampleRatio));
  sort(boundary.begin(), boundary.end());

  cerr << "SetDistanceBoundary..." << endl;
  vector<float> distance_to_boundary;
  SetDistanceToBoundary(frame, mask, &distance_to_boundary);
  
  cerr << "ComputeVisibility..." << endl;
  // Expensive to do visibility computation and clustering for all the pixels.
  // For each skipped pixel, a list of visible boundary indexes.
  vector<vector<int> > visibility;
  ComputeVisibility(frame.size[0], frame.size[1], kClusteringSubsample,
                    boundary, mask, distance_to_boundary, &visibility);

  cerr << "AssociateWeightToVisibility..." << endl;
  // Turn visibility to weighted visibility.
  vector<vector<pair<int, float> > > weighted_visibility;
  AssociateWeightToVisibility(frame.size[0], frame.size[1], kClusteringSubsample,
			      boundary, visibility, &weighted_visibility);

  {
    char buffer[1024];
    sprintf(buffer, "%svisibility.dat", directory.c_str());
    WriteVisibility(buffer, weighted_visibility, frame.size[0], frame.size[1], kClusteringSubsample);
  }

  // K-means clustering.
  // for (int s = 0; s < 5; ++s) {
  for (int s = 0; s < 1; ++s) {
    vector<int> centers;
    // Initialize centers.
    for (int i = 0; i < weighted_visibility.size(); ++i) {
      if (!weighted_visibility[i].empty()) {
        centers.push_back(i);
      }
    }
    random_shuffle(centers.begin(), centers.end());
    centers.resize(kInitialClusterNum);
    //----------------------------------------------------------------------
    vector<vector<int> > clusters;
    cerr << "ClusterMerge..." << endl;
    ClusterMerge(boundary, frame.size[0], frame.size[1], kClusteringSubsample,
                 weighted_visibility, &centers, &clusters);

    char buffer[1024];
    sprintf(buffer, "%scluster-%02d.ppm", directory.c_str(), s);
    DrawCluster(frame.size[0], frame.size[1], kClusteringSubsample,
                buffer, centers, clusters);
    cerr << "Wrote: " << buffer << endl;

    sprintf(buffer, "%sinitial_cluster.dat", directory.c_str());
    WriteClustering(frame.size[0], frame.size[1], centers, clusters, kClusteringSubsample, buffer);
  }
  

  /*
  // Seed points to compute shortest paths.
  vector<pair<int, int> > seeds;
  for (int y = 0; y < frame.size[1]; y += kSeedStep) {
    for (int x = 0; x < frame.size[0]; x += kSeedStep) {
      if (mask[y * frame.size[0] + x])
        seeds.push_back(make_pair(x, y));
    }
  }

  door_detection->clear();
  door_detection->resize(frame.size[0] * frame.size[1], 0.0);

  // Repeat computing shortest paths.
  for (int i = 0; i < seeds.size(); ++i) {
    if (i % 20 == 0)
      cerr << i << "\t/\t" << seeds.size() << endl;

    vector<float> path_counts;
    FindShortestPaths(mask, distance_to_boundary,
                      frame.size[0], frame.size[1], seeds[i], seeds, &path_counts);

    for (int i = 0; i < path_counts.size(); ++i)
      door_detection->at(i) += path_counts[i];
  }

  const double kSigma = 5.0;
  BlurField(frame.size[0], frame.size[1], mask, kSigma, door_detection);
  DrawDoorDetection(frame.size[0], frame.size[1], mask, *door_detection, directory);
  */
}

float VisibilityDistance(const vector<pair<int, float> >& lhs,
                         const vector<pair<int, float> >& rhs) {
  int lhs_index = 0;
  int rhs_index = 0;

  float distance = 0.0;
  while (lhs_index < lhs.size() || rhs_index < rhs.size()) {
    if (lhs_index == lhs.size()) {
      distance += rhs[rhs_index].second;
      ++rhs_index;
    } else if (rhs_index == rhs.size()) {
      distance += lhs[lhs_index].second;
      ++lhs_index;
    } else if (lhs[lhs_index].first == rhs[rhs_index].first) {
      ++lhs_index;
      ++rhs_index;
    } else if (lhs[lhs_index].first < rhs[rhs_index].first) {
      distance += lhs[lhs_index].second;
      ++lhs_index;
    } else {
      distance += rhs[rhs_index].second;
      ++rhs_index;
    }
  }

  // By dividing by 2, the maximum distance is 1.0.
  return distance / 2.0;
}
  
}  // namespace room_segmentation
