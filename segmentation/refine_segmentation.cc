#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "../base/mrf/GCoptimization.h"
#include "door_detection.h"
#include "refine_segmentation.h"

using namespace Eigen;
using namespace room_segmentation;
using namespace std;

bool no_smooth = false;
const std::vector<float>* point_evidence_ptr;
const std::vector<float>* free_space_evidence_ptr;
const std::vector<Vector3d>* normal_evidence_ptr;
const floored::Frame* frame_ptr;

std::vector<int> segmentation_tmp;
std::vector<int> horizontal_counts;
std::vector<int> vertical_counts;

bool debug = false;

namespace {

void PrepareDataArray(const floored::Frame& frame,
                      const std::vector<Eigen::Vector2i>& centers,
                      const std::vector<std::vector<std::pair<int, float> > > visibility,
                      std::vector<MRF::CostVal>* data_array) {  
  const MRF::CostVal kLarge = 10000.0f;
  const int width = frame.size[0];
  const int height = frame.size[1];
  const int num_label = 1 + (int)centers.size();

  const MRF::CostVal kDataWeight = 10.0;
  // Free space is the same as background with this confidence.
  const double kFreeSpaceSameAsBackground = 0.1;

  // Check.
  {
    for (const auto& center : centers) {
      const int index = center[1] * width + center[0];
      if (visibility[index].empty()) {
        cerr << "Cluster center has no visibility..." << endl;
        exit (1);
      }
    }
  }

  data_array->clear();
  data_array->reserve(num_label * width * height);

  int index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      // Boundary must be background.
      if (x == 0 || x == width - 1 || y == 0 || y == height - 1) {
        data_array->push_back(0);
        for (int l = 1; l < num_label; ++l)
          data_array->push_back(kLarge);
      } else if (visibility[index].empty()) {
        data_array->push_back(kDataWeight * free_space_evidence_ptr->at(index));
        for (int l = 1; l < num_label; ++l) {
          data_array->push_back(kDataWeight * kFreeSpaceSameAsBackground);
        }
      } else {
        data_array->push_back(kDataWeight * free_space_evidence_ptr->at(index));
        for (int c = 0; c < (int)centers.size(); ++c) {
          const int center_index = centers[c][1] * width + centers[c][0];
          data_array->push_back(4.0 * kDataWeight * VisibilityDistance(visibility[index], visibility[center_index]));
        }
      }
    }
  }
}

MRF::CostVal SmoothFunc(int lhs, int rhs, MRF::Label lhs_label, MRF::Label rhs_label) {
  if (no_smooth)
    return 0.0;

  const int width = frame_ptr->size[0];
  const int height = frame_ptr->size[1];
  
  const MRF::CostVal kSmoothPenalty = 30.0;
  const MRF::CostVal kLarge = 100.0f;

  if (lhs_label == rhs_label)
    return 0.0;
  // If one is background.
  if (lhs_label == 0 || rhs_label == 0) {
    const int lhs_x = lhs % width;
    const int lhs_y = lhs / width;
    const int rhs_x = rhs % width;
    const int rhs_y = rhs / width;
    Vector3f expected_normal = (rhs_x - lhs_x) * frame_ptr->axes[0] + (rhs_y - lhs_y) * frame_ptr->axes[1];
    if (rhs_label == 0) {
      expected_normal = -expected_normal;
    }

    const Vector3d normal = normal_evidence_ptr->at(lhs) + normal_evidence_ptr->at(rhs);
    double length = normal.norm();
    if (length == 0.0)
      return kSmoothPenalty * 1.2;

    // At least 3 points.
    length = max(3.0, length);
    
    Vector3f normalf(normal[0], normal[1], normal[2]);
    const double factor = (normalf.dot(expected_normal) / length + 1.0) / 2.0;

    return kSmoothPenalty * (1.0 - factor * (point_evidence_ptr->at(lhs) + point_evidence_ptr->at(rhs)) / 2.0);
    // return kSmoothPenalty;
  }
  return kLarge;
}

void SetNeighbors(const floored::Frame& frame, MRF* mrf) {
  const float kUniformWeight = 1.0;
  for (int y = 0; y < frame.size[1]; ++y) {
    for (int x = 0; x < frame.size[0]; ++x) {
      const int index = y * frame.size[0] + x;
      // Right.
      if (x != frame.size[0] - 1) {
        mrf->setNeighbors(index, index + 1, kUniformWeight);
        mrf->setNeighbors(index, index + 1, kUniformWeight);
      }
      // Bottom.
      if (y != frame.size[1] - 1) {
        mrf->setNeighbors(index, index + frame.size[0], kUniformWeight);
        mrf->setNeighbors(index, index + frame.size[0], kUniformWeight);
      }
    }
  }
}

}  // namespace

void NormalizeEvidence(const double min_sigma,
                       const double max_sigma,
                       std::vector<float>* evidence) {
  double average = 0.0;
  int denom = 0;

  for (const auto& value : *evidence) {
    if (value != 0.0) {
      average += value;
      ++denom;
    }
  }

  if (denom == 0) {
    cerr << "Cannot normalize." << endl;
    exit (1);
  }

  average /= denom;
  double variance = 0.0;

  for (const auto& value : *evidence) {
    if (value != 0.0) {
      variance += (value - average) * (value - average);
    }
  }
  variance /= denom;
  const double deviation = sqrt(variance);

  ofstream ofstr;
  ofstr.open("a.txt");
  for (auto& value : *evidence) {
    if (value != 0.0) {
      ofstr << value << endl;
      value = (value - average) / deviation;
      value = max(0.0, min(1.0, (value - min_sigma) / (max_sigma - min_sigma)));
    }
  }
  ofstr.close();
}

void DrawEvidence3(const int width, const int height,
                   const Eigen::Vector3f& x_axis,
                   const Eigen::Vector3f& y_axis,
                   const std::vector<Eigen::Vector3d>& evidence,
                   const std::string& filename) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;

  for (auto normal : evidence) {
    if (normal.norm() == 0.0) {
      ofstr << "0 0 0 ";
    } else {
      double length = max(30.0, normal.norm());
      normal = normal / length;
      Vector3f normalf(normal[0], normal[1], normal[2]);
      const double dot_x = normalf.dot(x_axis);
      const double dot_y = normalf.dot(y_axis);

      int red = min(255, max(0, (int)round((dot_x + 1.0) / 2.0 * 255.0)));
      int green = min(255, max(0, (int)round((dot_y + 1.0) / 2.0 * 255.0)));
      int blue = 127;
      ofstr << red << ' ' << green << ' ' << blue << ' ';
    }
  }
  ofstr.close();
}

void DrawEvidence(const int width, const int height,
                  const std::vector<float>& evidence,
                  const std::string& filename,
                  const double scale) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;

  for (const auto& value : evidence) {
    if (value == 0) {
      ofstr << "0 0 0 ";
    } else {
      const int gray_scale = min(255, static_cast<int>(scale * value));
      int red, green, blue;
      if (gray_scale < 128) {
        green  = gray_scale * 2;
        blue = 255 - green;
        red   = 0;
      } else {
        blue  = 0;
        red   = (gray_scale - 128) * 2;
        green = 255 - red;
      }
      ofstr << red << ' ' << green << ' ' << blue << ' ';
    }
  }
  ofstr.close();
}

void RefineSegmentation(const floored::Frame& frame,
                        const std::vector<float>& point_evidence,
                        const std::vector<float>& free_space_evidence,
                        const std::vector<Eigen::Vector3d>& normal_evidence,
                        const std::vector<Eigen::Vector2i>& centers,
                        const std::vector<std::vector<Eigen::Vector2i> >& clusters,
                        const std::vector<std::vector<std::pair<int, float> > > visibility,
                        vector<int>* segmentation) {
  frame_ptr = &frame;
  point_evidence_ptr = &point_evidence;
  free_space_evidence_ptr = &free_space_evidence;
  normal_evidence_ptr = &normal_evidence;
    
  cerr << "Prepare data" << endl;
  // Labels are [background, centers].
  vector<MRF::CostVal> data_array;
  PrepareDataArray(frame, centers, visibility, &data_array);
  
  DataCost data(&data_array[0]);
  SmoothnessCost smooth(&SmoothFunc);
  EnergyFunction energy(&data, &smooth);
  const int vnum = frame.size[0] * frame.size[1];
  const int label_num = 1 + (int)centers.size();
  Expansion expansion(vnum, label_num, &energy);
  MRF* mrf = &expansion;

  SetNeighbors(frame, mrf);

  mrf->initialize();
  no_smooth = true;
  float t;
  mrf->optimize(1, t);
  no_smooth = false;

  cout << "Energy at start (total/data/smooth): "
       << mrf->totalEnergy() << ' '
       << mrf->dataEnergy() << ' '
       << mrf->smoothnessEnergy() << endl;
  
  mrf->optimize(6, t);

  
  cout << "Energy at end (total/data/smooth): "
       << mrf->totalEnergy() << ' '
       << mrf->dataEnergy() << ' '
       << mrf->smoothnessEnergy() << endl;

  debug = true;
  cout << mrf->smoothnessEnergy() << endl;

  segmentation->clear();
  segmentation->resize(frame.size[0] * frame.size[1]);
  for (int i = 0; i < frame.size[0] * frame.size[1]; ++i)
    segmentation->at(i) = mrf->getLabel(i);
  
  
  /*  
  for (iteration = 0; iteration < kIteration; ++iteration) {
    cerr << "Iteration " << iteration << endl;
    PrepareDataArray(&data_array);

    mrf->initialize();
    mrf->clearAnswer();

    cerr << "optimize" << endl;
    float t;
    mrf->optimize(1, t);
  }    
  */
    
}

void WriteSegmentation(const std::string& output_file,
                       const std::vector<int>& segmentation,
                       const int width,
                       const int height,
                       const int num_label) {
  vector<Eigen::Vector3i> colors(num_label);
  colors[0] = Vector3i(0, 0, 0);
  for (int i = 1; i < num_label; ++i) {
    for (int j = 0; j < 3; ++j)
      colors[i][j] = rand() % 255;
  }
  
  ofstream ofstr;
  ofstr.open(output_file.c_str());

  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;
  for (const auto& value : segmentation) {
    if (value < 0 || colors.size() <= value) {
      cerr << "Segmentation index out of bounds." << endl;
      exit (1);
    }
    ofstr << colors[value][0] << ' ' << colors[value][1] << ' ' << colors[value][2] << ' ';
  }

  ofstr.close();
}

void ExpandVisibility(const int width, const int height,
                      std::vector<std::vector<std::pair<int, float> > >* visibility) {
  std::vector<std::vector<std::pair<int, float> > > visibility_tmp = *visibility;

  int index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      if (visibility_tmp[index].empty()) {
        // Right.
        if (x != width - 1 && !visibility_tmp[index + 1].empty()) {
          visibility->at(index) = visibility->at(index + 1);
          continue;
        }
        // Left.
        if (x != 0 && !visibility_tmp[index - 1].empty()) {
          visibility->at(index) = visibility->at(index - 1);
          continue;
        }
        // bottom
        if (y != height - 1 && !visibility_tmp[index + width].empty()) {
          visibility->at(index) = visibility->at(index + width);
          continue;
        }
        // top.
        if (y != 0 && !visibility_tmp[index - width].empty()) {
          visibility->at(index) = visibility->at(index - width);
          continue;
        }
      }
    }
  }
}
