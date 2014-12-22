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
const floored::Frame* frame_ptr;

namespace {

void PrepareDataArray(const floored::Frame& frame,
                      const std::vector<Eigen::Vector2i>& centers,
                      const std::vector<std::vector<std::pair<int, float> > > visibility,
                      std::vector<MRF::CostVal>* data_array) {  
  const MRF::CostVal kBackground = 0.8;
  const MRF::CostVal kLarge = 10000.0f;
  const int width = frame.size[0];
  const int height = frame.size[1];
  const int num_label = 1 + (int)centers.size();

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
        data_array->push_back(10.0 * free_space_evidence_ptr->at(index));
        for (int l = 1; l < num_label; ++l) {
          data_array->push_back(10.0 * (1.0 - free_space_evidence_ptr->at(index)));
        }
      } else {
        data_array->push_back(1000 * free_space_evidence_ptr->at(index));
        for (int c = 0; c < (int)centers.size(); ++c) {
          const int center_index = centers[c][1] * width + centers[c][0];
          //data_array->push_back(1000 * (1.0f - free_space_evidence_ptr->at(index)) * VisibilityDistance(visibility[index], visibility[center_index]));
          data_array->push_back(1000 * VisibilityDistance(visibility[index], visibility[center_index]));
        }

        /*
        data_array->push_back(kBackground);
        for (int c = 0; c < (int)centers.size(); ++c) {
          const int center_index = centers[c][1] * width + centers[c][0];
          data_array->push_back(VisibilityDistance(visibility[index], visibility[center_index]));
        }
        */
      }
    }
  }
}

MRF::CostVal SmoothFunc(int lhs, int rhs, MRF::Label lhs_label, MRF::Label rhs_label) {
  if (no_smooth)
    return 0.001;
  
  const MRF::CostVal kSmoothPenalty = 0.4;
  const MRF::CostVal kLarge = 100.0f;

  cout << lhs << ' ' << rhs << ' ' << (int)point_evidence_ptr->size() << endl;
  
  if (lhs_label == rhs_label)
    return 0.0;
  // If one is background.
  if (lhs_label == 0 || rhs_label == 0) {
    return kSmoothPenalty * (1.0 + 0.2 - (point_evidence_ptr->at(lhs) + point_evidence_ptr->at(rhs)) / 2.0);
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
  ofstr.close();
}


void RefineSegmentation(const floored::Frame& frame,
                        const std::vector<float>& point_evidence,
                        const std::vector<float>& free_space_evidence,
                        const std::vector<Eigen::Vector2i>& centers,
                        const std::vector<std::vector<Eigen::Vector2i> >& clusters,
                        const std::vector<std::vector<std::pair<int, float> > > visibility,
                        vector<int>* segmentation) {
  frame_ptr = &frame;
  point_evidence_ptr = &point_evidence;
  free_space_evidence_ptr = &free_space_evidence;
    
  cerr << "Prepare data" << endl;
  // Labels are [background, centers].
  vector<MRF::CostVal> data_array;
  PrepareDataArray(frame, centers, visibility, &data_array);
  cout << "done" << endl;
  
  DataCost data(&data_array[0]);
  SmoothnessCost smooth(&SmoothFunc);
  EnergyFunction energy(&data, &smooth);
  const int vnum = frame.size[0] * frame.size[1];
  const int label_num = 1 + (int)centers.size();
  Expansion expansion(vnum, label_num, &energy);
  MRF* mrf = &expansion;

  cout << "setn" << endl;
  SetNeighbors(frame, mrf);

  mrf->initialize();
  no_smooth = true;
  float t;
  cout << "top" << endl;
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
