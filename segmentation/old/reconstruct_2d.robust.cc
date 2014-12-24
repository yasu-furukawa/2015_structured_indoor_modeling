#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include "reconstruct_2d.h"
#include "../base/mrf/GCoptimization.h"

using namespace Eigen;
using namespace std;

namespace floored {

const Frame* frame;
const std::vector<float>* point_evidence;
const std::vector<float>* freespace_evidence;
std::string directory;
Floorplan* floorplan;

vector<Floorplan::SpaceType> shrunk_in_out;

const int kInvalid = -1;
Floorplan previous_floorplan;  
vector<int> horizontal_counts;
vector<int> vertical_counts;

vector<int> horizontal_count_map;
vector<int> vertical_count_map;

float smoothness_weight;
int iteration = 0;
const int kCountMapMargin = 50;
const float kMaxCountMargin = 20;
float count_margin;

namespace {

// Check how many times labels are different between (lhs + i step, rhs + istep).
float GetCount(const Floorplan& floorplan,
               const Vector2i& lhs,
               const Vector2i& rhs,
               const Vector2i& step,
               const int start,
               const int end) {
  int count = 0;
  for (int i = start; i <= end; ++i) {
    const Vector2i lhs_tmp = lhs + i * step;
    const Vector2i rhs_tmp = rhs + i * step;

    if (lhs_tmp[0] < 0 || floorplan.width <= lhs_tmp[0] ||
        lhs_tmp[1] < 0 || floorplan.height <= lhs_tmp[1] ||
        rhs_tmp[0] < 0 || floorplan.width <= rhs_tmp[0] ||
        rhs_tmp[1] < 0 || floorplan.height <= rhs_tmp[1]) {
      continue;
    }
    
    const int lhs_index = lhs_tmp[1] * floorplan.width + lhs_tmp[0];
    const int rhs_index = rhs_tmp[1] * floorplan.width + rhs_tmp[0];

    if (floorplan.in_out[lhs_index] != floorplan.in_out[rhs_index])
      ++count;
  }
  return count;
}

float GetHorizontalCountFromMap(const Floorplan& floorplan,
                                const Vector2i& lhs) {
  /*
  int count = 0;
  for (int y = 0; y < frame->size[1]; ++y) {
    const int distance = abs(y - lhs[1]);
    if (distance < margin + horizontal_count_map[y * frame->size[0] + lhs[0]])
      ++count;
  }
  return count;
  */
  return horizontal_count_map[lhs[1] * frame->size[0] + lhs[0]];
}

float GetVerticalCountFromMap(const Floorplan& floorplan,
                              const Vector2i& lhs) {
  /*
  int count = 0;
  for (int x = 0; x < frame->size[0]; ++x) {
    const int distance = abs(x - lhs[0]);
    if (distance < margin + vertical_count_map[lhs[1] * frame->size[0] + x])
      ++count;
  }
  return count;
  */
  return vertical_count_map[lhs[1] * frame->size[0] + lhs[0]];
}

void PrepareDataArray(vector<MRF::CostVal>* data_array) {
  const int kNumLabels = 2;
  data_array->clear();
  data_array->resize(kNumLabels * frame->size[0] * frame->size[1]);
  
  const MRF::CostVal kLarge = 100000.0f;
  for (int y = 0; y < frame->size[1]; ++y) {
    for (int x = 0; x < frame->size[0]; ++x) {
      const int index = y * frame->size[0] + x;
      if (x == 0 || x == frame->size[0] - 1 || y == 0 || y == frame->size[1] - 1) {
        data_array->at(kNumLabels * index)     = kLarge;
        data_array->at(kNumLabels * index + 1) = 0.0;
      } else {
        data_array->at(kNumLabels * index)     =
          static_cast<MRF::CostVal>(point_evidence->at(index));
        data_array->at(kNumLabels * index + 1) =
          static_cast<MRF::CostVal>(freespace_evidence->at(index));
      }
    }
  }
}

float RobustFunction(const float count, const float margin) {
  if (count < margin) {
    // return 1.0 - 0.1 * count / margin;
    //return (1.0 - 0.1 * count / margin) * exp(iteration / 3);
    // return exp(iteration / 3);
    return exp(min(16 / 3, iteration / 3));
  } else {
    return max(0.1f, 0.2f * (1.0f - min(1.0f, (count - margin) / margin)));
  }
}
  /*  
MRF::CostVal SmoothFunc(int lhs, int rhs, MRF::Label lhs_label, MRF::Label rhs_label) {
  //const float kSigma = 50.0;
  //const float kSigma = 10.0;
  //const float kSigma2 = 2 * kSigma * kSigma;
  const float epsilon = 0.01f;
  
  if (horizontal_counts.empty()) {
    if (lhs_label == rhs_label)
      return 0.0;
    else
      return smoothness_weight;
  } else {
    if (lhs_label == rhs_label)
      return 0.0;
    else {
      if (fabs(lhs - rhs) == 1) {
        // Horizontal.
        const int xtmp = min(lhs, rhs) % frame->size[0];
        return smoothness_weight * max(epsilon, count_margin - horizontal_counts[xtmp]);
        //return smoothness_weight * RobustFunction(horizontal_counts[xtmp], count_margin);
      } else if (fabs(lhs - rhs) == frame->size[0]) {
        // Vertical.
        const int ytmp = min(lhs, rhs) / frame->size[0];
        return smoothness_weight * max(epsilon, count_margin - vertical_counts[ytmp]);
         //return smoothness_weight * RobustFunction(vertical_counts[ytmp], count_margin);
      } else {
        return smoothness_weight;
      }
    }
  }
}
  */
MRF::CostVal SmoothFunc(int lhs, int rhs, MRF::Label lhs_label, MRF::Label rhs_label) {
  //const float kSigma = 50.0;
  // const float kSigma = 5.0;
  // ???? critical.
  // const float kSigma2 = 2 * kSigma * kSigma;
  // const float epsilon = 0.01f;
  
  if (previous_floorplan.width == kInvalid) {
    if (lhs_label == rhs_label)
      return 0.0;
    else
      return smoothness_weight;
  } else {
    if (lhs_label == rhs_label)
      return 0.0;
    else {
      const int xtmp = min(lhs, rhs) % frame->size[0];
      const int ytmp = min(lhs, rhs) / frame->size[0];

      float boundary_term = 0.0;
      {
        // Inside.
        if (lhs_label == 0) {
          boundary_term += (*point_evidence)[lhs];
        } else {
          ; //boundary_term -= (*point_evidence)[lhs];
        }
        if (rhs_label == 0) {
          boundary_term += (*point_evidence)[rhs];
        } else {
          ; // boundary_term -= (*point_evidence)[rhs];
        }
        boundary_term = max(0.0f, boundary_term);
        boundary_term = max(0.1f, exp(- boundary_term * boundary_term / (2 * 100 * 100)));
        //???
        boundary_term = 1.0f;
      }
              
      if (fabs(lhs - rhs) == 1) {
        // Horizontal.
        const float horizontal_count =
          /*
          GetCount(previous_floorplan,
                   Vector2i(xtmp, ytmp),
                   Vector2i(xtmp + 1, ytmp),
                   Vector2i(0, 1),
                   -kMargin,
                   kMargin);
          */
          GetHorizontalCountFromMap(previous_floorplan, Vector2i(xtmp, ytmp));
       
        //return smoothness_weight * boundary_term *
        //max(epsilon, exp(- horizontal_count * horizontal_count / kSigma2));
        // return max(epsilon, count_margin - horizontal_count) * smoothness_weight * boundary_term;

        return smoothness_weight * RobustFunction(horizontal_count, count_margin);
      } else if (fabs(lhs - rhs) == frame->size[0]) {
        // Vertical.
        const float vertical_count =
          /*
          GetCount(previous_floorplan,
                   Vector2i(xtmp, ytmp),
                   Vector2i(xtmp, ytmp + 1),
                   Vector2i(1, 0),
                   -kMargin,
                   kMargin);
          */
          GetVerticalCountFromMap(previous_floorplan, Vector2i(xtmp, ytmp));
          

        //return smoothness_weight * boundary_term *
        //max(epsilon, exp(- vertical_count * vertical_count / kSigma2));
        // return max(epsilon, count_margin - vertical_count) * smoothness_weight * boundary_term;
        return smoothness_weight * RobustFunction(vertical_count, count_margin);
      } else {
        return smoothness_weight * boundary_term;
      }
    }
  }
}
  
void SetCounts() {
  horizontal_counts.clear();
  horizontal_counts.resize(frame->size[0], 0);

  vertical_counts.clear();
  vertical_counts.resize(frame->size[1], 0);

  for (int y = 0; y < frame->size[1]; ++y) {
    for (int x = 0; x < frame->size[0]; ++x) {
      const int index = y * frame->size[0] + x;
      if (x != frame->size[0] - 1) {
        if (floorplan->in_out[index] != floorplan->in_out[index + 1])
          ++horizontal_counts[x];
      }
      if (y != frame->size[1] - 1) {
        if (floorplan->in_out[index] != floorplan->in_out[index + frame->size[0]])
          ++vertical_counts[y];
      }
    }
  }
}

void SetShrunkInOut() {
  if (previous_floorplan.width == kInvalid) {
    return;
  }

  shrunk_in_out = previous_floorplan.in_out;
  vector<Floorplan::SpaceType> inout_tmp;

  // Shrinks by count_margin / 2.
  const int margin = count_margin;

  for (int iteration = 0; iteration < margin; ++iteration) {
    inout_tmp = shrunk_in_out;

    for (int y = 1; y < previous_floorplan.height - 1; ++y) {
      for (int x = 1; x < previous_floorplan.width - 1; ++x) {
        // If boundary and inside, set it to outside.
        const int index = y * previous_floorplan.width + x;
        if (inout_tmp[index] != Floorplan::Inside)
          continue;
        
        for (int j = -1; j <= 1; ++j) {
          for (int i = -1; i <= 1; ++i) {
            if (i == 0 && j == 0)
              continue;
            const int neighbor_index = (y + j) * previous_floorplan.width + (x + i);
            if (inout_tmp[neighbor_index] == Floorplan::Outside)
              shrunk_in_out[index] = Floorplan::Outside;
          }
        }
      }
    }
  }
}

void SetCountMapsWithShrunkInOut() {
  horizontal_count_map.clear();
  horizontal_count_map.resize(frame->size[0] * frame->size[1], 0);

  for (int x = 0; x < frame->size[0] - 1; ++x) {
    for (int y = 0; y < frame->size[1]; ++y) {
      // Check how many label changes continuously.
      int end_y;
      for (end_y = y; end_y < frame->size[1]; ++end_y) {
        if (previous_floorplan.in_out[end_y * frame->size[0] + x] !=
            previous_floorplan.in_out[end_y * frame->size[0] + x + 1]) {
          continue;
        } else {
          break;
        }
      }
      if (y == end_y)
        continue;
      
      const int count = end_y - y;
      const int length = end_y - y;
      // Fill in the value for [y, end_y) with count.
      const int margin = (kCountMapMargin + length);// / exp(max(0, iteration - 10) / 2);

      for (int ytmp = y; ytmp < end_y; ++ytmp) {
        horizontal_count_map[ytmp * frame->size[0] + x] =
          max(horizontal_count_map[ytmp * frame->size[0] + x], count);
      }
      
      // Negative direction.
      for (int i = 1; i < margin; ++i) {
        const int ytmp = y - i;
        if (ytmp < 0)
          break;
        if (shrunk_in_out[ytmp * frame->size[0] + x] == Floorplan::Inside)
          break;
        
        horizontal_count_map[ytmp * frame->size[0] + x] =
          max(horizontal_count_map[ytmp * frame->size[0] + x], count);
      }
      // Positive direction.
      for (int i = 1; i < margin; ++i) {
        const int ytmp = end_y + i - 1;
        if (frame->size[1] <= ytmp)
          break;
        if (shrunk_in_out[ytmp * frame->size[0] + x] == Floorplan::Inside)
          break;
        
        horizontal_count_map[ytmp * frame->size[0] + x] =
          max(horizontal_count_map[ytmp * frame->size[0] + x], count);
      }
      
      y = end_y - 1;
    }
  }


  vertical_count_map.clear();
  vertical_count_map.resize(frame->size[0] * frame->size[1], 0);

  for (int y = 0; y < frame->size[1] - 1; ++y) {
    for (int x = 0; x < frame->size[0]; ++x) {
      // Check how many label changes continuously.
      int end_x;
      for (end_x = x; end_x < frame->size[0]; ++end_x) {
        if (previous_floorplan.in_out[y * frame->size[0] + end_x] !=
            previous_floorplan.in_out[(y + 1) * frame->size[0] + end_x]) {
          continue;
        } else {
          break;
        }
      }
      if (x == end_x)
        continue;

      const int count = end_x - x;
      const int length = end_x - x;
      // Fill in the value for [x, end_x) with count.
      const int margin = (kCountMapMargin + length);// / exp(max(0, iteration - 10) / 2);

      for (int xtmp = x; xtmp < end_x; ++xtmp) {
        vertical_count_map[y * frame->size[0] + xtmp] =
          max(vertical_count_map[y * frame->size[0] + xtmp], count);
      }

      // Negative direction.
      for (int i = 1; i < margin; ++i) {
        const int xtmp = x - i;
        if (xtmp < 0)
          break;
        if (shrunk_in_out[y * frame->size[0] + xtmp] == Floorplan::Inside)
          break;
        
        vertical_count_map[y * frame->size[0] + xtmp] =
          max(vertical_count_map[y * frame->size[0] + xtmp], count);
      }
      // Positive direction.
      for (int i = 1; i < margin; ++i) {
        const int xtmp = end_x + i - 1;
        if (frame->size[0] <= xtmp)
          break;
        if (shrunk_in_out[y * frame->size[0] + xtmp] == Floorplan::Inside)
          break;
        
        vertical_count_map[y * frame->size[0] + xtmp] =
          max(vertical_count_map[y * frame->size[0] + xtmp], count);
      }
      
      x = end_x - 1;
    }
  }
}
  
void SetCountMaps() {
  horizontal_count_map.clear();
  horizontal_count_map.resize(frame->size[0] * frame->size[1], 0);

  for (int x = 0; x < frame->size[0] - 1; ++x) {
    for (int y = 0; y < frame->size[1]; ++y) {
      // Check how many label changes continuously.
      int end_y;
      for (end_y = y; end_y < frame->size[1]; ++end_y) {
        if (previous_floorplan.in_out[end_y * frame->size[0] + x] !=
            previous_floorplan.in_out[end_y * frame->size[0] + x + 1]) {
          continue;
        } else {
          break;
        }
      }
      if (y == end_y)
        continue;
      
      const int count = end_y - y;
      const int length = end_y - y;
      // Fill in the value for [y, end_y) with count.
      const int margin = (kCountMapMargin + length);// / exp(max(0, iteration - 10) / 2);
      for (int ytmp = max(0, y - margin);
           ytmp < min(end_y + margin, frame->size[1]);
           ++ytmp) {
        horizontal_count_map[ytmp * frame->size[0] + x] =
          max(horizontal_count_map[ytmp * frame->size[0] + x], count);
      }
      y = end_y - 1;
    }
  }


  vertical_count_map.clear();
  vertical_count_map.resize(frame->size[0] * frame->size[1], 0);

  for (int y = 0; y < frame->size[1] - 1; ++y) {
    for (int x = 0; x < frame->size[0]; ++x) {
      // Check how many label changes continuously.
      int end_x;
      for (end_x = x; end_x < frame->size[0]; ++end_x) {
        if (previous_floorplan.in_out[y * frame->size[0] + end_x] !=
            previous_floorplan.in_out[(y + 1) * frame->size[0] + end_x]) {
          continue;
        } else {
          break;
        }
      }
      if (x == end_x)
        continue;

      const int count = end_x - x;
      const int length = end_x - x;
      // Fill in the value for [x, end_x) with count.
      const int margin = (kCountMapMargin + length);// / exp(max(0, iteration - 10) / 2);
      for (int xtmp = max(0, x - margin);
           xtmp < min(end_x + margin, frame->size[0]);
           ++xtmp) {
        vertical_count_map[y * frame->size[0] + xtmp] =
          max(vertical_count_map[y * frame->size[0] + xtmp], count);
      }
      x = end_x - 1;
    }
  }
}

void SetNeighbors(MRF* mrf) {
  const float kUniformWeight = 1.0;
  for (int y = 0; y < frame->size[1]; ++y) {
    for (int x = 0; x < frame->size[0]; ++x) {
      const int index = y * frame->size[0] + x;
      // Right.
      if (x != frame->size[0] - 1)
        mrf->setNeighbors(index, index + 1, kUniformWeight);
      // Bottom.
      if (y != frame->size[1] - 1)
        mrf->setNeighbors(index, index + frame->size[0], kUniformWeight);
    }
  }
}

// mrf should be constant, but the library does not use const member function.
void SetFloorplan(MRF& mrf) {
  floorplan->width = frame->size[0];
  floorplan->height = frame->size[1];

  floorplan->in_out.resize(floorplan->width * floorplan->height);
  for (int i = 0; i < floorplan->width * floorplan->height; ++i) {
    if (mrf.getLabel(i) == 0)
      floorplan->in_out[i] = Floorplan::Inside;
    else
      floorplan->in_out[i] = Floorplan::Outside;
  }
}

void WriteImage(const string filename, const Floorplan& floorplan) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P3" << endl
        << floorplan.width << ' ' << floorplan.height << endl
        << 255 << endl;
  for (int i = 0; i < floorplan.in_out.size(); ++i) {
    switch (floorplan.in_out[i]) {
    case Floorplan::Inside:
      ofstr << "0 0 255 ";
      break;
    case Floorplan::Outside:
      ofstr << "255 0 0 ";
      break;
    default:
      ofstr << "255 255 255 ";
      break;
    }
  }
  ofstr.close();
}

void WriteHorizontalSmoothness(const string filename,
                               const Floorplan& floorplan) {
  /*
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P3" << endl
        << floorplan.width << ' ' << floorplan.height << endl
        << 255 << endl;
  for (int y = 0; y < floorplan.height; ++y) {
    for (int x = 0; x < floorplan.width; ++x) {
      // Right.
      if (x == floorplan.width - 1) {
        ofstr << "0 0 0 ";
      } else if (floorplan.in_out[y * floorplan.width + x] ==
                 floorplan.in_out[y * floorplan.width + x + 1]) {
        ofstr << "0 0 0 ";
      } else {
        const int gray = 255 - min(255, static_cast<int>(255 * horizontal_counts[x] / count_margin));
        int red, green, blue;
        if (gray < 128) {
          green  = gray * 2;
          blue = 255 - green;
          red   = 0;
        } else {
          blue  = 0;
          red   = (gray - 128) * 2;
          green = 255 - red;
        }
        ofstr << red << ' ' << green << ' ' << blue << ' ';
      }
    }
  }
  ofstr.close();
  */

  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P3" << endl
        << floorplan.width << ' ' << floorplan.height << endl
        << 255 << endl;
  for (int y = 0; y < floorplan.height; ++y) {
    for (int x = 0; x < floorplan.width; ++x) {
      // Right.
      if (x == floorplan.width - 1 || horizontal_count_map[y * floorplan.width + x] == 0) {
        ofstr << "0 0 0 ";
      } else {
        const int gray = 255 - min(255, static_cast<int>(255 * horizontal_count_map[y * floorplan.width + x] / count_margin));
        int red, green, blue;
        if (gray < 128) {
          green  = gray * 2;
          blue = 255 - green;
          red   = 0;
        } else {
          blue  = 0;
          red   = (gray - 128) * 2;
          green = 255 - red;
        }
        ofstr << red << ' ' << green << ' ' << blue << ' ';
      }
    }
  }
  ofstr.close();
}

void WriteVerticalSmoothness(const string filename,
                               const Floorplan& floorplan) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P3" << endl
        << floorplan.width << ' ' << floorplan.height << endl
        << 255 << endl;
  for (int y = 0; y < floorplan.height; ++y) {
    for (int x = 0; x < floorplan.width; ++x) {
      // Right.
      if (y == floorplan.height - 1 || vertical_count_map[y * floorplan.width + x] == 0) {
        ofstr << "0 0 0 ";
      } else {
        const int gray =
          255 - min(255,
                    static_cast<int>(255 * vertical_count_map[y * floorplan.width + x] / count_margin));
        int red, green, blue;
        if (gray < 128) {
          green  = gray * 2;
          blue = 255 - green;
          red   = 0;
        } else {
          blue  = 0;
          red   = (gray - 128) * 2;
          green = 255 - red;
        }
        ofstr << red << ' ' << green << ' ' << blue << ' ';
      }
    }
  }
  ofstr.close();
}
  
  
}  // namespace

  
void Reconstruct2D(const Frame& frame_tmp,
                   const std::vector<float>& point_evidence_tmp,
                   const std::vector<float>& freespace_evidence_tmp,
                   const std::string& directory_tmp,
                   Floorplan* floorplan_tmp) {
  frame               = &frame_tmp;
  point_evidence      = &point_evidence_tmp;
  freespace_evidence  = &freespace_evidence_tmp;
  directory           = directory_tmp;
  floorplan           = floorplan_tmp;
  
  previous_floorplan.width = kInvalid;
  previous_floorplan.height = kInvalid;

  const int kIteration = 50;
  smoothness_weight = 200.0;
  count_margin =5;

  for (iteration = 0; iteration < kIteration; ++iteration) {
    cerr << "Iteration " << iteration << endl;
    vector<MRF::CostVal> data_array;
    PrepareDataArray(&data_array);

    /*
    DataCost* data = new DataCost(&data_array[0]);
    SmoothnessCost* smooth = new SmoothnessCost(&SmoothFunc);
    EnergyFunction* energy = new EnergyFunction(data, smooth);
    const int vnum = frame->size[0] * frame->size[1];
    MRF* mrf = new Expansion(vnum, 2, energy);
    */
    DataCost data(&data_array[0]);
    SmoothnessCost smooth(&SmoothFunc);
    EnergyFunction energy(&data, &smooth);
    const int vnum = frame->size[0] * frame->size[1];
    Expansion expansion(vnum, 2, &energy);
    MRF* mrf = &expansion;

    SetNeighbors(mrf);

    mrf->initialize();
    mrf->clearAnswer();
    /*
    cout << "Energy at start (total/data/smooth): "
         << mrf->totalEnergy() << ' '
         << mrf->dataEnergy() << ' '
         << mrf->smoothnessEnergy() << endl;
    */
    float t;
    mrf->optimize(1, t);
    
    cout << "Energy at end (total/data/smooth): "
         << mrf->totalEnergy() << ' '
         << mrf->dataEnergy() << ' '
         << mrf->smoothnessEnergy() << endl;
    
    SetFloorplan(*mrf);

    previous_floorplan = *floorplan;

    cerr << "Setting counts..." << flush;
    SetCounts();
    cerr << "done." << endl
         << "Setting count_maps..." << flush;
    SetShrunkInOut();
    // SetCountMaps();
    SetCountMapsWithShrunkInOut();
    cerr << "done." << endl;
    // ????
    if (iteration % 3 == 2) {
      // smoothness_weight *= 1.2;
      count_margin += 1.0;
      count_margin = min(kMaxCountMargin, count_margin);
      cerr << "Count Margin: " << count_margin << endl;
    }

    {
      Floorplan fp = previous_floorplan;
      fp.in_out = shrunk_in_out;
      char buffer[1024];
      sprintf(buffer, "%s/shrunk-%02d.ppm", directory.c_str(), iteration);
      WriteImage(buffer, fp);
    }    
    
    char buffer[1024];
    sprintf(buffer, "%s/%02d.ppm", directory.c_str(), iteration);
    WriteImage(buffer, *floorplan);
    sprintf(buffer, "%s/hor-%02d.ppm", directory.c_str(), iteration);
    WriteHorizontalSmoothness(buffer, *floorplan);
    sprintf(buffer, "%s/ver-%02d.ppm", directory.c_str(), iteration);
    WriteVerticalSmoothness(buffer, *floorplan);
    cerr << "all done." << endl;
    //delete data;
    //delete smooth;
    //delete energy;
    //delete mrf;
    cerr << "freed." << endl;
  }
}

}  // namespace floored
