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

const int kInvalid = -1;
Floorplan previous_floorplan;  
vector<int> horizontal_counts;
vector<int> vertical_counts;

vector<int> horizontal_count_map;
vector<int> vertical_count_map;

float smoothness_weight;
int iteration = 0;
const int kCountMapMargin = 50;
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
                                const Vector2i& lhs,
                                const int margin) {
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
                              const Vector2i& lhs,
                              const int margin) {
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
    return (1.0 - 0.1 * count / margin) * 20;
    //return exp(iteration / 3);
  } else {
    return 0.1f + 0.1f * (1.0f - min(1.0f, (count - margin) / 1000.0f));
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
  const float epsilon = 0.01f;
  
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
              
      const int kMargin = 100; // 2 * count_margin;
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
          GetHorizontalCountFromMap(previous_floorplan,
                                    Vector2i(xtmp, ytmp),
                                    kMargin);
       
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
          GetVerticalCountFromMap(previous_floorplan,
                                  Vector2i(xtmp, ytmp),
                                  kMargin);
          

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
      
      const int count = end_y - y - 1;
      const int length = end_y - y;
      // Fill in the value for [y, end_y) with count.
      for (int ytmp = y - 1;
           ytmp < end_y + 1;
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

      const int count = end_x - x - 1;
      const int length = end_x - x;
      // Fill in the value for [x, end_x) with count.
      for (int xtmp = x - 1;
           xtmp < end_x + 1;
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

  const int kIteration = 25;
  smoothness_weight = 200.0;
  count_margin = 10;

  for (iteration = 0; iteration < kIteration; ++iteration) {
    vector<MRF::CostVal> data_array;
    
    cerr << "Iteration " << iteration << endl;
    PrepareDataArray(&data_array);

    DataCost data(&data_array[0]);
    SmoothnessCost smooth(&SmoothFunc);
    EnergyFunction energy(&data, &smooth);
    const int vnum = frame->size[0] * frame->size[1];
    Expansion expansion(vnum, 2, &energy);
    MRF* mrf = &expansion;
    
    SetNeighbors(mrf);
    
    /*
    DataCost* data = new DataCost(&data_array[0]);
    SmoothnessCost* smooth = new SmoothnessCost(&SmoothFunc);
    EnergyFunction* energy = new EnergyFunction(data, smooth);
    const int vnum = frame->size[0] * frame->size[1];
    MRF* mrf = new Expansion(vnum, 2, energy);
    */

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
    SetCountMaps();
    cerr << "done." << endl;
    // ????
    //if (iteration % 3 == 2) {
    smoothness_weight *= 1.2;
    //count_margin += 1.0;
    //count_margin = min(10.0f, count_margin);
    //}
    
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


    if (iteration == kIteration - 1) {
      while (true) {
        cerr << "x y: " << flush;
        int x, y;
        cin >> x >> y;
        const int margin = 2;
        for (int j = -margin; j <= margin; ++j) {
          for (int i = -margin; i <= margin; ++i) {
            cerr << floorplan->in_out[(y + j) * floorplan->width + (x + i)] << ' ';
          }
          cerr << endl;
        }

        const int index = y * floorplan->width + x;
        cerr << SmoothFunc(index, index - floorplan->width, 0, 1) << ' '
             << SmoothFunc(index, index + 1, 0, 1) << ' '
             << SmoothFunc(index, index + floorplan->width, 0, 1) << ' '
             << SmoothFunc(index, index - 1, 0, 1) << endl;
          

        
        cerr << "label 0 or 1: ";
        int label;
        cin >> label;
        if (label == 0)
          floorplan->in_out[index] = Floorplan::Inside;
        else
          floorplan->in_out[index] = Floorplan::Outside;
        
        float t = mrf->totalEnergy();
        float d = mrf->dataEnergy();
        float s = mrf->smoothnessEnergy();
        
        mrf->setLabel(index, label);
        
        cout << "Energy at end (total/data/smooth): "
             << mrf->totalEnergy() - t << ' '
             << mrf->dataEnergy() - d << ' '
             << mrf->smoothnessEnergy() - s<< endl;
      }
    }
  }
}

}  // namespace floored
