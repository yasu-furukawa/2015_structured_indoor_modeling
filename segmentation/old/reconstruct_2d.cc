#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include "evidence.h"
#include "reconstruct_2d.h"
#include "../base/mrf/GCoptimization.h"
#include "dynamic_programming.h"

using namespace Eigen;
using namespace std;

namespace floored {

  const bool kDenselyConnectedMRF = false; //true;

  

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

const float smoothness_weight = 100.0; // For densely.

  
  //  const float smoothness_weight = 200.0; // have been using this.
  //  const float smoothness_weight = 1600.0;

int iteration = 0;
const int kCountMapMargin = 50;

  //??????? critical parameter to control the amount of shortest details to keep. 10 was used for floored and henry initially.
  //const float kMaxCountMargin = 10;
  //  const float kMaxCountMargin = 15;
  //    const float kMaxCountMargin = 20;
  const float kMaxCountMargin = 5;
float count_margin;

namespace {

float GetHorizontalCountFromMap(const Floorplan& floorplan,
                                const Vector2i& lhs) {
  return horizontal_count_map[lhs[1] * frame->size[0] + lhs[0]];
}

float GetVerticalCountFromMap(const Floorplan& floorplan,
                              const Vector2i& lhs) {
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
          static_cast<MRF::CostVal>(ScalePointEvidence(point_evidence->at(index)));
        data_array->at(kNumLabels * index + 1) =
          static_cast<MRF::CostVal>(ScaleFreeSpaceEvidence(freespace_evidence->at(index)));
      }
    }
  }
}

float RobustFunction(const float count) {
  if (count < count_margin) {
    // return 1.0 - 0.1 * count / margin;
    //return (1.0 - 0.1 * count / margin) * exp(iteration / 3);
    // return exp(iteration / 3);
    //return exp(min(16 / 3, iteration / 3));
    if (iteration == 0)
      return 1.0;
    else
      //return pow(2.0, min(6, iteration));
      // return (1.0 - 0.1 * count / count_margin) * pow(2.0, min(6, iteration));
      return (1.0 - 0.1 * count / count_margin) * pow(2.0, min(10, iteration));

  } else {
    return max(0.1f, 0.2f * (1.0f - min(1.0f, (count - count_margin) / count_margin)));
    // return 0.2f;
  }
}

MRF::CostVal SmoothFunc(int lhs, int rhs, MRF::Label lhs_label, MRF::Label rhs_label) {
  if (previous_floorplan.width == kInvalid) {
    if (lhs_label == rhs_label)
      return 0.0;
    else {
      if (kDenselyConnectedMRF) {
        const int xl = lhs % frame->size[0];
        const int yl = lhs / frame->size[0];
        const int xr = rhs % frame->size[0];
        const int yr = rhs / frame->size[0];
        const int diff = max(abs(xl - xr), abs(yl - yr)) - 1;
        /*
        if (diff == 1)
          return smoothness_weight;
        else if (diff < 30)
          return smoothness_weight / 2 * (30 - diff) / 30;
        */
        if (diff < 300)
          return smoothness_weight;
        else
          return 0;
        /*
        const float sigma = 8.0;
        return smoothness_weight * exp(- diff * diff / (2 * sigma * sigma));
        */
      } else
        return smoothness_weight;
    }
  } else {
    if (lhs_label == rhs_label)
      return 0.0;
    else {
      const int xtmp = min(lhs, rhs) % frame->size[0];
      const int ytmp = min(lhs, rhs) / frame->size[0];

      if (fabs(lhs - rhs) == 1) {
        // Horizontal.
        const float horizontal_count =
          GetHorizontalCountFromMap(previous_floorplan, Vector2i(xtmp, ytmp));
        return smoothness_weight * RobustFunction(horizontal_count);
      } else if (fabs(lhs - rhs) == frame->size[0]) {
        // Vertical.
        const float vertical_count =
          GetVerticalCountFromMap(previous_floorplan, Vector2i(xtmp, ytmp));
        return smoothness_weight * RobustFunction(vertical_count);
      } else {
        return smoothness_weight;
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

  // Shrinks by count_margin.
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

  // Do not use shrunk mask
  // fill(shrunk_in_out.begin(), shrunk_in_out.end(), Floorplan::Outside);
  
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
      const int margin = kCountMapMargin + length;

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
      const int margin = kCountMapMargin + length;

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
      const int margin = kCountMapMargin + length;
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
      const int margin = kCountMapMargin + length;
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
      if (x != frame->size[0] - 1) {
        mrf->setNeighbors(index, index + 1, kUniformWeight);
        mrf->setNeighbors(index, index + 1, kUniformWeight);
      }
      // Bottom.
      if (y != frame->size[1] - 1) {
        mrf->setNeighbors(index, index + frame->size[0], kUniformWeight);
        mrf->setNeighbors(index, index + frame->size[0], kUniformWeight);
      }

      // For densely
      // bottom right.
      if (x != frame->size[0] - 1 && y != frame->size[1] - 1)
        mrf->setNeighbors(index, index + 1 + frame->size[0], kUniformWeight);
      // Bottom left.
      if (x != 0 && y != frame->size[1] - 1)
        mrf->setNeighbors(index, index - 1 + frame->size[0], kUniformWeight);

    }
  }
  
  //25 -> 50
  if (kDenselyConnectedMRF) {
    ifstream ifstr;
    ifstr.open("dense_prior.ppm");
    string stmp;
    int width, height;
    int itmp;
    ifstr >> stmp >> width >> height >> itmp;
  
    vector<int> inout(width * height);
    int count = 0;
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        ifstr >> inout[count++] >> itmp >> itmp;
    ifstr.close();

    //{ new stuff
    const bool kNew = true;
    if (kNew) {
      vector<int> horizontal_mask(width * height, 0);
      vector<int> vertical_mask(width * height, 0);
      
      // A pixel becomes a horizontal mask if there is a vertical label
      // change within kOrthogonalMargin, which must span at least
      // kParallelMargin on one side.
      const int kOrthogonalMargin = 20;
      const int kParallelMargin = 10;
      
      // horizontal
      for (int y = kOrthogonalMargin; y < height - kOrthogonalMargin; ++y) {
        for (int x = kParallelMargin; x < width - kParallelMargin; ++x) {
          // Check to right.
          bool right_good = true;
          for (int i = 0; i < kParallelMargin; ++i) {
            const int xtmp = x + i;
            // If label happens at (xtmp, y) in some y range.
            bool found = false;
            for (int j = -kOrthogonalMargin; j < kOrthogonalMargin; ++j) {
              const int ytmp = y + j;
              if (inout[ytmp * width + xtmp] != inout[(ytmp + 1) * width + xtmp]) {
                found = true;
                break;
              }
            }
            if (!found) {
              right_good = false;
              break;
            }
          }
          // Check to left.
          bool left_good = true;
          for (int i = 0; i < kParallelMargin; ++i) {
            const int xtmp = x - i;
            // If label happens at (xtmp, y) in some y range.
            bool found = false;
            for (int j = -kOrthogonalMargin; j < kOrthogonalMargin; ++j) {
              const int ytmp = y + j;
              if (inout[ytmp * width + xtmp] != inout[(ytmp + 1) * width + xtmp]) {
                found = true;
                break;
              }
            }
            if (!found) {
              left_good = false;
              break;
            }
          }
          if (right_good || left_good)
            horizontal_mask[y * width + x] = 1;
        }     
      }
      {
        ofstream ofstr;
        ofstr.open("test_hor.pgm");
        ofstr << "P2" << endl
              << width << ' ' << height << endl
              << 255 << endl;
        for (int i = 0; i < horizontal_mask.size(); ++i)
          if (horizontal_mask[i])
            ofstr << "255 " << endl;
          else
            ofstr << "0 " << endl;
        ofstr.close();
      }
      
      // vertical
      for (int x = kOrthogonalMargin; x < width - kOrthogonalMargin; ++x) {
        for (int y = kParallelMargin; y < height - kParallelMargin; ++y) {
          // Check to bottom.
          bool bottom_good = true;
          for (int i = 0; i < kParallelMargin; ++i) {
            const int ytmp = y + i;
            // If label happens at (x, ytmp) in some y range.
            bool found = false;
            for (int j = -kOrthogonalMargin; j < kOrthogonalMargin; ++j) {
              const int xtmp = x + j;
              if (inout[ytmp * width + xtmp] != inout[ytmp * width + (xtmp + 1)]) {
                found = true;
                break;
              }
            }
            if (!found) {
              bottom_good = false;
              break;
            }
          }
          // Check to top.
          bool top_good = true;
          for (int i = 0; i < kParallelMargin; ++i) {
            const int ytmp = y - i;
            // If label happens at (xtmp, y) in some y range.
            bool found = false;
            for (int j = -kOrthogonalMargin; j < kOrthogonalMargin; ++j) {
              const int xtmp = x + j;
              if (inout[ytmp * width + xtmp] != inout[ytmp * width + (xtmp + 1)]) {
                found = true;
                break;
              }
            }
            if (!found) {
              top_good = false;
              break;
            }
          }
          if (bottom_good || top_good)
            vertical_mask[y * width + x] = 1;
        }     
      }
      {
        ofstream ofstr;
        ofstr.open("test_ver.pgm");
        ofstr << "P2" << endl
              << width << ' ' << height << endl
              << 255 << endl;
        for (int i = 0; i < vertical_mask.size(); ++i)
          if (vertical_mask[i])
            ofstr << "255 " << endl;
          else
            ofstr << "0 " << endl;
        ofstr.close();
      }
      int hcount = 0;
      for (int y = 0; y < height; ++y) {
        for (int x0 = 0; x0 < width; ++x0) {
          // Try to connect (x0, y) to (x1, y).
          for (int x1 = x0 + 2; x1 < width; ++x1) {
            if (horizontal_mask[y * width + x0] &&
                horizontal_mask[y * width + x1]) {// ||
              ///              vertical_mask[y * width + x0] &&
              //              vertical_mask[y * width + x1]) {
              mrf->setNeighbors(y * width + x0, y * width + x1, 1);
              ++hcount;
            }
            else {
              break;
            }
          }
        }
      }
      int vcount = 0;
      for (int x = 0; x < width; ++x) {
        for (int y0 = 0; y0 < height; ++y0) {
          // Try to connect (x, y0) to (x, y1).
          for (int y1 = y0 + 2; y1 < height; ++y1) {
            if (vertical_mask[y0 * width + x] &&
                vertical_mask[y1 * width + x]) { // ||
              //              horizontal_mask[y0 * width + x] &&
              //              horizontal_mask[y1 * width + x]) {
              mrf->setNeighbors(y0 * width + x, y1 * width + x, 1);
              ++vcount;
            }
            else {
              break;
            }
          }
        }
      }
      cout << "hco vco " << hcount << ' ' << vcount << endl;
    } else {
      // OLD manual
      vector<int> boundary(width * height, 0);
      
      for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
          const int index = y * width + x;
          if (inout[index] != inout[index + 1] ||
              inout[index] != inout[index - 1] ||
              inout[index] != inout[index + width] ||
              inout[index] != inout[index - width])
            boundary[index] = 1;
        }
      }
      const int kExpand = 20;
      for (int t = 0; t < kExpand; ++t) {
        inout = boundary;
        for (int y = 1; y < height - 1; ++y) {
          for (int x = 1; x < width - 1; ++x) {
            const int index = y * width + x;
            if (inout[index] ||
                inout[index + 1] ||
                inout[index - 1] ||
                inout[index + width] ||
                inout[index - width])
              boundary[index] = 1;
          }
        }
      }
      
      {
        ofstream ofstr;
        ofstr.open("test.ppm");
        ofstr << "P3" << endl
              << width << ' ' << height << endl
              << 255 << endl;
        for (int i = 0; i < boundary.size(); ++i)
          if (boundary[i])
            ofstr << "255 255 255 " << endl;
          else
            ofstr << "0 0 0 " << endl;
        ofstr.close();
      }
      
      const int skip = 1;
      const int skip2 = 6;
      {
        vector<vector<int> > boxes;
        vector<int> box; // (top left) bottom right.
        box.push_back(45);
        box.push_back(20);
        box.push_back(556);
        box.push_back(65);
        boxes.push_back(box);
        
        box.clear();
        box.push_back(393);
        box.push_back(246);
        box.push_back(568);
        box.push_back(268);
        boxes.push_back(box);
        
        box.clear();
        box.push_back(21);
        box.push_back(606);
        box.push_back(400);
        box.push_back(641);
        boxes.push_back(box);
        
        for (const auto& box : boxes) {
          for (int y = box[1]; y <= box[3]; y += skip) {
            for (int x0 = box[0]; x0 <= box[2]; x0 += skip) {
              for (int x1 = x0 + skip2; x1 <= box[2]; x1 += skip2) {
                const int index0 = y * frame->size[0] + x0;
                const int index1 = y * frame->size[0] + x1;
                if (boundary[index0] && boundary[index1]) {
                  mrf->setNeighbors(index0, index1, kUniformWeight);
                }
                else
                  break;
              }
            }
          }
        }
      }
      {
        vector<vector<int> > boxes;
        vector<int> box;
        box.push_back(533);
        box.push_back(6);
        box.push_back(563);
        box.push_back(267);
        boxes.push_back(box);
        
        box.clear();
        box.push_back(17);
        box.push_back(29);
        box.push_back(80);
        box.push_back(637);
        boxes.push_back(box);
        
        box.clear();
        box.push_back(363);
        box.push_back(254);
        box.push_back(411);
        box.push_back(637);
        boxes.push_back(box);
        
        box.clear();
        box.push_back(194);
        box.push_back(610);
        box.push_back(208);
        box.push_back(638);
        boxes.push_back(box);
        
        box.clear();
        box.push_back(442);
        box.push_back(15);
        box.push_back(507);
        box.push_back(64);
        boxes.push_back(box);
        
        for (const auto& box : boxes) {
          for (int x = box[0]; x <= box[2]; x += skip) {
            for (int y0 = box[1]; y0 <= box[3]; y0 += skip) {
              for (int y1 = y0 + skip2; y1 <= box[2]; y1 += skip2) {
                const int index0 = y0 * frame->size[0] + x;
                const int index1 = y1 * frame->size[0] + x;
                if (boundary[index0] && boundary[index1]) {
                  mrf->setNeighbors(index0, index1, kUniformWeight);
                }
                else
                  break;
              }
            }
          }
        }
      }
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
  const bool kDP = true;
  if (kDP) {
    Reconstruct2DDP(frame_tmp,
                    point_evidence_tmp,
                    freespace_evidence_tmp,
                    directory_tmp,
                    floorplan_tmp);
    return;
  }

  
  frame               = &frame_tmp;
  point_evidence      = &point_evidence_tmp;
  freespace_evidence  = &freespace_evidence_tmp;
  directory           = directory_tmp;
  floorplan           = floorplan_tmp;
  
  previous_floorplan.width = kInvalid;
  previous_floorplan.height = kInvalid;

  const int kIteration = 50;
  count_margin = kMaxCountMargin;

  vector<MRF::CostVal> data_array;
  iteration = 0;
  PrepareDataArray(&data_array);
  
  DataCost data(&data_array[0]);
  SmoothnessCost smooth(&SmoothFunc);
  EnergyFunction energy(&data, &smooth);
  const int vnum = frame->size[0] * frame->size[1];
  Expansion expansion(vnum, 2, &energy);
  MRF* mrf = &expansion;

  SetNeighbors(mrf);
  
  for (iteration = 0; iteration < kIteration; ++iteration) {
    cerr << "Iteration " << iteration << endl;
    PrepareDataArray(&data_array);

    mrf->initialize();
    mrf->clearAnswer();

    cerr << "optimize" << endl;
    float t;
    mrf->optimize(1, t);
    
    cout << "Energy at end (total/data/smooth): "
         << mrf->totalEnergy() << ' '
         << mrf->dataEnergy() << ' '
         << mrf->smoothnessEnergy() << endl;

    /*
    {
      for (int y = 54; y < 60; ++y)
        for (int x = 250; x < 260; ++x)
          cout << x << ' ' << y << ' ' << mrf->getLabel(y * frame->size[0] + x) << endl;
    }
    {
      mrf->setLabel(58 * frame->size[0] + 258, 0);

      cout << "Energy at end (total/data/smooth): "
           << mrf->totalEnergy() << ' '
           << mrf->dataEnergy() << ' '
           << mrf->smoothnessEnergy() << endl;
    }
    */    
    
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
    /*
    if (iteration % 3 == 2) {
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
    */
    
    char buffer[1024];
    sprintf(buffer, "%s/%02d.ppm", directory.c_str(), iteration);
    WriteImage(buffer, *floorplan);
    /*
    sprintf(buffer, "%s/hor-%02d.ppm", directory.c_str(), iteration);
    WriteHorizontalSmoothness(buffer, *floorplan);
    sprintf(buffer, "%s/ver-%02d.ppm", directory.c_str(), iteration);
    WriteVerticalSmoothness(buffer, *floorplan);
    cerr << "all done." << endl;
    */
    //delete data;
    //delete smooth;
    //delete energy;
    //delete mrf;
    cerr << "freed." << endl;
  }
}

}  // namespace floored
