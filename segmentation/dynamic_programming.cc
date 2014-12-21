#include "dynamic_programming.h"

#include <numeric>

#include "evidence.h"

using namespace std;
using namespace Eigen;

namespace floored {

struct Edge {
  Eigen::Vector2i start;
  Eigen::Vector2i end;
  float cost;
};

struct State {
  Eigen::Vector2i previous;
  float cost;
};

bool IsIntersect(const bool adjacent,
                 const Vector2i& start0,
                 const Vector2i& end0,
                 const Vector2i& start1,
                 const Vector2i& end1) {
  bool hor_ver0, hor_ver1;
  if (start0[0] == end0[0])
    hor_ver0 = 1;
  else
    hor_ver0 = 0;

  if (start1[0] == end1[0])
    hor_ver1 = 1;
  else
    hor_ver1 = 0;

  // Both hor.
  if (hor_ver0 == 0 && hor_ver1 == 0) {
    if (start0[1] != start1[1])
      return false;
    
    if (max(start0[0], end0[0]) <= min(start1[0], end1[0]) ||
        max(start1[0], end1[0]) <= min(start0[0], end0[0]))
      return false;
    else
      return true;
  } else if (hor_ver0 == 1 && hor_ver1 == 1) {
    if (start0[0] != start1[0])
      return false;
    
    if (max(start0[1], end0[1]) <= min(start1[1], end1[1]) ||
        max(start1[1], end1[1]) <= min(start0[1], end0[1]))
      return false;
    else
      return true;
  } else if (hor_ver0 == 0 && hor_ver1 == 1) {
    if (start0 == start1 || start0 == end1 ||
        end0 == start1 || end0 == end1) {
      if (adjacent)
        return false;
      else
        return true;
    }

    if (min(start0[0], end0[0]) <= start1[0] && start1[0] <= max(start0[0], end0[0]) &&
        min(start1[1], end1[1]) <= start0[1] && start0[1] <= max(start1[1], end1[1]))
      return true;
    
    return false;
  } else {
    if (start0 == start1 || start0 == end1 ||
        end0 == start1 || end0 == end1) {
      if (adjacent)
        return false;
      else
        return true;
    }

    if (min(start1[0], end1[0]) <= start0[0] && start0[0] <= max(start1[0], end1[0]) &&
        min(start0[1], end0[1]) <= start1[1] && start1[1] <= max(start0[1], end0[1]))
      return true;
    
    return false;
  }
}


void SetActiveArea(vector<int>* active) {
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

  active->clear();
  active->resize(width * height, 0);
      
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      const int index = y * width + x;
      if (inout[index] != inout[index + 1] ||
          inout[index] != inout[index - 1] ||
          inout[index] != inout[index + width] ||
          inout[index] != inout[index - width])
        active->at(index) = 1;
    }
  }
  const int kExpand = 30;
  for (int t = 0; t < kExpand; ++t) {
    inout = *active;
    for (int y = 1; y < height - 1; ++y) {
      for (int x = 1; x < width - 1; ++x) {
        const int index = y * width + x;
        if (inout[index] ||
            inout[index + 1] ||
            inout[index - 1] ||
            inout[index + width] ||
            inout[index - width])
          active->at(index) = 1;
      }
    }
  }
  
  {
    ofstream ofstr;
    ofstr.open("test.ppm");
    ofstr << "P3" << endl
          << width << ' ' << height << endl
          << 255 << endl;
    for (int i = 0; i < active->size(); ++i)
      if (active->at(i))
        ofstr << "255 255 255 " << endl;
      else
        ofstr << "0 0 0 " << endl;
    ofstr.close();
  }
}

float ComputeEdgeCost(const Vector2i& pos,
                      const Vector2i& free_space_direction,
                      const Vector2i& size,
                      const std::vector<float>& point_evidence,
                      const std::vector<float>& free_space_evidence) {
  float cost = - ScalePointEvidence(point_evidence[pos[1] * size[0] + pos[0]]);
  // Must be free space.
  const int kSmallMargin = 1;
  const int kLargeMargin = 20;

  for (int i = kSmallMargin; i < kLargeMargin; ++i) {
    const Vector2i must_be_free_space_pos = pos + i * free_space_direction;
    if (0 <= must_be_free_space_pos[0] && must_be_free_space_pos[0] < size[0] &&
        0 <= must_be_free_space_pos[1] && must_be_free_space_pos[1] < size[1]) {
      const int index = must_be_free_space_pos[1] * size[0] + must_be_free_space_pos[0];
      cost -= ScaleFreeSpaceEvidence(free_space_evidence[index]);
    }
  }

  for (int i = kSmallMargin; i < kLargeMargin; ++i) {
    const Vector2i must_not_be_free_space_pos = pos - i * free_space_direction;
    if (0 <= must_not_be_free_space_pos[0] && must_not_be_free_space_pos[0] < size[0] &&
        0 <= must_not_be_free_space_pos[1] && must_not_be_free_space_pos[1] < size[1]) {
      const int index = must_not_be_free_space_pos[1] * size[0] + must_not_be_free_space_pos[0];
      cost += 4.0 * ScaleFreeSpaceEvidence(free_space_evidence[index]);
    }
  }

  return cost;  
}
  
void Reconstruct2DDP(const Frame& frame,
                     const std::vector<float>& point_evidence,
                     const std::vector<float>& free_space_evidence,
                     const std::string& directory,
                     Floorplan* floorplan) {
  vector<int> active;
  SetActiveArea(&active);
  
  const int width  = frame.size[0];
  const int height = frame.size[1];
  vector<vector<Edge> > edges(width * height);

  {
    const float kModelComplexity = 5.0;
    const float kPointEvidenceThreshold = 5.0;

    cerr << "Adding edges..." << endl;
    int count = 0;


    int pos = 0;
    int neg = 0;
    // Right, bottom, left, top.
    for (int y = 1; y < height - 1; ++y) {
      for (int x = 1; x < width - 1; ++x) {
        const int index = y * width + x;
        
        if (!active[index])
          continue;
        
        // Right.
        {
          float accumulated_cost = 0.0;
          for (int xtmp = x + 1; xtmp < width - 1; ++xtmp) {
            const int index_tmp = y * width + xtmp;
            if (!active[index_tmp])
              break;
            
            Edge edge;
            edge.start = Vector2i(x, y);
            edge.end   = Vector2i(xtmp, y);
            accumulated_cost +=
              ComputeEdgeCost(Vector2i(xtmp, y), Vector2i(0, -1), Vector2i(width, height),
                              point_evidence, free_space_evidence);
            /*ScaleFreeSpaceEvidence(free_space_evidence[index_tmp]) / 4.0 -
              ScalePointEvidence(point_evidence[index_tmp]);*/
            //ScalePointEvidence(point_evidence[index_tmp]) < kPointEvidenceThreshold ?
            //1.0f : 0.0f;
            edge.cost = accumulated_cost + kModelComplexity;
            if (edge.cost > 0)
              ++pos;
            else
              ++neg;
            edges[index].push_back(edge);
            ++count;
          }
        }
        
        // Bottom.
        {
          float accumulated_cost = 0.0;
          for (int ytmp = y + 1; ytmp < height - 1; ++ytmp) {
            const int index_tmp = ytmp * width + x;
            if (!active[index_tmp])
              break;
            
            Edge edge;
            edge.start = Vector2i(x, y);
            edge.end   = Vector2i(x, ytmp);
            accumulated_cost +=
              ComputeEdgeCost(Vector2i(x, ytmp), Vector2i(1, 0), Vector2i(width, height),
                              point_evidence, free_space_evidence);
            //              ScaleFreeSpaceEvidence(free_space_evidence[index_tmp]) / 4.0 -
            //              ScalePointEvidence(point_evidence[index_tmp]);
            //ScalePointEvidence(point_evidence[index_tmp]) < kPointEvidenceThreshold ?
            // 1.0f : 0.0f;
            edge.cost = accumulated_cost + kModelComplexity;
            if (edge.cost > 0)
              ++pos;
            else
              ++neg;
            
            edges[index].push_back(edge);
            ++count;
          }
        }
        
        // Left
        {
          float accumulated_cost = 0.0;
          for (int xtmp = x - 1; xtmp > 0; --xtmp) {
            const int index_tmp = y * width + xtmp;
            if (!active[index_tmp])
              break;
            
            Edge edge;
            edge.start = Vector2i(x, y);
            edge.end   = Vector2i(xtmp, y);
            accumulated_cost +=
              ComputeEdgeCost(Vector2i(xtmp, y), Vector2i(0, 1), Vector2i(width, height),
                              point_evidence, free_space_evidence);
            //ScaleFreeSpaceEvidence(free_space_evidence[index_tmp]) / 4.0 -
            //ScalePointEvidence(point_evidence[index_tmp]);
            //ScalePointEvidence(point_evidence[index_tmp]) < kPointEvidenceThreshold ?
            //1.0f : 0.0f;
            edge.cost = accumulated_cost + kModelComplexity;
            if (edge.cost > 0)
              ++pos;
            else
              ++neg;
            
            edges[index].push_back(edge);
            ++count;
          }
        }
        
        // Top.
        {
          float accumulated_cost = 0.0;
          for (int ytmp = y - 1; ytmp > 0; --ytmp) {
            const int index_tmp = ytmp * width + x;
            if (!active[index_tmp])
              break;
            
            Edge edge;
            edge.start = Vector2i(x, y);
            edge.end   = Vector2i(x, ytmp);
            accumulated_cost +=
              ComputeEdgeCost(Vector2i(x, ytmp), Vector2i(-1, 0), Vector2i(width, height),
                              point_evidence, free_space_evidence);
            //ScaleFreeSpaceEvidence(free_space_evidence[index_tmp]) / 4.0 -
            //ScalePointEvidence(point_evidence[index_tmp]);
            //ScalePointEvidence(point_evidence[index_tmp]) < kPointEvidenceThreshold ?
            //1.0f : 0.0f;
            edge.cost = accumulated_cost + kModelComplexity;
            if (edge.cost > 0)
              ++pos;
            else
              ++neg;
            
            edges[index].push_back(edge);
            ++count;
          }
        }
      }
    }
    cout << count << " over " << width * height << " pixels." << endl;
    cout << "pos neg " << pos << ' ' << neg << endl;
  }  
  // DP starts.
  // 34, 336 starts going down. Cannot cross y = 336 from x = [0 70].

  // Store cost and previous node location.
  const float kInvalid = numeric_limits<float>::max();

  // Cost at each pixel and for each number of length of path.
  const int kMaxPathLength = 32;
  vector<vector<State> > states(width * height);
  for (int i = 0; i < width * height; ++i) {
    states[i].resize(kMaxPathLength);
    for (int j = 0; j < kMaxPathLength; ++j)
      states[i][j].cost = kInvalid;
  }

  // First state.
  const int start_index = 336 * width + 34;
  const int goal_index = 335 * width + 34;
  const int kInitialPathLength = 0;
  states[start_index][kInitialPathLength].cost = 0.0;
  states[start_index][kInitialPathLength].previous = Vector2i(34, 335);

  // Start. Find a node with valid cost at path length = t.
  // Then, fill out node with path length t + 1.
  for (int t = 0; t < kMaxPathLength - 1; ++t) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int index = y * width + x;
        if (states[index][t].cost != kInvalid) {
          // Use all the edges to update.
          for (const auto& edge : edges[index]) {
            /*
            // Avoid too short.
            if (max(abs(edge.start[0] - edge.end[0]),
                    abs(edge.start[1] - edge.end[1])) < 8)
              continue;
            */
            // Exceptional case hacks.
            // Hack for one dataset. First move must be vertical.
            /*
            if (t == 0 && edge.start[1] == edge.end[1])
              continue;
            */
            // Check if an edge crosses a line (0, 335.5)-(70,336).
            if (min(edge.start[1], edge.end[1]) < 335.5 &&
                max(edge.start[1], edge.end[1]) > 335.5 &&
                edge.start[0] >= 0 &&
                edge.start[0] <= 70) {
              // Intersected.
              continue;
            }

            // Not intersected.
            const int next_index = edge.end[1] * width + edge.end[0];
            const float new_cost = states[index][t].cost + edge.cost;


            // Check if an edge intersects with a previous pass.
            {
              bool intersect = false;
              Vector2i current(index % width, index / width);
              int current_index = current[1] * width + current[0];
              for (int current_p = t; current_p >= 0; --current_p) {
                if (IsIntersect(current_p == t, edge.start, edge.end,
                                current, states[current_index][current_p].previous)) {
                  intersect = true;
                  break;
                }

                current = states[current_index][current_p].previous;
                current_index = current[1] * width + current[0];
              }

              if (intersect)
                continue;
            }

            // Hack of one dataset. Last move must be vertical.
            /*
            if (next_index == goal_index && edge.start[1] == edge.end[1])
              continue;
            */
            if (new_cost < states[next_index][t + 1].cost) {
              states[next_index][t + 1].previous = edge.start;
              states[next_index][t + 1].cost = new_cost;
            }
          }
        }
      }
    }
  }

  // Now your solution is at states[goal_index]...
  for (int p = 0; p < states[goal_index].size(); ++p) {
    if (states[goal_index][p].cost == kInvalid)
      continue;
    
    char buffer[1024];
    sprintf(buffer, "%s/result_%02d.svg", directory.c_str(), p);
    ofstream ofstr;
    ofstr.open(buffer);
    ofstr << "<svg xmlns=\"http://www.w3.org/2000/svg\"" << endl
          << "xmlns:xlink=\"http//www.w3.org/1999/xlink\">" << endl
          << "<rect x=\"0\" y=\"0\" width=\""
          << width << "\" height=\"" << height << "\" style=\"fill: #FFFFFF\"/>" << endl;

    Vector2i current(goal_index % width, goal_index / width);
    int current_index = current[1] * width + current[0];

    cout << "PATH: " << p << endl;
    for (int current_p = p; current_p >= 0; --current_p) {
      ofstr << "<line x1=\"" << current[0] << "\" y1=\"" << current[1]
            << "\" x2=\"" << states[current_index][current_p].previous[0]
            << "\" y2=\"" << states[current_index][current_p].previous[1]
            << "\" style=\"stroke: black; stroke-width: 4;\"/>" << endl;

      cout << current[0] << ' ' << current[1]
           << " -> " << states[current_index][current_p].previous[0]
           << ' ' << states[current_index][current_p].previous[1] << "   "
           << states[current_index][current_p].cost << endl;
      
      current = states[current_index][current_p].previous;
      current_index = current[1] * width + current[0];
    }
    cout << endl;
    ofstr << "</svg>" << endl;    
    ofstr.close();
  }
}  
  
}  // namespace floored
  
