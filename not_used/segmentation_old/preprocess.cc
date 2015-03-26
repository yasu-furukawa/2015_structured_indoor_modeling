#include "preprocess.h"

#include <fstream>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

namespace floored {

namespace {

void SetNormals(const int width, const int height, vector<OrientedPoint>* points) {
  // Computes normal.
  for (int x = 0; x < width; ++x) {
    const int left_x = (x - 1 + width) % width;
    const int right_x = (x + 1) % width;
    for (int y = 0; y < height; ++y) {       
      const int index        = x * height + y;
      const int left_index   = left_x * height + y;
      const int right_index  = right_x * height + y;

      const Vector3f center_to_left = points->at(left_index).position -
        points->at(index).position;
      const Vector3f center_to_right = points->at(right_index).position -
        points->at(index).position;

      Vector3f& normal = points->at(index).normal;
      normal = Vector3f(0, 0, 0);
      // Use 4 triangles to accumulate the normal.
      const int top_y    = y - 1;
      if (0 <= top_y) {
        const int top_index    = x * height + top_y;
        const Vector3f center_to_top = points->at(top_index).position -
          points->at(index).position;
        
        normal += center_to_left.cross(center_to_top).normalized();
        normal += center_to_top.cross(center_to_right).normalized();
      }
      const int bottom_y = y + 1;
      if (bottom_y < height) {
        const int bottom_index = x * height + bottom_y;
        const Vector3f center_to_bottom = points->at(bottom_index).position -
          points->at(index).position;
        
        normal += center_to_right.cross(center_to_bottom).normalized();
        normal += center_to_bottom.cross(center_to_left).normalized();
      }
      normal.normalize();
      
      if (normal.dot(points->at(index).position) > 0.0) {
        normal = -normal;
      }
    }
  }
}

void BlurNormals(const int width, const int height, vector<OrientedPoint>* points) {
  // Blur normals.
  vector<Vector3f> blurred_normals(width * height);
  for (int x = 0; x < width; ++x) {
    const int left_x = (x - 1 + width) % width;
    const int right_x = (x + 1) % width;
    for (int y = 0; y < height; ++y) {
      const int index        = x * height + y;
      const int left_index   = left_x * height + y;
      const int right_index  = right_x * height + y;
      blurred_normals[index] = points->at(index).normal;
      blurred_normals[index] += points->at(left_index).normal;
      blurred_normals[index] += points->at(right_index).normal;
      
      const int top_y    = y - 1;
      if (0 <= top_y) {
        const int top_index       = x * height + top_y;
        const int top_left_index  = (left_x * height + top_y);
        const int top_right_index = (right_x * height + top_y);
        blurred_normals[index] += points->at(top_index).normal;
        blurred_normals[index] += points->at(top_left_index).normal;
        blurred_normals[index] += points->at(top_right_index).normal;
      }
      
      const int bottom_y = y + 1;
      if (bottom_y < height) {
        const int bottom_index = x * height + bottom_y;
        const int bottom_left_index  = (left_x * height + bottom_y);
        const int bottom_right_index = (right_x * height + bottom_y);
        blurred_normals[index] += points->at(bottom_index).normal;
        blurred_normals[index] += points->at(bottom_left_index).normal;
        blurred_normals[index] += points->at(bottom_right_index).normal;
      }
      
      blurred_normals[index].normalize();
    }
  }
  for (int i = 0; i < width * height; ++i) {
    points->at(i).normal = blurred_normals[i];
  }
}

void SetRanges(const vector<OrientedPoint>& points,
               const float average_distance,
               Frame* frame) {
  // Do not take into account point weights.
  vector<double> histogram[3];

  for (const auto& point : points) {
    for (int a = 0; a < 3; ++a) {
      histogram[a].push_back(point.position.dot(frame->axes[a]));
    }
  }

  for (int a = 0; a < 3; ++a) {
    sort(histogram[a].begin(), histogram[a].end());

    // Take the 5 and 95 percentiles.
    const auto pos_5  = histogram[a].begin() + (histogram[a].size() * 5 / 100);
    const auto pos_95 = histogram[a].begin() + (histogram[a].size() * 95 / 100);

    nth_element(histogram[a].begin(), pos_5,  histogram[a].end());
    nth_element(histogram[a].begin(), pos_95, histogram[a].end());

    const double diff = (*pos_95) - (*pos_5);
    // Allow a margin of 5 percents.
    const double margin = diff * 5 / 100;

    frame->ranges[a][0] = (*pos_5)  - margin;
    frame->ranges[a][1] = (*pos_95) + margin;
  }

  //----------------------------------------------------------------------
  // Initial guess of unit.
  //???????
  double unit = average_distance / 200.0;
  // Compute resolution.
  const int width  = static_cast<int>(round((frame->ranges[0][1] - frame->ranges[0][0]) / unit));
  const int height = static_cast<int>(round((frame->ranges[1][1] - frame->ranges[1][0]) / unit));
  // const int depth  = static_cast<int>(round((frame->max_z - frame->min_z) / unit));

  const int kMaxResolution = 1024;
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

void WritePointAsPly(const std::vector<OrientedPoint>& points,
                     const std::string filename) {
  ofstream ofstr;
  ofstr.open(filename.c_str());

  ofstr << "ply" << endl
        << "format ascii 1.0" << endl
        << "element vertex " << points.size() << endl
        << "property float x" << endl
        << "property float y" << endl
        << "property float z" << endl
        << "property float nx" << endl
        << "property float ny" << endl
        << "property float nz" << endl
        << "end_header" << endl;
  for (const auto& point : points) {
    ofstr << point.position[0] << ' '
          << point.position[1] << ' '
          << point.position[2] << ' '
          << point.normal[0] << ' '
          << point.normal[1] << ' '
          << point.normal[2] << endl;
  }
  ofstr.close();
}

  
}  // namespace

void ReadSweeps(const string directory, const int num,
                vector<Sweep>* sweeps) {
  sweeps->clear();
  sweeps->resize(num);
  cerr << "Reading points... " << flush;
  for (int s = 1; s <= num; ++s) {
    cerr << s << ' ' << flush;
    const int array_index = s - 1;
    char buffer[1024];
    sprintf(buffer, "%s/sweeps/%03d/sweep.xyz", directory.c_str(), s);

    ifstream ifstr;
    ifstr.open(buffer);
    if (!ifstr.is_open()) {
      cerr << "Sweep file does not exist: " << buffer;
      exit (1);
    }

    sweeps->at(array_index).center = Vector3f(0, 0, 0);
    vector<OrientedPoint>& points = sweeps->at(array_index).points;

    string header;
    int width, height;
    ifstr >> header >> width >> header >> height;
    points.resize(width * height);
    int index = 0;
    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y, ++index) {
        ifstr >> points[index].position[0]
              >> points[index].position[1]
              >> points[index].position[2];

        // Compute angle at y, where 0 corresponds to the horizontal..
        const double angle = M_PI * (y + 1) / (height + 1) - M_PI / 2.0;
        points[index].weight = cos(angle);
      }
    }

    ifstr.close();

    SetNormals(width, height, &points);
    BlurNormals(width, height, &points);
  }
 
  cerr << endl;
}

void FilterSweeps(const float lower_height,
                  const float upper_height,
                  std::vector<Sweep>* sweeps) {
  vector<Sweep> filtered_sweeps;
  for (const auto& sweep : (*sweeps)) {
    Sweep filtered_sweep;
    filtered_sweep.center = sweep.center;
    for (const auto& point : sweep.points) {
      if (lower_height <= point.position[2] &&
          point.position[2] <= upper_height) {
        filtered_sweep.points.push_back(point);
      }
    }
    filtered_sweeps.push_back(filtered_sweep);    
  }
  sweeps->swap(filtered_sweeps);
}

void WriteSweepsAsPly(const std::vector<Sweep>& sweeps,
                        const std::string filename) {
  int point_num = 0;
  for (const auto& sweep : sweeps) {
    point_num += sweep.points.size();
  }

  ofstream ofstr;
  ofstr.open(filename.c_str());

  ofstr << "ply" << endl
        << "format ascii 1.0" << endl
        << "element vertex " << point_num << endl
        << "property float x" << endl
        << "property float y" << endl
        << "property float z" << endl
        << "property float nx" << endl
        << "property float ny" << endl
        << "property float nz" << endl
        << "end_header" << endl;

  for (const auto& sweep : sweeps) {
    //ofstr << sweep.center[0] << ' ' << sweep.center[1] << ' ' << sweep.center[2]
    //<< " 0 0 1" << endl;
    for (const auto& point : sweep.points) {
      ofstr << point.position[0] << ' '
            << point.position[1] << ' '
            << point.position[2] << ' '
            << point.normal[0] << ' '
            << point.normal[1] << ' '
            << point.normal[2] << endl;
    }
  }
  ofstr.close();
}

void ComputeFrame(const std::string directory,
                  const vector<Sweep>& sweeps,
                  const float average_distance,
                  Frame* frame) {
  frame->axes[2] = Vector3f(0, 0, 1);

  const int kIteration = 20000;
  const float kThreshold = 4.0;
  vector<OrientedPoint> inliers;
  cerr << "Ransac for x-axis..." << flush;

  vector<OrientedPoint> points;
  for (const auto& sweep : sweeps) {
    points.insert(points.end(), sweep.points.begin(), sweep.points.end());
  }
  
  const double kDotProductThreshold = cos(67.5 * M_PI / 180.0);
  vector<OrientedPoint> points_with_valid_normal;
  for (const auto& point : points) {
    if (fabs(frame->axes[2].dot(point.normal)) < kDotProductThreshold) {
      points_with_valid_normal.push_back(point);
    }
  }

  RansacPlane(points_with_valid_normal, frame->axes[2], kIteration, kThreshold,
              &frame->axes[0], &inliers);
  cerr << "done." << endl;

  frame->axes[1] = -frame->axes[0].cross(frame->axes[2]);

  char buffer[1024];
  sprintf(buffer, "%s/x_axis.ply", directory.c_str());
  WritePointAsPly(inliers, buffer);

  // Set frame and apply ToLocal.
  SetRanges(points, average_distance, frame);
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
    exit (1);
  }
  
  return average_distance / count;
}
 
void RansacPlane(const std::vector<OrientedPoint>& points,
                 const Eigen::Vector3f& axis,
                 const int iteration,
                 const float threshold,
                 Eigen::Vector3f* orthogonal_axis,
                 std::vector<OrientedPoint>* inliers) {
  // const double kDotProductThreshold = cos(22.5 * M_PI / 180.0);

  // Do not take into account point weights in ransac.
  // Normals must be similar.
  inliers->clear();
  float inlier_weight = 0.0;
  int last_i = -1;
  for (int i = 0; i < iteration; ++i) {
    if (i != last_i) {
      if (i % 100 == 0)
        cerr << i << ' ' << flush;
      last_i = i;
    }
    // Pick 3 points.
    const int id0 = rand() % points.size();

    // Collect ids that are on the same plane.
    vector<int> ids;
    for (int p = 0; p < points.size(); ++p) {
      //      if (fabs((points[p].position - points[id0].position).dot(points[id0].normal))
      //          < threshold) {
      ids.push_back(p);
      //      }
    }
    if (ids.size() < 2) {
      --i;
      continue;
    }
    
    const int id1 = ids[rand() % ids.size()];
    const int id2 = ids[rand() % ids.size()];
    if (id0 == id1 || id0 == id2 || id1 == id2) {
      --i;
      continue;
    }
    // Check that normals are similar.
    /*
    if (points[id0].normal.dot(points[id1].normal) < kDotProductThreshold ||
        points[id1].normal.dot(points[id2].normal) < kDotProductThreshold ||
        points[id2].normal.dot(points[id0].normal) < kDotProductThreshold) {
      --i;
      continue;
    }
    */

    Vector3f orthogonal =
      (points[id1].position - points[id0].position).cross(points[id2].position - points[id0].position);
    orthogonal.normalize();


    //????
    if (fabs(orthogonal.dot(axis)) > cos(85.0 * M_PI / 180.0))
      continue;      
    
    vector<OrientedPoint> inliers_tmp;
    float inlier_weight_tmp = 0.0;
    for (const auto& point : points) {
      const double displacement =
        fabs(orthogonal.dot(points[id0].position - point.position));
      if (displacement < threshold) {
        inliers_tmp.push_back(point);
        inlier_weight_tmp += point.weight;
      }
    }
    // if (inliers_tmp.size() > inliers->size()) {
    if (inlier_weight_tmp > inlier_weight) {
      *orthogonal_axis = orthogonal;
      inliers->swap(inliers_tmp);
      inlier_weight = inlier_weight_tmp;
    }
  }
}
 
}  // namespace floored
