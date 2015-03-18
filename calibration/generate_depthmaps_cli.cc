#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <limits>
#include <vector>

#include <gflags/gflags.h>

#include "../base/panorama.h"
#include "../base/point_cloud.h"

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

DEFINE_int32(depthmap_shrink_ratio, 8, "8 times smaller.");

void SmoothField(const int width,
                 const int height,
                 const double hole,
                 std::vector<double>* field) {
  const double kDataWeight = 4.0;
  SparseMatrix<double> A(width * height, width * height);
  VectorXd b(width * height);

  vector<Eigen::Triplet<double> > triplets;
  
  int index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      int count = 0;
      if (x != 0) {
        triplets.push_back(Eigen::Triplet<double>(index, index - 1, -1));
        ++count;
      }
      if (x != width - 1) {
        triplets.push_back(Eigen::Triplet<double>(index, index + 1, -1));
        ++count;
      }
      if (y != 0) {
        triplets.push_back(Eigen::Triplet<double>(index, index - width, -1));
        ++count;
      }
      if (y != height - 1) {
        triplets.push_back(Eigen::Triplet<double>(index, index + width, -1));
        ++count;
      }

      if (field->at(index) == hole) {
        triplets.push_back(Eigen::Triplet<double>(index, index, count));
        b[index] = 0;
      } else {
        triplets.push_back(Eigen::Triplet<double>(index, index, kDataWeight + count));
        b[index] = kDataWeight * field->at(index);
      }
    }
  }
  A.setFromTriplets(triplets.begin(), triplets.end());
  
  SimplicialCholesky<SparseMatrix<double> > chol(A);
  VectorXd x = chol.solve(b);

  for (int i = 0; i < (int)field->size(); ++i) {
    field->at(i) = x[i];
  }
}  

void WriteDepthmapData(const FileIO& file_io,
                       const int panorama,
                       const int depth_width,
                       const int depth_height,
                       const std::vector<double>& depthmap) {
  const double kInvalid = -1.0;
  double min_distance, max_distance;
  bool first = true;
  for (const auto value : depthmap) {
    if (value != kInvalid) {
      if (first) {
        min_distance = value;
        max_distance = value;
        first = false;
      } else {
        min_distance = min(min_distance, value);
        max_distance = max(max_distance, value);
      }
    }
  }
  
  {
    ofstream ofstr;
    ofstr.precision(5);
    ofstr.open(file_io.GetDepthPanorama(panorama));
    ofstr << "Depth" << endl
          << depth_width << ' ' << depth_height << endl
          << min_distance << ' ' << max_distance << endl;
    for (const auto value : depthmap) {
      ofstr << value << ' ';
    }
    ofstr.close();
  }
  {
    cv::Mat depth_image(depth_height, depth_width, CV_8UC3);
    int index = 0;
    for (int y = 0; y < depth_height; ++y) {
      for (int x = 0; x < depth_width; ++x, ++index) {
        if (depthmap[index] == kInvalid) {
          depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
        } else {
          const int itmp =
            static_cast<int>(round(255.0 *
                                   (depthmap[index] - min_distance) / (max_distance - min_distance)));
          depth_image.at<cv::Vec3b>(y, x) = cv::Vec3b(itmp, itmp, itmp);
        }
      }
    }
    imwrite(file_io.GetDepthVisualization(panorama), depth_image);
  }
} 

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
#ifdef __APPLE__
  google::ParseCommandLineFlags(&argc, &argv, true);
#else
  gflags::ParseCommandLineFlags(&argc, &argv, true);
#endif

  FileIO file_io(argv[1]);

  vector<Panorama> panoramas;
  ReadPanoramasWithoutDepths(file_io, &panoramas);

  vector<PointCloud> point_clouds;
  ReadPointClouds(file_io, &point_clouds);

  if (panoramas.size() != point_clouds.size()) {
    cerr << "Impossible: " << panoramas.size() << ' ' << point_clouds.size() << endl;
    exit (1);
  }
  
  for (int p = 0; p < (int)panoramas.size(); ++p) {
    const Panorama& panorama = panoramas[p];
    const PointCloud& point_cloud = point_clouds[p];    
    const int depth_width =
      panorama.Width() / FLAGS_depthmap_shrink_ratio;
    const int depth_height =
      panorama.Height() / FLAGS_depthmap_shrink_ratio;
    
    const double kInvalid = -1.0;
    vector<double> depthmap(depth_width * depth_height, kInvalid);

    for (int q = 0; q < point_cloud.GetNumPoints(); ++q) {
      const Point& point = point_cloud.GetPoint(q);
      const Vector2d pixel = panorama.Project(point.position);
      const int x = static_cast<int>(round(pixel[0] * depth_width / panorama.Width()));
      const int y = static_cast<int>(round(pixel[1] * depth_height / panorama.Height()));
                                           
      if (0 <= x && x < depth_width && 0 <= y && y < depth_height) {
        const double distance = (panorama.GetCenter() - point.position).norm();
        const int index = y * depth_width + x;
        if (depthmap[index] == kInvalid ||
            distance < depthmap[index]) {
          depthmap[index] = distance;
        }
      }
    }
    // Laplacian smoothing.
    SmoothField(depth_width, depth_height, kInvalid, &depthmap);

    WriteDepthmapData(file_io, p, depth_width, depth_height, depthmap);
  }
  
  return 0;
}
