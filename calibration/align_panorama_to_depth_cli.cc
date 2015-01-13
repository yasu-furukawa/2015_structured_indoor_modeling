#include <cstdio>
#include <iostream>
#include <fstream>
#include <numeric>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "align_images.h"
#include "ceres/ceres.h"
#include "depthmap_refiner.h"
#include "../base/file_io.h"
#include "gflags/gflags.h"
#include "transformation.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace structured_indoor_modeling;

DEFINE_int32(num_pyramid_levels, 3, "Num pyramid levels.");
DEFINE_int32(start_panorama, 0, "Start panorama index.");
DEFINE_int32(end_panorama, 1, "End panorama index (exclusive).");
DEFINE_int32(ncc_window_radius, 2, "ncc window radius");
DEFINE_bool(overwrite, false, "overwrite result");

const double kInvalid = -1.0;

struct DepthPoint {
  int x;
  int y;
  double X;
  double Y;
  double Z;
  double distance;
};

struct Image {
  vector<Vector3d> color;
  vector<double> depth;
  
  vector<double> edge;
  int width;
  int height;
};

void ReadPly(const string filename,
             int* depth_width,
             int* depth_height,
             vector<DepthPoint>* depth_points) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  if (!ifstr.is_open()) {
    cerr << "ply file does not exist: " << filename << endl;
    exit (1);
  }

  string stmp;
  for (int i = 0; i < 6; ++i)
    ifstr >> stmp;
  int num_vertex;
  ifstr >> num_vertex;
  for (int i = 0; i < 37; ++i)
    ifstr >> stmp;
    
  const int kXOffset = 1;
  const int kYOffset = 1;
  
  int max_x = 0, max_y = 0;

  depth_points->resize(num_vertex);
  for (int i = 0; i < num_vertex; ++i) {
    ifstr >> depth_points->at(i).y >> depth_points->at(i).x
          >> depth_points->at(i).X >> depth_points->at(i).Y >> depth_points->at(i).Z;
      
    depth_points->at(i).x -= kXOffset;
    depth_points->at(i).y -= kYOffset;
    double dummy;
    for (int j = 0; j < 7; ++j)
      ifstr >> dummy;
    
    const double distance = sqrt(depth_points->at(i).X * depth_points->at(i).X +
                                 depth_points->at(i).Y * depth_points->at(i).Y +
                                 depth_points->at(i).Z * depth_points->at(i).Z);
    depth_points->at(i).distance = distance;
      
    if (i == 0) {
      max_x = depth_points->at(i).x;
      max_y = depth_points->at(i).y;
    } else {
      max_x = max(depth_points->at(i).x, max_x);
      max_y = max(depth_points->at(i).y, max_y);
    }
  }
  ifstr.close();

  *depth_width = max_x + 1;
  *depth_height = max_y + 1;
}

void SetDepthImage(const int depth_width,
                   const int depth_height,
                   const vector<DepthPoint>& depth_points,
                   const double depth_phi_range,
                   vector<double>* depth_image) {
  const double depth_phi_per_pixel = depth_phi_range / depth_height;
  depth_image->clear();
  depth_image->resize(depth_width * depth_height, kInvalid);
  for (const auto& depth_point : depth_points) {
    Vector2d uv;
    ConvertLocalToPanorama(depth_width, depth_height, depth_phi_per_pixel,
                           Vector3d(depth_point.X, depth_point.Y, depth_point.Z),
                           &uv);
    const int x = static_cast<int>(round(uv[0])) % depth_width;
    const int y = static_cast<int>(round(uv[1]));
    if (y < 0 || depth_height <= y)
      continue;

    depth_image->at(y * depth_width + x) = depth_point.distance;
  }
}

void DetectEdgeDepth(Image* depth_image) {
  const int width  = depth_image->width;
  const int height = depth_image->height;
  const vector<double>& depth = depth_image->depth;  
  vector<double>* edge = &depth_image->edge;
  
  edge->clear();
  edge->resize(width * height, kInvalid);

  for (int y = 1; y < height - 1; ++y) {
    for (int x = 0; x < width; ++x) {
      const int right_x = (x + 1) % width;
      const int left_x = (x - 1 + width) % width;
      if (depth[y * width + x] == kInvalid)
        continue;
      
      double negative = 0.0;
      int denom = 0;
      if (depth[(y - 1) * width + x] != kInvalid) {
        negative += depth[(y - 1) * width + x];
        ++denom;
      }
      if (depth[(y + 1) * width + x] != kInvalid) {
        negative += depth[(y + 1) * width + x];
        ++denom;
      }
      if (depth[y * width + left_x] != kInvalid) {
        negative += depth[y * width + left_x];
        ++denom;
      }
      if (depth[y * width + right_x] != kInvalid) {
        negative += depth[y * width + right_x];
        ++denom;
      }
      if (denom == 4) {
        edge->at(y * width + x) =
          fabs(depth[y * width + x] - negative / denom);
      }
    }
  }

  double mean = 0.0, variance = 0.0;
  int denom = 0;
  for (const auto& value : (*edge)) {
    if (value == kInvalid)
      continue;
    mean += value;
    variance += value * value;
    ++denom;
  }
  if (denom == 0) {
    cerr << "Impossible0." << endl;
    exit (1);
  }
  mean /= denom;
  variance /= denom;
  const double deviation = sqrt(variance - mean * mean);
  
  for (auto& value : (*edge)) {
    if (value == kInvalid)
      continue;
    // value = max(0.0, min(1.0, (value - mean + deviation / 2.0) / (2 * deviation)));
    value = max(0.0, min(1.0, (value - (mean - deviation / 2.0)) / (deviation)));
  }
}  

/*
Vector2d PanoramaToDepth(const int panorama_width,
                         const int panorama_height,
                         const int depth_width,
                         const int depth_height,
                         const double depth_phi_per_pixel,
                         const double* const params,
                         const Vector2d& panorama_pixel) {
  const double panorama_phi_per_pixel = params[0] / panorama_height;
  const Matrix3d rx = RotationX(params[1]);
  const Matrix3d rz = RotationZ(params[2]);
  const Matrix3d ry = RotationY(params[3]);
  const Vector3d t(params[4], params[5], params[6]);
  const Matrix3d r = ry * rz * rx;
  Vector3d ray;
  ConvertPanoramaToLocal(panorama_width, panorama_height,
                         panorama_phi_per_pixel, panorama_pixel, &ray);
  ray = r * ray + t;
  
  Vector2d uv;
  ConvertLocalToPanorama(depth_width, depth_height, depth_phi_per_pixel, ray, &uv);
  return uv;
}
*/

Vector2d DepthToPanorama(const int color_width,
                         const int color_height,
                         const int depth_width,
                         const int depth_height,
                         const double depth_phi_per_pixel,
                         const double* const params,
                         const Vector2d& depth_pixel,
                         const double depth) {
  const double color_phi_per_pixel = params[0] / color_height;
  const Matrix3d rx = RotationX(params[1]);
  const Matrix3d rz = RotationZ(params[2]);
  const Matrix3d ry = RotationY(params[3]);
  const Vector3d t(params[6], params[5], params[4]);
  const Matrix3d r = ry * rz * rx;
  Vector3d ray;
  ConvertPanoramaToLocal(depth_width, depth_height,
                         depth_phi_per_pixel, depth_pixel, &ray);
  ray.normalize();
  ray *= depth;
  
  ray = r * ray + t;
  
  Vector2d uv;
  ConvertLocalToPanorama(color_width, color_height, color_phi_per_pixel, ray, &uv);
  return uv;
}

Vector2d ProjectLocalToColor(const Vector3d& local_point,
                             const vector<double>& params,
                             const int color_width,
                             const int color_height) {
  const double color_phi_per_pixel = params[0] / color_height;
  const Matrix3d rx = RotationX(params[1]);
  const Matrix3d rz = RotationZ(params[2]);
  const Matrix3d ry = RotationY(params[3]);
  const Vector3d t(params[6], params[5], params[4]);
  const Matrix3d r = ry * rz * rx;

  Vector3d color_point = r * local_point + t;

  Vector2d color_uv;
  ConvertLocalToPanorama(color_width, color_height, color_phi_per_pixel, color_point, &color_uv);
  return color_uv;
}

/*
Vector2d DepthToPanorama(const int panorama_width,
                         const int panorama_height,
                         const int depth_width,
                         const int depth_height,
                         const double depth_phi_per_pixel,
                         const double* const params,
                         const Vector2d& panorama_pixel) {
  const double panorama_phi_per_pixel = params[0] / panorama_height;
  const Matrix3d rx = RotationX(params[1]);
  const Matrix3d rz = RotationZ(params[2]);
  const Matrix3d ry = RotationY(params[3]);
  const Vector3d t(params[4], params[5], params[6]);
  const Matrix3d r = ry * rz * rx;
  Vector3d ray;
  ConvertPanoramaToLocal(panorama_width, panorama_height,
                         panorama_phi_per_pixel, panorama_pixel, &ray);
  ray = r * ray + t;
  
  Vector2d uv;
  ConvertLocalToPanorama(depth_width, depth_height, depth_phi_per_pixel, ray, &uv);
  return uv;
}
*/

/*
Vector2d PanoramaToDepth(const int panorama_width,
                         const int panorama_height,
                         const int depth_width,
                         const int depth_height,
                         const int depth_phi_per_pixel,
                         const double* const params,
                         const Vector2d& depth_pixel) {
  const double panorama_phi_per_pixel = params[0] / panorama_height;
  const Matrix3d rx = RotationX(params[1]);
  const Matrix3d rz = RotationZ(params[2]);
  const Matrix3d ry = RotationY(params[3]);
  const Vector3d t(params[4], params[5], params[6]);
  const Matrix3d r = ry * rz * rx;
  Vector3d ray;
  ConvertPanoramaToLocal(depth_width, depth_height,
                         depth_phi_per_pixel, depth_pixel, &ray);
  ray = r.transpose() * (ray - t);

  Vector2d uv;
  ConvertLocalToPanorama(panorama_width, panorama_height, panorama_phi_per_pixel, ray, &uv);
  return uv;
}
*/

void DetectEdgeColor(Image* color_image) {
  const int width = color_image->width;
  const int height = color_image->height;
  vector<double>* edge = &color_image->edge;
  const vector<Vector3d>& color = color_image->color;
  
  edge->clear();
  edge->resize(width * height, kInvalid);
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 0; x < width; ++x) {
      const int right_x = (x + 1) % width;
      const int left_x   = (x - 1 + width) % width;

      const Vector3d x0 = color[y * width + right_x];
      const Vector3d x1 = color[y * width + left_x];
      const Vector3d y0 = color[(y + 1) * width + x];
      const Vector3d y1 = color[(y - 1) * width + x];
      const Vector3d diffx = x0 - x1;
      const Vector3d diffy = y0 - y1;
      
      edge->at(y * width + x) =
        sqrt(diffx[0] * diffx[0] + diffx[1] * diffx[1] + diffx[2] * diffx[2] +
             diffy[0] * diffy[0] + diffy[1] * diffy[1] + diffy[2] * diffy[2]);
    }
  }

  // Normalize.
  double mean = 0.0, variance = 0.0;
  int denom = 0;
  for (const auto& value : (*edge)) {
    if (value != kInvalid) {
      mean += value;
      variance += value * value;
      ++denom;
    }
  }
  if (denom == 0) {
    cerr << "Impossible1." << endl;
    exit (1);
  }
  mean /= denom;
  variance /= denom;
  double deviation = sqrt(variance - mean * mean);
  
  for (auto& value : (*edge)) {
    if (value == kInvalid)
      continue;
    value = max(0.0, min(1.0, (value - mean) / (2 * deviation)));
  }
}


void SetColorImage(const cv::Mat& panorama_raw,
                   const int depth_width,
                   const int depth_height,
                   Image* color_image) {
  color_image->width = depth_width;
  color_image->height = depth_height;

  cv::Mat panorama;
  resize(panorama_raw, panorama,
         cv::Size(color_image->width, color_image->height));

  vector<Vector3d>* color = &color_image->color;
  color->resize(color_image->width * color_image->height);
  int index = 0;
  for (int y = 0; y < color_image->height; ++y) {
    for (int x = 0; x < color_image->width; ++x, ++index) {
      const Vec3b vtmp = panorama.at<Vec3b>(y, x);
      for (int i = 0; i < 3; ++i)
        color->at(index)[i] = vtmp[i];
    }
  }

  DetectEdgeColor(color_image);
}

double ReadOffsetTheta(const string buffer) {
  ifstream ifstr;
  ifstr.open(buffer.c_str());
  string stmp;
  for (int i = 0; i < 17; ++i)
    ifstr >> stmp;
  double dtmp;
  ifstr >> dtmp;
  ifstr.close();

  return dtmp;
}

double ReadOffsetZ(const string buffer) {
  ifstream ifstr;
  ifstr.open(buffer);
  string stmp;
  for (int i = 0; i < 15; ++i)
    ifstr >> stmp;
  double dtmp;
  ifstr >> dtmp;
  ifstr.close();

  return dtmp;
}

void InitializeParameters(const FileIO& file_io, const int p, vector<double>* params) {
  const string filename = file_io.GetImageAlignmentCalibration(p);
  params->resize(7);
  params->at(0) = 0.7 * M_PI;
  params->at(1) = 0.0 * M_PI;
  params->at(2) = -ReadOffsetTheta(filename);
  params->at(3) = 0.0;
  params->at(4) = -ReadOffsetZ(filename);
  params->at(5) = 0.0;
  params->at(6) = 0.0;

  double dtmp[] = {2.19772, -3.14159e-06, 2.55223, -3.14159e-06, 127.213, -2.36926, 0.632438 };
  for (int i = 0; i < 7; ++i)
    params->at(i) = dtmp[i];

  // lumber-cashew
  // params->at(2) = -ReadOffsetTheta(filename) + M_PI * 0.5529;

  // equal-sky
  //params->at(2) = -ReadOffsetTheta(filename) + M_PI * 0.4829;

  // equal sky (last image).
  // params->at(2) = -ReadOffsetTheta(filename) + M_PI * 0.8;
  // params->at(4) = -ReadOffsetZ(filename) + 60;

  // new-breeze
  // params->at(2) = -ReadOffsetTheta(filename) + M_PI * 0.8;
  // params->at(4) = -ReadOffsetZ(filename) - 60;
  // and pyramid_level 2.

  // red-lion.
  // params->at(4) += 60;
  

  // salmon palace
  //params->at(2) = -ReadOffsetTheta(filename) + M_PI * 0.8;
  //params->at(4) = -ReadOffsetZ(filename) - 60;
  // salmon palace 19
  // params->at(2) = -ReadOffsetTheta(filename) + M_PI * 0.8;
  // params->at(4) = -ReadOffsetZ(filename) + 60;
   

}

void FindEffectiveDepthPixels(const vector<double>& edge_image,
                              const int width,
                              const int height,
                              const int radius,
                              set<pair<int, int> >* pixels) {
  const double kThreshold = 0.4;
  int index = 0;
  for (int y = radius; y < height - radius; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      if (edge_image[index] >= kThreshold) {
        pixels->insert(make_pair(x, y));
      }
    }
  }

  /*
  const int kMaxNum = 10000;
  if (pixels->size() > kMaxNum) {
    vector<pair<int, int> > vtmp(pixels->begin(), pixels->end());
    random_shuffle(vtmp.begin(), vtmp.end());
    vtmp.resize(kMaxNum);
    pixels->clear();
    pixels->insert(vtmp.begin(), vtmp.end());
  }
  */
}

void ExhaustiveSearch(const Image& color_image,
                      const Image& depth_image,
                      const double depth_phi_range,
                      vector<double>* params,
                      ceres::Problem* problem) {
  const double depth_phi_per_pixel = depth_phi_range / depth_image.height;
  // Identify the representative point.
  double average_depth = 0.0;
  {
    int denom = 0;
    int index = 0;
    for (int y = 0; y < depth_image.height; ++y) {
      for (int x = 0; x < depth_image.width; ++x, ++index) {
        if (depth_image.depth[index] != kInvalid) {
          average_depth += depth_image.depth[index];
          ++denom;
        }
      }
    }
    if (denom == 0) {
      cerr << "impossible0" << endl;
      exit (1);
    }
    average_depth /= denom;
  }
  const Vector2d depth_pixel(depth_image.width / 2, depth_image.height / 3);
  const Vector2d reference = DepthToPanorama(color_image.width,
                                             color_image.height,
                                             depth_image.width,
                                             depth_image.height,
                                             depth_phi_per_pixel,
                                             &params->at(0),
                                             depth_pixel,
                                             average_depth);
  vector<double> params_org = *params;

  const double kTargetMove = 0.5;
  vector<double> units(7);
  vector<int> indexes;
  indexes.push_back(2);
  units[2] = M_PI * 0.005;
  indexes.push_back(4);
  units[4] = 20.0;
  indexes.push_back(5);
  units[5] = 40.0;
  indexes.push_back(6);
  units[6] = 40.0;
  for (const auto index : indexes) {
    *params = params_org;
    params->at(index) += units[index];
    const Vector2d move = DepthToPanorama(color_image.width,
                                          color_image.height,
                                          depth_image.width,
                                          depth_image.height,
                                          depth_phi_per_pixel,
                                          &params->at(0),
                                          depth_pixel,
                                          average_depth);
    const double diff = (reference - move).norm();
    units[index] *= kTargetMove / diff;
  }
    
  
  double min_cost = 1000000;
  double best_params[4];
  for (int l = -40; l <= 40; ++l) {
    cout << '*' << flush;
    for (int k = -20; k <= 20; ++k) {
      for (int j = -0; j <= 0; ++j) {
        for (int i = -0; i <= 0; ++i) {
          *params = params_org;
          params->at(2) += l * units[2];
          params->at(4) += k * units[4];
          params->at(5) += j * units[5];
          params->at(6) += i * units[6];
          
          double cost;
          problem->Evaluate(ceres::Problem::EvaluateOptions(),
                           &cost, NULL, NULL, NULL);
          if (cost < min_cost) {
            min_cost = cost;
            best_params[0] = params->at(2);
            best_params[1] = params->at(4);
            best_params[2] = params->at(5);
            best_params[3] = params->at(6);
          }
        }
      }
    }
  }
  cout << endl;
  params->at(2) = best_params[0];
  params->at(4) = best_params[1];
  params->at(5) = best_params[2];
  params->at(6) = best_params[3];
  for (int i = 0; i < params->size(); ++i)
    cout << params->at(i) << ' ';
  cout << endl;
}

void SetBounds(ceres::Problem* problem, vector<double>* params) {
  problem->SetParameterLowerBound(&(*params)[0], 0, params->at(0) * 0.8);
  problem->SetParameterUpperBound(&(*params)[0], 0, params->at(0) * 1.2);

  const double kInitialError = 0.2 * M_PI;
  const double kInitialSmallError = 0.000001 * M_PI;

  problem->SetParameterLowerBound(&(*params)[0], 1,
                                  (*params)[1] - kInitialSmallError);
  problem->SetParameterUpperBound(&(*params)[0], 1,
                                  (*params)[1] + kInitialSmallError);
  problem->SetParameterLowerBound(&(*params)[0], 2,
                                  (*params)[2] - kInitialError);
  problem->SetParameterUpperBound(&(*params)[0], 2,
                                  (*params)[2] + kInitialError);
  problem->SetParameterLowerBound(&(*params)[0], 3,
                                  (*params)[3] - kInitialSmallError);
  problem->SetParameterUpperBound(&(*params)[0], 3,
                                  (*params)[3] + kInitialSmallError);

  const double kTranslationError = 50;
  const double kTranslationSmallError = 10;
  problem->SetParameterLowerBound(&(*params)[0], 4, (*params)[4] - kTranslationError);
  problem->SetParameterUpperBound(&(*params)[0], 4, (*params)[4] + kTranslationError);
  problem->SetParameterLowerBound(&(*params)[0], 5, (*params)[5] - kTranslationSmallError);
  problem->SetParameterUpperBound(&(*params)[0], 5, (*params)[5] + kTranslationSmallError);
  problem->SetParameterLowerBound(&(*params)[0], 6, (*params)[6] - kTranslationSmallError);
  problem->SetParameterUpperBound(&(*params)[0], 6, (*params)[6] + kTranslationSmallError);
}

bool Normalize(vector<double>* patch) {
  double mean = 0.0;
  double variance = 0.0;
  int denom = 0;
  for (const auto& value : *patch) {
    if (value != kInvalid) {
      mean += value;
      variance += value * value;
      ++denom;
    }
  }

  if (denom == 0)
    return false;
  
  mean /= denom;
  variance /= denom;
  const double deviation = sqrt(max(0.0, variance - mean * mean));
  if (deviation == 0.0)
    return false;

  for (auto& value : *patch) {
    if (value != kInvalid)
      value = (value - mean) / deviation;
  }
  return true;
}

void VisualizeAlignment(const Image& color_image,
                        const Image& depth_image,
                        const double depth_phi_range,
                        const double* const params,
                        const set<pair<int, int> >& depth_pixels,
                        const string header,
                        const string filename) {
  const int depth_width = depth_image.width;
  const int depth_height = depth_image.height;
  const double depth_phi_per_pixel = depth_phi_range / depth_height;

  const int color_width = color_image.width;
  const int color_height = color_image.height;
  
  vector<double> depth_edge_in_color(color_width * color_height, kInvalid);
  vector<bool> depth_pixel_flag(color_width * color_height, false);
  for (int y = 0; y < depth_height; ++y) {
    for (int x = 0; x < depth_width; ++x) {
      const int index = y * depth_width + x;
      if (depth_image.depth[index] == kInvalid)
        continue;
      Vector2d color_pixel =
        DepthToPanorama(color_width,
                        color_height,
                        depth_width,
                        depth_height,
                        depth_phi_per_pixel,
                        params,
                        Vector2d(x, y),
                        depth_image.depth[index]);
      const int u = static_cast<int>(round(color_pixel[0]));
      const int v = static_cast<int>(round(color_pixel[1]));

      const int color_index = v * color_width + u;
      depth_edge_in_color[color_index] = depth_image.edge[index];
      if (depth_pixels.find(make_pair(x, y)) != depth_pixels.end())
        depth_pixel_flag[color_index] = true;
    }
  }

  cv::Mat blended_panorama(color_height, color_width, CV_8UC3);
  int index = 0;
  for (int y = 0; y < color_height; ++y) {
    for (int x = 0; x < color_width; ++x, ++index) {
      Vec3b color(0, 0, 0);
      if (color_image.edge[index] != kInvalid) {
        color[1] = static_cast<unsigned char>(round(255.0 * color_image.edge[index]));
      }

      if (depth_pixel_flag[index])
        color[2] = static_cast<unsigned char>(round(255.0 * depth_edge_in_color[index]));
        //color[2] = 255;//static_cast<unsigned char>(round(255.0 * depth_edge_in_color[index]));

      blended_panorama.at<Vec3b>(y, x) = color;
    }  
  }

  if (!filename.empty())
    cv::imwrite(filename.c_str(), blended_panorama);  
  cv::imshow(header.c_str(), blended_panorama);
}

double Interpolate(const vector<double>& image,
                   const int width,
                   const int height,
                   const Vector2d& pixel) {
  const int u0 = static_cast<int>(floor(pixel[0]));
  const int v0 = static_cast<int>(floor(pixel[1]));
  const int u1 = (u0 + 1) % width;
  const int v1 = v0 + 1;

  double numer = 0.0;
  double denom = 0.0;

  {
    const int index = v0 * width + u0;
    if (image[index] != kInvalid) {
      const double weight = (u0 + 1 - pixel[0]) * (v1- pixel[1]);
      numer += image[index] * weight;
      denom += weight;
    }
  }
  {
    const int index = v0 * width + u1;
    if (image[index] != kInvalid) {
      const double weight = (pixel[0] - u0) * (v1- pixel[1]);
      numer += image[index] * weight;
      denom += weight;
    }
  }
  {
    const int index = v1 * width + u0;
    if (image[index] != kInvalid) {
      const double weight = (u0 + 1 - pixel[0]) * (pixel[1] - v0);
      numer += image[index] * weight;
      denom += weight;
    }
  }
  {
    const int index = v1 * width + u1;
    if (image[index] != kInvalid) {
      const double weight = (pixel[0] - u0) * (pixel[1] - v0);
      numer += image[index] * weight;
      denom += weight;
    }
  }

  if (denom == 0.0)
    return kInvalid;
  else
    return numer / denom;
}

struct AlignPanoramaToDepthResidual {
public:
  AlignPanoramaToDepthResidual(const Image& color_image,
                               const Image& depth_image,
                               const Vector2i& depth_pixel,
                               const double depth_phi_range,
                               const int ncc_window_radius) :
    color_image(color_image),
    depth_image(depth_image),
    depth_pixel(depth_pixel),
    depth_phi_range(depth_phi_range),
    ncc_window_radius(ncc_window_radius) {
    depth_to_color_scale = color_image.width / static_cast<double>(depth_image.width);
  }

  template <typename T> bool operator()(const T* const params,
                                        T* residual) const {
    const double depth_phi_per_pixel = depth_phi_range / depth_image.height;
    Vector2d color_pixel =
      DepthToPanorama(color_image.width,
                      color_image.height,
                      depth_image.width,
                      depth_image.height,
                      depth_phi_per_pixel,
                      params,
                      Vector2d(depth_pixel[0], depth_pixel[1]),
                      depth_image.depth[depth_pixel[1] * depth_image.width + depth_pixel[0]]);
    const int color_margin = ceil(depth_to_color_scale * ncc_window_radius) + 1;
    const double kMinPenalty = 1.0;
    if (color_pixel[1] <= color_margin ||
        color_pixel[1] >= color_image.height - 1 - color_margin) {
      residual[0] = kMinPenalty;
      return true;
    }

    const double lhs =
      depth_image.edge[depth_pixel[1] * depth_image.width + depth_pixel[0]];
    const double rhs =
      Interpolate(color_image.edge, color_image.width, color_image.height, color_pixel);
    //cout << depth_pixel[0] << ' ' << depth_pixel[1] << ' ' << color_pixel[0] << ' ' << color_pixel[1]
    //<< ' ' << lhs << ' ' << rhs << endl;
    if (lhs == kInvalid || rhs == kInvalid)
      residual[0] = kMinPenalty;
    else
      residual[0] = 1.0 - rhs; // max(lhs - rhs, 0.0); // lhs - rhs;
    return true;

    /*    
    const int size = 2 * ncc_window_radius + 1;
    vector<double> depth_patch(size * size, kInvalid);
    vector<double> color_patch(size * size, kInvalid);
    
    int index = 0;
    for (int j = -ncc_window_radius; j <= ncc_window_radius; ++j) {
      const int y = depth_pixel[1] + j;
      for (int i = -ncc_window_radius; i <= ncc_window_radius; ++i, ++index) {
        const int x = depth_pixel[0] + i;
        depth_patch[index] = depth_image.edge[y * depth_image.width + x];
      }
    }

    index = 0;
    for (int j = -ncc_window_radius; j <= ncc_window_radius; ++j) {
      const double y = color_pixel[1] + j * depth_to_color_scale;
      for (int i = -ncc_window_radius; i <= ncc_window_radius; ++i, ++index) {
        const double x = color_pixel[0] + i * depth_to_color_scale;

        color_patch[index] =
          Interpolate(color_image.edge, color_image.width, color_image.height, Vector2d(x, y));
      }
    }
    
    if (Normalize(&depth_patch) && Normalize(&color_patch)) {
      residual[0] = 0.0;
      int denom = 0;
      for (int i = 0; i < depth_patch.size(); ++i) {
        if (depth_patch[i] != kInvalid && color_patch[i] != kInvalid) {
          const double diff = depth_patch[i] - color_patch[i];
          residual[0] += diff * diff;
          ++denom;
        }
      }
      residual[0] = residual[0] / denom;
      // Robust incc.
      residual[0] = residual[0] / (1 + 3.0 * residual[0]);
      
    } else {
      residual[0] = 1.0;
    }

    return true;
    */
  }

private:
  const Image& color_image;
  const Image& depth_image;
  const Vector2i depth_pixel;
  const double depth_phi_range;
  const int ncc_window_radius;
  double depth_to_color_scale;
};

void InitializeDepthImage(const FileIO& file_io,
                          const int p,
                          const double depth_phi_range,
                          Image* depth_image) {
  vector<DepthPoint> depth_points;
  ReadPly(file_io.GetLocalPly(p), &depth_image->width, &depth_image->height, &depth_points);
  
  SetDepthImage(depth_image->width, depth_image->height, depth_points, depth_phi_range,
                &depth_image->depth);

  DetectEdgeDepth(depth_image);
}

void InitializeColorImage(const FileIO& file_io,
                          const int p,
                          const int depth_width,
                          const int depth_height,
                          Image* color_image) {
  string buffer = file_io.GetPanoramaImage(p);
  cv::Mat panorama_raw = cv::imread(buffer, 1);
  if (panorama_raw.empty()) {
    cerr << "panorama does not exist: " << buffer << endl;
    exit (1);
  }
  SetColorImage(panorama_raw, depth_width, depth_height, color_image);
}

void MakeHalf(const int width, const int height,
              const vector<Vector3d>& image,
              const int half_width, const int half_height,
              vector<Vector3d>* half_image) {
  half_image->resize(half_width * half_height);
  int index = 0;
  for (int y = 0; y < half_height; ++y) {
    for (int x = 0; x < half_width; ++x, ++index) {
      int denom = 0;
      half_image->at(index) = Vector3d(0, 0, 0);
      for (int j = 0; j < 2; ++j) {
        const int ytmp = 2 * y + j;
        if (height <= ytmp)
          continue;
        for (int i = 0; i < 2; ++i) {
          const int xtmp = 2 * x + i;
          if (width <= xtmp)
            continue;
          
          half_image->at(index) += image[ytmp * width + xtmp];
          ++denom;
        }
      }
      if (denom == 0) {
        cerr << "Impossible in makehalf." << endl;
        exit (1);
      }
      half_image->at(index) /= denom;
    }
  }
}

void MakeHalf(const int width, const int height,
              const vector<double>& image,
              const int half_width, const int half_height,
              vector<double>* half_image) {
  half_image->resize(half_width * half_height);
  int index = 0;
  for (int y = 0; y < half_height; ++y) {
    for (int x = 0; x < half_width; ++x, ++index) {
      int denom = 0;
      half_image->at(index) = 0.0;
      for (int j = 0; j < 2; ++j) {
        const int ytmp = 2 * y + j;
        if (height <= ytmp)
          continue;
        for (int i = 0; i < 2; ++i) {
          const int xtmp = 2 * x + i;
          if (width <= xtmp)
            continue;

          if (image[ytmp * width + xtmp] != kInvalid) {
            half_image->at(index) += image[ytmp * width + xtmp];
            ++denom;
          }
        }
      }
      if (denom == 0) {
        half_image->at(index) = kInvalid;
      } else {
        half_image->at(index) /= denom;
      }
    }
  }
}

void BuildPyramid(vector<Image>* pyramid) {
  for (int level = 1; level < pyramid->size(); ++level) {
    pyramid->at(level).width  = pyramid->at(level - 1).width / 2;
    pyramid->at(level).height = pyramid->at(level - 1).height / 2;

    if (!pyramid->at(level - 1).color.empty()) {
      MakeHalf(pyramid->at(level - 1).width,
               pyramid->at(level - 1).height,
               pyramid->at(level - 1).color,
               pyramid->at(level).width,
               pyramid->at(level).height,
               &pyramid->at(level).color);
      DetectEdgeColor(&pyramid->at(level));
    } else {
      MakeHalf(pyramid->at(level - 1).width,
               pyramid->at(level - 1).height,
               pyramid->at(level - 1).depth,
               pyramid->at(level).width,
               pyramid->at(level).height,
               &pyramid->at(level).depth);
      DetectEdgeDepth(&pyramid->at(level));
    }
  }
}

void SetupProblem(const Image& color_image, const Image& depth_image, const double depth_phi_range,
                  const int ncc_window_radius, const set<pair<int, int> >& depth_pixels,
                  ceres::Problem* problem, vector<double>* params) {
  const double kHuberParameter = 0.3;
  cerr << "Pixels: " << depth_pixels.size() << endl;
  for (const auto& depth_pixel : depth_pixels) {
    problem->AddResidualBlock(new ceres::NumericDiffCostFunction
                              <AlignPanoramaToDepthResidual, ceres::CENTRAL, 1, 7>
                              (new AlignPanoramaToDepthResidual(color_image,
                                                                depth_image,
                                                                Vector2i(depth_pixel.first, depth_pixel.second),
                                                                depth_phi_range,
                                                                ncc_window_radius)),
                              new ceres::HuberLoss(kHuberParameter),
                              &params->at(0));
  }
}

void WriteResults(const FileIO& file_io, const int panorama, const vector<double>& params) {
  {
    ofstream ofstr;
    ofstr.open(file_io.GetPanoramaDepthAlignmentCalibration(panorama));
    ofstr << "CALIBRATION2" << endl;
    for (int i = 0; i < params.size(); ++i)
      ofstr << params[i] << ' ';
    ofstr << endl;      
    ofstr.close();
  }
  // Write camera_to_global.
  {   
    const Matrix3d rx = RotationX(params[1]);
    const Matrix3d rz = RotationZ(params[2]);
    const Matrix3d ry = RotationY(params[3]);
    const Vector3d t(params[6], params[5], params[4]);
    const Matrix3d r = ry * rz * rx;

    const Vector3d translation = - r.transpose() * t;
    
    Matrix4d camera_to_local;
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x)
        camera_to_local(y, x) = r.transpose()(y, x);
      camera_to_local(y, 3) = translation[y];
    }
    camera_to_local(3, 0) = 0;
    camera_to_local(3, 1) = 0;
    camera_to_local(3, 2) = 0;
    camera_to_local(3, 3) = 1;
    //???? debug
    Matrix4d local_to_global;
    {
      ifstream ifstr;
      ifstr.open(file_io.GetLocalToGlobalTransformation(panorama));
      char ctmp;
      ifstr >> ctmp;
      for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 4; ++x) {
          ifstr >> local_to_global(y, x);
        }
      }
      ifstr.close();
      local_to_global(3, 0) = 0;
      local_to_global(3, 1) = 0;
      local_to_global(3, 2) = 0;
      local_to_global(3, 3) = 1;
    }
    const Matrix4d camera_to_global = local_to_global * camera_to_local;

    ofstream ofstr;
    ofstr.open(file_io.GetPanoramaToGlobalTransformation(panorama));
    ofstr << "CAMEARA_TO_GLOBAL" << endl;
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        ofstr << camera_to_global(y, x) << ' ';
      }
      ofstr << endl;
    }
    ofstr << params[0] << endl;
    ofstr.close();    
  }
}

void WriteDepth(const FileIO& file_io, const int p, const vector<double>& params) {
  int color_width, color_height;
  {
    cv::Mat panorama_raw = cv::imread(file_io.GetPanoramaImage(p), 1);
    // Width of a depthmap must be odd (then we can split the geometry grid into 2 halves).
    const int kSkip = 8;
    color_width  = panorama_raw.cols / kSkip;
    color_height = panorama_raw.rows / kSkip;
    if (color_width % 2 == 0) {
      color_width += 1;
    }
  }
  const double color_phi_per_pixel = params[0] / color_height;

  vector<DepthPoint> depth_points;
  {
    int wtmp, htmp;
    ReadPly(file_io.GetLocalPly(p), &wtmp, &htmp, &depth_points);
  }

  vector<double> depth_in_color(color_width * color_height, kInvalid);
  for (const auto& depth_point : depth_points) {
    const Vector2d uv =
      ProjectLocalToColor(Vector3d(depth_point.X, depth_point.Y, depth_point.Z),
                          params,
                          color_width, color_height);
      
    const int x = static_cast<int>(round(uv[0])) % color_width;
    const int y = static_cast<int>(round(uv[1]));
    if (y < 0 || color_height <= y)
      continue;

    depth_in_color[y * color_width + x] = depth_point.distance;
  }

  // Apply hole-filling and smoothing.
  FillHolesAndSmooth(color_width, color_height, kInvalid, &depth_in_color);

  /*
  for (int y = 0; y < 27; ++y) {
    for (int x = 0; x < color_width; ++x) {
      depth_in_color[y * color_width + x] = 5000;
    }
  }
  for (int y = color_height - 39; y < color_height; ++y) {
    for (int x = 0; x < color_width; ++x) {
      depth_in_color[y * color_width + x] = 5000;
    }
  }
  */
  
  double min_distance, max_distance;
  bool first = true;
  for (const auto value : depth_in_color) {
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
    char buffer[1024];
    ofstream ofstr;
    ofstr.precision(5);
    ofstr.open(file_io.GetDepthPanorama(p));
    ofstr << "Depth" << endl
          << color_width << ' ' << color_height << endl
          << min_distance << ' ' << max_distance << endl;
    for (const auto value : depth_in_color) {
      ofstr << value << ' ';
    }
    ofstr.close();
  }
  {
    cv::Mat depth_image(color_height, color_width, CV_8UC3);
    int index = 0;
    for (int y = 0; y < color_height; ++y) {
      for (int x = 0; x < color_width; ++x, ++index) {
        if (depth_in_color[index] == kInvalid) {
          depth_image.at<Vec3b>(y, x) = Vec3b(0, 255, 0);
        } else {
          const int itmp =
            static_cast<int>(round(255.0 *
                                   (depth_in_color[index] - min_distance) / (max_distance - min_distance)));
          depth_image.at<Vec3b>(y, x) = Vec3b(itmp, itmp, itmp);
        }
      }
    }
    imwrite(file_io.GetDepthVisualization(p), depth_image);
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  const FileIO file_io(argv[1]);
  
  const double kDepthPhiRange = 0.8 * M_PI; // PhiPerPixel = 0.004363323;

  for (int p = FLAGS_start_panorama; p < FLAGS_end_panorama; ++p) {
    Image depth_image;
    InitializeDepthImage(file_io, p, kDepthPhiRange, &depth_image);
    Image color_image;
    InitializeColorImage(file_io, p, depth_image.width, depth_image.height, &color_image);

    vector<Image> depth_pyramid(FLAGS_num_pyramid_levels);
    vector<Image> color_pyramid(FLAGS_num_pyramid_levels);
    const int kBottomLevel = 0;
    depth_pyramid[kBottomLevel] = depth_image;
    color_pyramid[kBottomLevel] = color_image;

    BuildPyramid(&depth_pyramid);
    BuildPyramid(&color_pyramid);
    
    // Align. Parameters.
    // phi_coverage_along_ y, rotation_x, rotation_z, rotation_y, Tz, Ty, Tx.
    // Rotation from panorama to the local coordinate frame is given by: Ry Rz Rx.
    vector<double> params;

    ifstream ifstr;
    ifstr.open(file_io.GetPanoramaDepthAlignmentCalibration(p));
    if (!FLAGS_overwrite && ifstr.is_open()) {
      string stmp;
      ifstr >> stmp;
      const int kNumParams = 7;
      params.resize(kNumParams);
      for (int i = 0; i < kNumParams; ++i)
        ifstr >> params[i];
      ifstr.close();
    } else {
      InitializeParameters(file_io, p, &params);

      for (int level = FLAGS_num_pyramid_levels - 1; level >= 0; --level) {
        set<pair<int, int> > depth_pixels;
        FindEffectiveDepthPixels(depth_pyramid[level].edge,
                                 depth_pyramid[level].width,
                                 depth_pyramid[level].height,
                                 FLAGS_ncc_window_radius,
                                 &depth_pixels);
        
        VisualizeAlignment(color_pyramid[level], depth_pyramid[level], kDepthPhiRange,
                           &params[0], depth_pixels, "before", "");
        
        ceres::Problem problem;
        SetupProblem(color_pyramid[level], depth_pyramid[level], kDepthPhiRange,
                     FLAGS_ncc_window_radius, depth_pixels, &problem, &params);
        
        
        if (level == FLAGS_num_pyramid_levels - 1)
          ExhaustiveSearch(color_pyramid[level], depth_pyramid[level],
                           kDepthPhiRange, &params, &problem);
        
        SetBounds(&problem, &params);
        
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.num_threads = 2;
        options.minimizer_progress_to_stdout = true;
        
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";
        
        cout << "Param: ";
        for (int i = 0; i < params.size(); ++i)
          cout << params[i] << ' ';
        cout << endl;
        
        if (level == 0) {
          VisualizeAlignment(color_pyramid[level], depth_pyramid[level], kDepthPhiRange,
                             &params[0], depth_pixels, "after",
                             file_io.GetPanoramaDepthAlignmentVisualization(p));
        } else {
          VisualizeAlignment(color_pyramid[level], depth_pyramid[level], kDepthPhiRange,
                             &params[0], depth_pixels, "after", "");
        }
        // cv::waitKey(0);
      }
    }

    //----------------------------------------------------------------------
    WriteResults(file_io, p, params);

    WriteDepth(file_io, p, params);
  }

  return 0;
}





    /*
    {    
      ofstream ofstr;
      ofstr.open("depth.ppm");
      ofstr << "P3" << endl
            << depth_width << ' ' << depth_height << endl
            << 255 << endl;
      
      for (const auto& value : depth_edge_image) {
        if (value == kInvalid)
          ofstr << "0 255 0 " << endl;
        else {
          const int itmp = static_cast<int>(255 * value);
          ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
        }
      }
      ofstr.close();
    }
    {    
      ofstream ofstr;
      ofstr.open("pano.ppm");
      ofstr << "P3" << endl
            << width << ' ' << height << endl
            << 255 << endl;
      
      for (const auto& value : panorama_edge_image) {
        if (value == kInvalid)
          ofstr << "0 255 0 " << endl;
        else {
          const int itmp = static_cast<int>(255 * value);
          ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
        }
      }
      ofstr.close();
    }
    */
/*
      {
        const Image& depth_image = depth_pyramid[kBottomLevel];
        ofstream ofstr;
        ofstr.open("depth.ppm");
        ofstr << "P3" << endl
              << depth_image.width << ' ' << depth_image.height << endl
              << 255 << endl;
        
        for (const auto& value : depth_image.edge) {
          if (value == kInvalid)
            ofstr << "0 255 0 " << endl;
          else {
            const int itmp = static_cast<int>(255 * value);
            ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
          }
        }
        ofstr.close();
      }
      {
        const Image& color_image = color_pyramid[kBottomLevel];
        ofstream ofstr;
        ofstr.open("pano.ppm");
        ofstr << "P3" << endl
              << color_image.width << ' ' << color_image.height << endl
              << 255 << endl;
        
        for (const auto& value : color_image.edge) {
          if (value == kInvalid)
            ofstr << "0 255 0 " << endl;
          else {
            const int itmp = static_cast<int>(255 * value);
            ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
          }
        }
        ofstr.close();
      }

*/  
