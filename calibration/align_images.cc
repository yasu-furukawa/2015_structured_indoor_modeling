#include <cstdio>
#include <iostream>
#include <fstream>
#include <numeric>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "align_images.h"
#include "ceres/ceres.h"
#include "transformation.h"

using namespace cv;
using namespace Eigen;
using namespace file_io;
using namespace std;

namespace structured_indoor_modeling {

namespace {

Matrix3d LocalParamsToRotation(const double* const local_params) {
  const Matrix3d Rgx = RotationX(local_params[3]);
  const Matrix3d Rgz = RotationZ(local_params[4]);
  const Matrix3d Ry  = RotationY(local_params[8]);
  const Matrix3d Rlx = RotationX(local_params[5]);
  const Matrix3d Rly = RotationY(local_params[6]);
  const Matrix3d Rlz = RotationZ(local_params[7]);
  
  return Rlz * Rly * Rlx * Ry * Rgz * Rgx;
}
                       
void FindLhsPixelsInsideMargin(const int lhs_width,
                               const int lhs_height,
                               const vector<double>& lhs_local_params,
                               const int rhs_width,
                               const int rhs_height,
                               const vector<double>& rhs_local_params,
                               const int skip,
                               set<pair<int, int> >* lhs_uv_set) {
  Matrix<double, 4, 3> lhs_unprojection;
  ConvertParamsToUnprojection(lhs_width, lhs_height, &lhs_local_params[0], &lhs_unprojection);
  Matrix<double, 3, 4> rhs_projection;
  ConvertParamsToProjection(rhs_width, rhs_height, &rhs_local_params[0], &rhs_projection);  

  //????
  const int margin = 5;
  for (int y = margin; y < lhs_height - margin; y += skip) {
    for (int x = margin; x < lhs_width - margin; x += skip) {
      Vector2d corrected_uv;
      CorrectDistortion(lhs_width, lhs_height,
                        lhs_local_params[0], lhs_local_params[1], lhs_local_params[2],
                        Vector2d(x, y), &corrected_uv);

      Vector4d xyz = lhs_unprojection * Vector3d(corrected_uv[0], corrected_uv[1], 1.0);
      Vector3d uv = rhs_projection * xyz;    
      
      if (uv[2] <= 0.0) {
        continue;
      } else {
        uv /= uv[2];
        Vector2d distorted_uv;
        ApplyDistortion(rhs_width, rhs_height,
                        rhs_local_params[0], rhs_local_params[1], rhs_local_params[2],
                        Vector2d(uv[0], uv[1]), &distorted_uv);
        {
          Vector2d corrected_uv;
          CorrectDistortion(rhs_width, rhs_height,
                            rhs_local_params[0], rhs_local_params[1], rhs_local_params[2],
                            distorted_uv, &corrected_uv);
        if ((corrected_uv - Vector2d(uv[0], uv[1])).norm() > 2.0)
          continue;
        }
        
        const int u = static_cast<int>(round(distorted_uv[0]));
        const int v = static_cast<int>(round(distorted_uv[1]));
        if (margin <= u && u < rhs_width - margin && margin <= v && v < rhs_height - margin) {
          lhs_uv_set->insert(make_pair(x, y));
        }
      }
    }
  }
}

void KeepHighGradientPixels(const Mat& dx_image, const Mat& dy_image,
                            set<pair<int, int> >* uv_set) {
  const int kMinimumPixelsToProceed = 100;
  if (uv_set->size() < kMinimumPixelsToProceed) {
    return;
  }
  double top_threshold, bottom_threshold;
  const double kPercentile = 0.8;
  
  {
    vector<double> top_values, bottom_values;
    for (const auto& uv : *uv_set) {
      const float dx = dx_image.at<float>(uv.second, uv.first);
      const float dy = dy_image.at<float>(uv.second, uv.first);
      const double diff = sqrt(dx * dx + dy * dy);
      if (uv.second < dx_image.rows / 2) {
        top_values.push_back(diff);
      } else {
        bottom_values.push_back(diff);
      }
    }
    nth_element(top_values.begin(),
                top_values.begin() + static_cast<int>(kPercentile * top_values.size()),
                top_values.end());
    top_threshold = *(top_values.begin() + static_cast<int>(kPercentile * top_values.size()));
    nth_element(bottom_values.begin(),
                bottom_values.begin() + static_cast<int>(kPercentile * bottom_values.size()),
                bottom_values.end());
    bottom_threshold = *(bottom_values.begin() + static_cast<int>(kPercentile * bottom_values.size()));

  }

  set<pair<int, int> > new_uv_set;
  for (const auto& uv : *uv_set) {
    const float dx = dx_image.at<float>(uv.second, uv.first);
    const float dy = dy_image.at<float>(uv.second, uv.first);
    const double diff = sqrt(dx * dx + dy * dy);
    if (uv.second < dx_image.rows / 2) {
      if (diff >= top_threshold)
        new_uv_set.insert(uv);
    } else {
      if (diff >= bottom_threshold)
        new_uv_set.insert(uv);          
    }
  }
  
  new_uv_set.swap(*uv_set);

  /*
  bool kUseMedian = true;
  double threshold;
  if (kUseMedian) {
    const double kPercentile = 0.85;
    vector<double> values;
    for (int y = 1; y < dx_image.rows - 1; ++y) {
      for (int x = 1; x < dx_image.cols - 1; ++x) {
        const float dx = dx_image.at<float>(y, x);
        const float dy = dy_image.at<float>(y, x);
        values.push_back(sqrt(dx * dx + dy * dy));
      }
    }
    nth_element(values.begin(),
                values.begin() + static_cast<int>(kPercentile * values.size()),
                values.end());
    threshold = *(values.begin() + static_cast<int>(kPercentile * values.size()));
  } else {
    const double low_threshold = 10.0;
    double mean = 0.0;
    double variance = 0.0;
    int denom = 0;
    for (int y = 1; y < dx_image.rows - 1; ++y) {
      for (int x = 1; x < dx_image.cols - 1; ++x) {
        const float dx = dx_image.at<float>(y, x);
        const float dy = dy_image.at<float>(y, x);
        const double diff = sqrt(dx * dx + dy * dy);
        if (diff > low_threshold) {
          mean += diff;
          variance += diff * diff;
          ++denom;
        }
      }
    }
    
    mean /= denom;
    variance = variance / denom - mean * mean;
    const double deviation = sqrt(variance);
    threshold = mean - 1.5 * deviation;
  }

  set<pair<int, int> > new_uv_set;
  for (const auto& uv : *uv_set) {
      const float dx = dx_image.at<float>(uv.second, uv.first);
      const float dy = dy_image.at<float>(uv.second, uv.first);
      const double diff = sqrt(dx * dx + dy * dy);
      if (diff >= threshold)
        new_uv_set.insert(uv);
  }

  new_uv_set.swap(*uv_set);
  */
}

void SetSingleParams(const vector<double>& global_params,
                     const int index,
                     vector<double>* local_params) {
  const int kOffset = 8;
  local_params->resize(9);
  for (int i = 0; i < kOffset; ++i)
    local_params->at(i) = global_params[i];

  local_params->at(kOffset) = global_params[kOffset + index];
}  

template <typename T>
void ClearImage(Mat* image) {
  for (int y = 0; y < image->rows; ++y) {
    for (int x = 0; x < image->cols; ++x) {
      image->at<T>(y, x) = cv::Vec3b(0, 0, 0);
    }
  }
}

void AccumulateImageToPanorama(const double* const local_params,
                               const Mat& image,
                               const double phi_per_pixel,
                               Mat* panorama) {
  Matrix<double, 3, 4> projection;
  ConvertParamsToProjection(image.cols, image.rows, local_params, &projection);

  for (int y = 0; y < panorama->rows; ++y) {
    for (int x = 0; x < panorama->cols; ++x) {
      Vector3d ray;
      ConvertPanoramaToLocal(panorama->cols, panorama->rows,
                             phi_per_pixel, Vector2d(x, y),
                             &ray);
      const Vector4d ray4(ray[0], ray[1], ray[2], 1.0);
      Vector3d uv = projection * ray4;
      
      if (uv[2] <= 0.0)
        continue;
      uv /= uv[2];
      
      Vector2d distorted_uv;
      ApplyDistortion(image.cols, image.rows, local_params[0], local_params[1], local_params[2],
                      Vector2d(uv[0], uv[1]), &distorted_uv);
      {
        Vector2d corrected_uv;
        CorrectDistortion(image.cols, image.rows,
                          local_params[0], local_params[1], local_params[2],
                          distorted_uv, &corrected_uv);
        if ((corrected_uv - Vector2d(uv[0], uv[1])).norm() > 2.0)
          continue;
      }
      
      const int u = static_cast<int>(round(distorted_uv[0]));
      const int v = static_cast<int>(round(distorted_uv[1]));

      if (0 <= u && u < image.cols && 0 <= v && v < image.rows) {
        if (panorama->at<cv::Vec3b>(y, x) == cv::Vec3b(0, 0, 0)) {
          panorama->at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(v, u);
        } else {
          panorama->at<cv::Vec3b>(y, x) = panorama->at<cv::Vec3b>(y, x) / 2 + image.at<cv::Vec3b>(v, u) / 2;
        }
      }
    }
  }
}

void ComputeWeight(const double distance0, const double distance1,
                   double* weight0, double* weight1) {
  // How quickly blending changes.
  const double kTransition = 10;
  const double diff0 = distance0 - distance1;
  const double diff1 = distance1 - distance0;
  
  *weight0 = max(0.0, min(1.0, diff0 / kTransition / 2.0 + 0.5));
  *weight1 = max(0.0, min(1.0, diff1 / kTransition / 2.0 + 0.5));
}
  
void BlendImageToPanorama(const double* const local_params,
                          const Mat& image,
                          const double phi_per_pixel,
                          Mat* panorama,
                          vector<double>* alpha) {
  Matrix<double, 3, 4> projection;
  ConvertParamsToProjection(image.cols, image.rows, local_params, &projection);

  int index = 0;
  for (int y = 0; y < panorama->rows; ++y) {
    for (int x = 0; x < panorama->cols; ++x, ++index) {
      Vector3d ray;
      ConvertPanoramaToLocal(panorama->cols, panorama->rows,
                             phi_per_pixel, Vector2d(x, y),
                             &ray);
      const Vector4d ray4(ray[0], ray[1], ray[2], 1.0);
      Vector3d uv = projection * ray4;
      
      if (uv[2] <= 0.0)
        continue;
      uv /= uv[2];
      
      Vector2d distorted_uv;
      ApplyDistortion(image.cols, image.rows, local_params[0], local_params[1], local_params[2],
                      Vector2d(uv[0], uv[1]), &distorted_uv);
      {
        Vector2d corrected_uv;
        CorrectDistortion(image.cols, image.rows, local_params[0], local_params[1], local_params[2],
                          distorted_uv, &corrected_uv);
        if ((corrected_uv - Vector2d(uv[0], uv[1])).norm() > 2.0)
          continue;
      }        

      const int u0 = static_cast<int>(floor(distorted_uv[0]));
      const int v0 = static_cast<int>(floor(distorted_uv[1]));
      const int u1 = u0 + 1;
      const int v1 = v0 + 1;

      if (0 <= u0 && u0 < image.cols - 1 && 0 <= v0 && v0 < image.rows - 1) {
        // Distance from the boundary.
        const double du = min(distorted_uv[0], image.cols - 1 - distorted_uv[0]);
        const double dv = min(distorted_uv[1], image.rows - 1 - distorted_uv[1]);
        const double distance = sqrt(du * du + dv * dv);

        const double weight00 = (u1 - distorted_uv[0]) * (v1 - distorted_uv[1]);
        const double weight01 = (distorted_uv[0] - u0) * (v1 - distorted_uv[1]);
        const double weight10 = (u1 - distorted_uv[0]) * (distorted_uv[1] - v0);
        const double weight11 = (distorted_uv[0] - u0) * (distorted_uv[1] - v0);

        const cv::Vec3b& color00 = image.at<cv::Vec3b>(v0, u0);
        const cv::Vec3b& color01 = image.at<cv::Vec3b>(v0, u1);
        const cv::Vec3b& color10 = image.at<cv::Vec3b>(v1, u0);
        const cv::Vec3b& color11 = image.at<cv::Vec3b>(v1, u1);

        Vec3b interpolated_color(static_cast<unsigned char>(weight00 * color00[0] +
                                                            weight01 * color01[0] +
                                                            weight10 * color10[0] +
                                                            weight11 * color11[0]),
                                 static_cast<unsigned char>(weight00 * color00[1] +
                                                            weight01 * color01[1] +
                                                            weight10 * color10[1] +
                                                            weight11 * color11[1]),
                                 static_cast<unsigned char>(weight00 * color00[2] +
                                                            weight01 * color01[2] +
                                                            weight10 * color10[2] +
                                                            weight11 * color11[2]));
                
        if (alpha->at(index) == 0.0) {
          panorama->at<cv::Vec3b>(y, x) = interpolated_color;
          alpha->at(index) = distance;
        } else {
          double weight0, weight1;
          ComputeWeight(distance, alpha->at(index), &weight0, &weight1);

          Vec3b color0 = interpolated_color;
          Vec3b color1 = panorama->at<cv::Vec3b>(y, x);
          panorama->at<cv::Vec3b>(y, x) =
                        Vec3b(static_cast<unsigned char>(weight0 * color0[0] + weight1 * color1[0]),
                              static_cast<unsigned char>(weight0 * color0[1] + weight1 * color1[1]),
                              static_cast<unsigned char>(weight0 * color0[2] + weight1 * color1[2]));
        }
      }
    }
  }
}
  
// Convert RGB image to dx/dy gradient image.
void ConvertRGBToDxDy(const Mat& rgb_image, Mat* dx_image, Mat* dy_image) {
  *dx_image = Mat(rgb_image.rows, rgb_image.cols, CV_32F);
  *dy_image = Mat(rgb_image.rows, rgb_image.cols, CV_32F);

  for (int y = 0; y < rgb_image.rows; ++y) {
    for (int x = 0; x < rgb_image.cols; ++x) {
      if (x == 0 || x == rgb_image.cols - 1) {
        dx_image->at<float>(y, x) = 0.0;
      } else {
        cv::Vec3b diff = rgb_image.at<cv::Vec3b>(y, x + 1) - rgb_image.at<cv::Vec3b>(y, x - 1);
        dx_image->at<float>(y, x) = (diff[0] + diff[1] + diff[2]) / 3.0 / 2.0;
      }        
    }
  }
  for (int y = 0; y < rgb_image.rows; ++y) {
    for (int x = 0; x < rgb_image.cols; ++x) {
      if (y == 0 || y == rgb_image.rows - 1) {
        dy_image->at<float>(y, x) = 0.0;
      } else {
        cv::Vec3b diff = rgb_image.at<cv::Vec3b>(y + 1, x) - rgb_image.at<cv::Vec3b>(y - 1, x);
        dy_image->at<float>(y, x) = (diff[0] + diff[1] + diff[2]) / 3.0 / 2.0;
      }
    }
  }
}

}  // namespace

//----------------------------------------------------------------------
void SetBounds(const int num_images, ceres::Problem* problem, vector<double>* global_params) {
  const double kEpsilon = 0.000001;
  // f
  {
    problem->SetParameterLowerBound(&((*global_params)[0]), 0, (*global_params)[0] * 0.8);
    problem->SetParameterUpperBound(&((*global_params)[0]), 0, (*global_params)[0] * 1.2);
  }
  // k0, k1
  {
    const double kDistortionLimit = 0.4; // kEpsilon;
    problem->SetParameterLowerBound(&(*global_params)[0], 1, -kDistortionLimit); // k0 must be positive.
    problem->SetParameterUpperBound(&(*global_params)[0], 1, kDistortionLimit);
    problem->SetParameterLowerBound(&(*global_params)[0], 2, -kDistortionLimit);
    problem->SetParameterUpperBound(&(*global_params)[0], 2, kDistortionLimit);
  }
  // rgx, rgz
  {
    const double kSmallError = M_PI / 16.0;
    problem->SetParameterLowerBound(&((*global_params)[0]), 3, (*global_params)[3] - kSmallError);
    problem->SetParameterUpperBound(&((*global_params)[0]), 3, (*global_params)[3] + kSmallError);
    problem->SetParameterLowerBound(&((*global_params)[0]), 4, (*global_params)[4] - kSmallError);
    problem->SetParameterUpperBound(&((*global_params)[0]), 4, (*global_params)[4] + kSmallError);
  }
  // rlx rly rlz
  {
    const double kSmallError = M_PI / 16.0;
    problem->SetParameterLowerBound(&((*global_params)[0]), 5, (*global_params)[5] - kSmallError);
    problem->SetParameterUpperBound(&((*global_params)[0]), 5, (*global_params)[5] + kSmallError);
    problem->SetParameterLowerBound(&((*global_params)[0]), 6, (*global_params)[6] - kSmallError);
    problem->SetParameterUpperBound(&((*global_params)[0]), 6, (*global_params)[6] + kSmallError);
    problem->SetParameterLowerBound(&((*global_params)[0]), 7, (*global_params)[7] - kSmallError);
    problem->SetParameterUpperBound(&((*global_params)[0]), 7, (*global_params)[7] + kSmallError);
  }

  // [ry]
  {
    const int kOffset = 8;
    const int kFixId = 0;
    // Does not allow the first to move. hack fix properly later.
    problem->SetParameterLowerBound(&(*global_params)[kOffset + kFixId], 0,
                                    (*global_params)[kOffset + kFixId]);
    problem->SetParameterUpperBound(&(*global_params)[kOffset + kFixId], 0,
                                    (*global_params)[kOffset + kFixId] + kEpsilon); 

    for (int image = 0; image < num_images; ++image) {
      if (image == kFixId)
        continue;
      const double kVerySmallError = M_PI / 32.0;
      problem->SetParameterLowerBound(&(*global_params)[kOffset + image], 0,
                                      (*global_params)[kOffset + image] - kVerySmallError);
      problem->SetParameterUpperBound(&(*global_params)[kOffset + image], 0,
                                      (*global_params)[kOffset + image] + kVerySmallError);
    }
  }
}

double ComputeSAD(const vector<double>& lhs, const vector<double>& rhs) {
  if (lhs.size() != rhs.size()) {
    cerr << "Impossible" << endl;
    exit (1);
  }
  // Mean.
  double lhs_mean = accumulate(lhs.begin(), lhs.end(), 0.0) / lhs.size();
  double rhs_mean = accumulate(rhs.begin(), rhs.end(), 0.0) / rhs.size();

  double sad = 0.0;
  for (int i = 0; i < lhs.size(); ++i) {
    const double diff = (lhs[i] - lhs_mean) - (rhs[i] - rhs_mean);
    sad += diff * diff;
  }
  return sad;
}

void ConvertParamsToProjection(const int width, const int height,
                               const double* const local_params,
                               Matrix<double, 3, 4>* projection) {
  const double& f  = local_params[0];
  Eigen::Matrix3d intrinsics;
  intrinsics << f, 0, width / 2.0, 0, f, height / 2.0, 0, 0, 1;

  projection->block(0, 0, 3, 3) = intrinsics * LocalParamsToRotation(local_params);
  (*projection)(0, 3) = 0;
  (*projection)(1, 3) = 0;
  (*projection)(2, 3) = 0;
}

void ConvertParamsToUnprojection(const int width, const int height,
                                 const double* const local_params, Matrix<double, 4, 3>* unprojection) {
  const double& f  = local_params[0];
  Eigen::Matrix3d intrinsics_inverse;
  intrinsics_inverse <<
    1 / f, 0, - width / 2.0 / f,
    0, 1 / f, - height / 2.0 / f,
    0, 0, 1;

  const Matrix3d rotation = LocalParamsToRotation(local_params);

  unprojection->block(0, 0, 3, 3) = rotation.transpose() * intrinsics_inverse;
  (*unprojection)(3, 0) = 0;
  (*unprojection)(3, 1) = 0;
  (*unprojection)(3, 2) = 0;
}

void ApplyDistortion(const int width,
                     const int height,
                     const double f,
                     const double k1,
                     const double k2,
                     const Vector2d& uv,
                     Vector2d* distorted_uv) {
  const Vector2d center(width / 2.0, height / 2.0);
  Vector2d canonical_uv = (uv - center) / f;
  const double r2 = canonical_uv.squaredNorm();
  canonical_uv *= (1.0 + k1 * r2 + k2 * r2 * r2);
  *distorted_uv = canonical_uv * f + center;
}

void CorrectDistortion(const int width,
                       const int height,
                       const double f,
                       const double k1,
                       const double k2,
                       const Vector2d& uv,
                       Vector2d* corrected_uv) {
  const Vector2d center(width / 2.0, height / 2.0);
  Vector2d canonical_uv = (uv - center) / f;
  // Solve the following.
  // canonica_uv = x * (1 + k1 r^2 + k4 r^4).
  // x = corrected_uv.
  const int kMaxIteration = 5;
  *corrected_uv = canonical_uv;
  for (int i = 0; i < kMaxIteration; ++i) {
    const double r2 = corrected_uv->squaredNorm();
    const double m = (1.0 + k1 * r2 + k2 * r2 * r2);
    *corrected_uv = canonical_uv / m;
  }

  *corrected_uv = *corrected_uv * f + center;
}

void PrepareImages(const FileIO& file_io,
                   const int num_images,
                   const int dynamic_range_index,
                   const int num_pyramid_levels,
                   const int p,
                   vector<vector<cv::Mat> >* images,
                   vector<vector<cv::Mat> >* dx_images,
                   vector<vector<cv::Mat> >* dy_images) {
  images->resize(num_pyramid_levels);
  dx_images->resize(num_pyramid_levels);
  dy_images->resize(num_pyramid_levels);

  const int kFirstLevel = 0;
  images->at(kFirstLevel).resize(num_images);
  dx_images->at(kFirstLevel).resize(num_images);
  dy_images->at(kFirstLevel).resize(num_images);
  for (int i = 0; i < num_images; ++i) {
    (*images)[kFirstLevel][i] = cv::imread(file_io.GetRawImage(p, i, dynamic_range_index), 1);
    if ((*images)[kFirstLevel][i].empty()) {
      cerr << "Image does not exist." << endl;
      exit (1);      
    }
    
    ConvertRGBToDxDy((*images)[kFirstLevel][i], &(*dx_images)[kFirstLevel][i], &(*dy_images)[kFirstLevel][i]);
  }

  for (int level = 1; level < num_pyramid_levels; ++level) {
    images->at(level).resize(num_images);
    dx_images->at(level).resize(num_images);
    dy_images->at(level).resize(num_images);
    for (int image = 0; image < num_images; ++image) {
      pyrDown((*images)[level - 1][image], (*images)[level][image],
              Size((*images)[level - 1][image].cols / 2,
                   (*images)[level - 1][image].rows / 2));
      ConvertRGBToDxDy((*images)[level][image], &(*dx_images)[level][image], &(*dy_images)[level][image]);
    }
  }
}                   

void InitializeParams(const int num_images, const double focal_length, const int level, vector<double>* global_params) {
  // f k0 k1.
  (*global_params)[0] = focal_length / (0x01 << level);
  (*global_params)[1] = 0.0;
  (*global_params)[2] = 0.0;

  // rgx rgz
  (*global_params)[3] = M_PI / 2.0;
  (*global_params)[4] = 0.0;

  // rlx rly rlz
  (*global_params)[5] = 0.0;
  (*global_params)[6] = 0.0;
  (*global_params)[7] = 0.0;

  // ry
  const int kOffset = 8;
  for (int i = 0; i < num_images; ++i) {
    (*global_params)[kOffset + i] = M_PI - M_PI / 4.0 * i;
  }

  /*
  // lumber_cashew
  const double values[] =
    { 541.323, -0.000987145, -0.0103715, 1.5708, 0, -0.0725574, 0.111371, -0.0351254, 3.1416, 2.3713, 1.593, 0.806331, 0.0274377, -0.819101, -1.58709, -2.36137 };
  */
  // red_lion
  const double values[] =
    { 536.569, -0.00412691, -0.0066791, 1.5708, 0, -0.0338938, 0.0770123, 0.00658064, 3.1416, 2.35784, 1.57512, 0.788915, 0.00359924, -0.778726, -1.56254, -2.34497 };
  // { 524.561, 0.00212407, -0.0114861, 1.5708, 0, -0.0338403, 0.00395927, 0.00836349, 3.14159, 2.47073, 1.67251, 0.864916, 0.0648947, -0.731859, -1.53224, -2.32955 };
  for (int i = 0; i < 16; ++i)
    global_params->at(i) = values[i];
  global_params->at(0) /= (0x01 << level);
}  

void SetPanorama(const int num_images, const vector<double>& global_params,
                 const vector<cv::Mat>& images, const double phi_per_pixel, Mat* panorama) {
  ClearImage<cv::Vec3b>(panorama);
  for (int i = 0; i < num_images; ++i) {
    vector<double> local_params;
    SetSingleParams(global_params, i, &local_params);
    AccumulateImageToPanorama(&local_params[0], images[i], phi_per_pixel, panorama);
  }
}

void FindEffectivePixels(const vector<cv::Mat>& dx_images,
                         const vector<cv::Mat>& dy_images,
                         const vector<double>& global_params,
                         const int lhs_image,
                         const int rhs_image,
                         const int skip,
                         const int max_pixels_per_pair,
                         set<pair<int, int> >* lhs_uv_set) {
  vector<double> lhs_local_params, rhs_local_params;
  SetSingleParams(global_params, lhs_image, &lhs_local_params);
  SetSingleParams(global_params, rhs_image, &rhs_local_params);

  FindLhsPixelsInsideMargin(dx_images[lhs_image].cols, dx_images[lhs_image].rows, lhs_local_params,
                            dx_images[rhs_image].cols, dx_images[rhs_image].rows, rhs_local_params,
                            skip, lhs_uv_set);
  cerr << static_cast<int>(lhs_uv_set->size()) << " -> ";
  /*
  {
    Mat image(dx_images[lhs_image].rows, dx_images[lhs_image].cols, CV_8UC1);
    for (int y = 0; y < dx_images[lhs_image].rows; ++y) {
      for (int x = 0; x < dx_images[lhs_image].cols; ++x) {
        image.at<unsigned char>(y, x) = 0;
      }
    }
    for (const auto& pixel : *lhs_uv_set)
      image.at<unsigned char>(pixel.second, pixel.first) = 255;
    cv::imshow("overlap", image);
    cv::waitKey(0);
  }
  */  
  KeepHighGradientPixels(dx_images[lhs_image], dy_images[lhs_image], lhs_uv_set);
  cerr << static_cast<int>(lhs_uv_set->size());

  if (lhs_uv_set->size() > max_pixels_per_pair) {
    vector<pair<int, int> > vptmp(lhs_uv_set->begin(), lhs_uv_set->end());
    random_shuffle(vptmp.begin(), vptmp.end());
    vptmp.resize(max_pixels_per_pair);
    lhs_uv_set->clear();
    lhs_uv_set->insert(vptmp.begin(), vptmp.end());
    cerr << " -> " << lhs_uv_set->size() << " pixels." << endl;
  } else {
    cerr << " pixels." << endl;
  }
  /*
  {
    Mat image(dx_images[lhs_image].rows, dx_images[lhs_image].cols, CV_8UC1);
    for (int y = 0; y < dx_images[lhs_image].rows; ++y) {
      for (int x = 0; x < dx_images[lhs_image].cols; ++x) {
        image.at<unsigned char>(y, x) = 0;
      }
    }
    for (const auto& pixel : *lhs_uv_set)
      image.at<unsigned char>(pixel.second, pixel.first) = 255;
    cv::imshow("gradient", image);
    cv::waitKey(0);
  }
  */
}

void PrintParams(const vector<double>& global_params) {
  const int kOffset = 8;
  cerr << "Parameters: ";
  for (int i = 0; i < kOffset; ++i)
    cerr << global_params[i] << ' ';
  cerr << "  ";
  for (int i = kOffset; i < global_params.size(); ++i) {
    cerr << global_params[i] << ' ';
  }
  cerr << endl;
}

void BlendPanorama(const int num_images, const vector<double>& global_params,
                   const vector<cv::Mat>& images, const double phi_per_pixel, Mat* panorama) {
  ClearImage<cv::Vec3b>(panorama);
  vector<double> alpha(panorama->cols * panorama->rows, 0.0);
  for (int i = 0; i < num_images; ++i) {
    vector<double> local_params;
    SetSingleParams(global_params, i, &local_params);
    BlendImageToPanorama(&local_params[0], images[i], phi_per_pixel, panorama, &alpha);
  }
}

}  // namespace structured_indoor_modeling
  
