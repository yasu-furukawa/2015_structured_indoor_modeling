#ifndef ALIGN_IMAGES_H__
#define ALIGN_IMAGES_H__

#include <cstdio>
#include <iostream>
#include <fstream>
#include <numeric>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "align_images.h"
#include "ceres/ceres.h"
#include "../../base/file_io.h"

namespace structured_indoor_modeling {

void PrepareImages(const FileIO& file_io,
                   const int num_images,
                   const int dynamic_range_index,
                   const int num_pyramid_levels,
                   const int p,
                   std::vector<std::vector<cv::Mat> >* images,
                   std::vector<std::vector<cv::Mat> >* dx_images,
                   std::vector<std::vector<cv::Mat> >* dy_images);

void InitializeParams(const int num_images,
                      const double focal_length,
                      const int level,
                      std::vector<double>* global_params);

void BlendPanorama(const int num_images, const std::vector<double>& global_params,
                   const std::vector<cv::Mat>& images, const double phi_per_pixel, cv::Mat* panorama);

void SetPanorama(const int num_images, const std::vector<double>& global_params,
                 const std::vector<cv::Mat>& images, const double phi_per_pixel, cv::Mat* panorama);

void FindEffectivePixels(const std::vector<cv::Mat>& dx_images,
                         const std::vector<cv::Mat>& dy_images,
                         const std::vector<double>& global_params,
                         const int lhs_image,
                         const int rhs_image,
                         const int skip,
                         const int max_pixels_per_pair,
                         std::set<std::pair<int, int> >* lhs_uv_set);

void ConvertPanoramaToLocal(const int panorama_width, const int panorama_height,
                            const double phi_per_pixel, const Eigen::Vector2d& uv,
                            Eigen::Vector3d* ray);

void ApplyDistortion(const int width,
                     const int height,
                     const double f,
                     const double k1,
                     const double k2,
                     const Eigen::Vector2d& uv,
                     Eigen::Vector2d* distorted_uv);

void CorrectDistortion(const int width,
                       const int height,
                       const double f,
                       const double k1,
                       const double k2,
                       const Eigen::Vector2d& uv,
                       Eigen::Vector2d* corrected_uv);

void ConvertParamsToProjection(const int width, const int height,
                               const double* const local_params,
                               Eigen::Matrix<double, 3, 4>* projection);

void ConvertParamsToUnprojection(const int width, const int height,
                                 const double* const local_params,
                                 Eigen::Matrix<double, 4, 3>* unprojection);

double ComputeSAD(const std::vector<double>& lhs, const std::vector<double>& rhs);

void SetBounds(const int num_images, ceres::Problem* problem, std::vector<double>* global_params);

struct AlignImagesResidual {
public:
  AlignImagesResidual(const int ssd_window_radius,
                      const cv::Mat& lhs, const cv::Mat& lhs_dx, const cv::Mat& lhs_dy,
                      const cv::Mat& rhs, const cv::Mat& rhs_dx, const cv::Mat& rhs_dy,
                      const Eigen::Vector2i lhs_uv) :
  ssd_window_radius(ssd_window_radius), lhs(lhs), lhs_dx(lhs_dx), lhs_dy(lhs_dy),
    rhs(rhs), rhs_dx(rhs_dx), rhs_dy(rhs_dy), lhs_uv(lhs_uv) {
  }

  template <typename T> bool operator()(const T* const f,
                                        const T* const lhs_param_block,
                                        const T* const rhs_param_block,
                                        T* residual) const {
    const double lhs_local_params[9] =
      { f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], lhs_param_block[0] };
    const double rhs_local_params[9] =
      { f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], rhs_param_block[0] };

    Eigen::Matrix<double, 4, 3> lhs_unprojection;
    ConvertParamsToUnprojection(lhs_dx.cols, lhs_dx.rows, lhs_local_params, &lhs_unprojection);
    Eigen::Matrix<double, 3, 4> rhs_projection;
    ConvertParamsToProjection(rhs_dx.cols, rhs_dx.rows, rhs_local_params, &rhs_projection);

    Eigen::Vector2d lhs_corrected_uv;
    CorrectDistortion(lhs_dx.cols, lhs_dx.rows, f[0], f[1], f[2],
                      Eigen::Vector2d(lhs_uv[0], lhs_uv[1]), &lhs_corrected_uv);

    Eigen::Vector4d xyz = lhs_unprojection *
      Eigen::Vector3d(lhs_corrected_uv[0], lhs_corrected_uv[1], 1.0);
    Eigen::Vector3d rhs_uv = rhs_projection * xyz;
    
    const double kMinPenalty = 10;
    if (rhs_uv[2] <= 0.0) {
      residual[0] = kMinPenalty;
      // residual[1] = kMinPenalty;
    } else {
      rhs_uv /= rhs_uv[2];
      Eigen::Vector2d rhs_distorted_uv;
      ApplyDistortion(rhs_dx.cols, rhs_dx.rows, f[0], f[1], f[2],
                      Eigen::Vector2d(rhs_uv[0], rhs_uv[1]), &rhs_distorted_uv);

      const int u0 = static_cast<int>(floor(rhs_distorted_uv[0]));
      const int v0 = static_cast<int>(floor(rhs_distorted_uv[1]));
      const int u1 = (u0 + 1);
      const int v1 = (v0 + 1);
      const float u = rhs_distorted_uv[0];
      const float v = rhs_distorted_uv[1];
      
      if (ssd_window_radius <= u0 && u0 < rhs_dx.cols - ssd_window_radius - 1 &&
          ssd_window_radius <= v0 && v0 < rhs_dx.rows - ssd_window_radius - 1) {
        std::vector<double> lhs_values, rhs_values;
        for (int j = -ssd_window_radius; j <= ssd_window_radius; ++j) {
          const int ytmp = lhs_uv[1] + j;
          for (int i = -ssd_window_radius; i <= ssd_window_radius; ++i) {
            const int xtmp = lhs_uv[0] + i;
            const cv::Vec3b color = lhs.at<cv::Vec3b>(ytmp, xtmp);
            //for (int k = 0; k < 3; ++k)
            //lhs_values.push_back(color[k]);
            lhs_values.push_back((color[0] + color[1] + color[2]) / 3.0);
          }
        }
        
        const double weight00 = (u1 - u) * (v1 - v);
        const double weight01 = (u - u0) * (v1 - v);
        const double weight10 = (u1 - u) * (v - v0);
        const double weight11 = (u - u0) * (v - v0);

        for (int j = -ssd_window_radius; j <= ssd_window_radius; ++j) {
          for (int i = -ssd_window_radius; i <= ssd_window_radius; ++i) {
            const cv::Vec3b& color00 = rhs.at<cv::Vec3b>(v0 + j, u0 + i);
            const cv::Vec3b& color01 = rhs.at<cv::Vec3b>(v0 + j, u1 + i);
            const cv::Vec3b& color10 = rhs.at<cv::Vec3b>(v1 + j, u0 + i);
            const cv::Vec3b& color11 = rhs.at<cv::Vec3b>(v1 + j, u1 + i);

            /*
            for (int k = 0; k < 3; ++k) {
              rhs_values.push_back(weight00 * color00[k] +
                                   weight01 * color01[k] +
                                   weight10 * color10[k] +
                                   weight11 * color11[k]);
            }
            */
            double value = 0.0;
            for (int k = 0; k < 3; ++k) {
              value += weight00 * color00[k] +
                weight01 * color01[k] +
                weight10 * color10[k] +
                weight11 * color11[k];
            }
            rhs_values.push_back(value / 3.0);
          }
        }

        // Normalize mean and take the difference.
        residual[0] = ComputeSAD(lhs_values, rhs_values);
        // residual[1] = lhs_dy.at<float>(lhs_uv[1], lhs_uv[0]) - dy;
      } else {
        residual[0] = kMinPenalty;
        // residual[1] = kMinPenalty;
      }
    }
    return true;
  }
  
private:
  int ssd_window_radius;
  const cv::Mat& lhs;
  const cv::Mat& lhs_dx;
  const cv::Mat& lhs_dy;
  const cv::Mat& rhs;
  const cv::Mat& rhs_dx;
  const cv::Mat& rhs_dy;
  const Eigen::Vector2i lhs_uv;
};

struct RegularizationResidual {
public:
RegularizationResidual(const int num_images, const int num_constraints) :
  num_images(num_images), num_constraints(num_constraints) {
  }

  template <typename T> bool operator()(const T* const param0,
                                        const T* const param1,
                                        const T* const param2,
                                        const T* const param3,
                                        const T* const param4,
                                        const T* const param5,
                                        const T* const param6,
                                        const T* const param7,
                                        T* residual) const {
    residual[0] = num_constraints * 100 * (param0[0] + param2[0] - 2 * param1[0]);
    residual[1] = num_constraints * 100 * (param1[0] + param3[0] - 2 * param2[0]);
    residual[2] = num_constraints * 100 * (param2[0] + param4[0] - 2 * param3[0]);
    residual[3] = num_constraints * 100 * (param3[0] + param5[0] - 2 * param4[0]);
    residual[4] = num_constraints * 100 * (param4[0] + param6[0] - 2 * param5[0]);
    residual[5] = num_constraints * 100 * (param5[0] + param7[0] - 2 * param6[0]);
    
    /*
    for (int left = 0; left < num_images - 2; ++left) {
      const int middle = left + 1;
      const int right = left + 2;
      residual[left] = num_constraints * (params[right] + params[left] - 2 * params[middle]);
    }
    */
    return true;
  }
  
private:
  const int num_images;
  const int num_constraints;
    

};

void PrintParams(const std::vector<double>& global_params);

}  // namespace structured_indoor_modeling

#endif  // ALIGN_IMAGES_H__
