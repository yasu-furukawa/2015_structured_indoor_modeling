#include <cstdio>
#include <iostream>
#include <fstream>
#include <numeric>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "align_images.h"
#include "ceres/ceres.h"
#include "file_io.h"
#include "gflags/gflags.h"

//----------------------------------------------------------------------
// Rotation - projection
// [ Rlz Rly Rlx Ry Rgz Rgx ]
// Rgz Rgx is to align world coordinate frame to the axis of rotation.
// Ry is per camera and a rotation around an axis.
// Rlz Rly Rlx is to align the axis of rotation to the camera coordinate frame.
//
// [ global_params ]
// f k0 k1 rgx rgz rlx rly rlz   ry0 ry1 ry2 ...
// [ local_params ]
// f k0 k1 rgx rgz rlx rly rlz ry
// [ param_block ]
// ry
//----------------------------------------------------------------------

using namespace cv;
using namespace Eigen;
using namespace std;

DEFINE_int32(num_pyramid_levels, 6, "Num pyramid levels.");
DEFINE_double(phi_in_panorama, 0.7 * M_PI, "phi per pixel.");
DEFINE_int32(panorama_width, 3000, "Panorama width.");
DEFINE_double(focal_length, 500.0, "Focal length.");
DEFINE_int32(ssd_window_radius, 2, "ssd window radius");

DEFINE_double(gradient_percentile, 0.8, "Use pixels with strong gradient.");
DEFINE_int32(skip, 2, "Subsampling pixels.");
DEFINE_int32(max_pixels_per_pair, 200, "max pixels per pair.");
DEFINE_int32(start_panorama, 0, "Start panorama index.");
DEFINE_int32(end_panorama, 1, "End panorama index (exclusive).");
DEFINE_int32(num_images, 8, "Number of images per panorama");
DEFINE_int32(dynamic_range_index, 1, "There are 3 input images.");

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  const file_io::FileIO file_io(argv[1]);
  const int num_images = FLAGS_num_images;
  for (int p = FLAGS_start_panorama; p < FLAGS_end_panorama; ++p) {
    // Images to be aligned.
    vector<vector<cv::Mat> > images, dx_images, dy_images;
    PrepareImages(file_io, num_images, FLAGS_dynamic_range_index,
                  FLAGS_num_pyramid_levels, p, &images, &dx_images, &dy_images);
    
    // Parameters.
    const int kOffset = 8;
    vector<double> global_params(kOffset + num_images);
    InitializeParams(num_images, FLAGS_focal_length, FLAGS_num_pyramid_levels, &global_params);

    for (int level = FLAGS_num_pyramid_levels - 1; level >= 0; --level) {
      global_params[0] *= 2.0;
      
      const int panorama_width = FLAGS_panorama_width / (0x01 << level);
      const int panorama_height = panorama_width / 2;
      const double phi_per_pixel = FLAGS_phi_in_panorama  / panorama_height;
      Mat panorama(panorama_height, panorama_width, CV_8UC3);
    
      SetPanorama(num_images, global_params, images[level], phi_per_pixel, &panorama);
      char buffer[1024];
      sprintf(buffer, "Before %02d", level);
      cv::imshow(buffer, panorama);
      // cv::waitKey(0);

      ceres::Problem problem;
      const double kHuberParameter = 20;
      // Identify (lhs_uv) sets.
      int num_constraints = 0;
      for (int lhs_image = 0; lhs_image < num_images; ++lhs_image) {
        const int rhs_image = (lhs_image + 1) % num_images;
        set<pair<int, int> > lhs_uv_set;
        FindEffectivePixels(dx_images[level], dy_images[level],
                            global_params, lhs_image, rhs_image,
                            FLAGS_skip, FLAGS_max_pixels_per_pair, &lhs_uv_set);
        //----------------------------------------------------------------------
        num_constraints += lhs_uv_set.size();
        for (const auto& uv : lhs_uv_set) {
          problem.AddResidualBlock(new ceres::NumericDiffCostFunction<AlignImagesResidual,
                                   ceres::CENTRAL, 1, 8, 1, 1>
                                   (new AlignImagesResidual(FLAGS_ssd_window_radius,
                                                            images[level][lhs_image],
                                                            dx_images[level][lhs_image],
                                                            dy_images[level][lhs_image],
                                                            images[level][rhs_image],
                                                            dx_images[level][rhs_image],
                                                            dy_images[level][rhs_image],
                                                            Vector2i(uv.first, uv.second))),
                                   new ceres::HuberLoss(kHuberParameter),
                                   &(global_params[0]),
                                   &(global_params[kOffset + lhs_image]),
                                   &(global_params[kOffset + rhs_image]));
        }        
      }
      problem.AddResidualBlock(new ceres::NumericDiffCostFunction<RegularizationResidual,
                               ceres::CENTRAL, 6, 1, 1, 1, 1, 1, 1, 1, 1>
                               (new RegularizationResidual(num_images, num_constraints)),
                               new ceres::TrivialLoss(),
                               &global_params[kOffset + 0],
                               &global_params[kOffset + 1],
                               &global_params[kOffset + 2],
                               &global_params[kOffset + 3],
                               &global_params[kOffset + 4],
                               &global_params[kOffset + 5],
                               &global_params[kOffset + 6],
                               &global_params[kOffset + 7]);

      // Set Bounds.
      SetBounds(num_images, &problem, &global_params);
    
      ceres::Solver::Options options;
      options.max_num_iterations = 100;
      options.num_threads = 2;
      // options.linear_solver_type = ceres::DENSE_QR;
      // options.minimizer_progress_to_stdout = true;
      PrintParams(global_params);
      // Run the solver!
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      // std::cout << summary.FullReport() << "\n";
      
      PrintParams(global_params);
      cerr << summary.initial_cost << " -> " << summary.final_cost << endl;

      SetPanorama(num_images, global_params, images[level], phi_per_pixel, &panorama);
      sprintf(buffer, "After %02d", level);
      cv::imshow(buffer, panorama);

      BlendPanorama(num_images, global_params, images[level], phi_per_pixel, &panorama);
      sprintf(buffer, "Blended %02d", level);
      cv::imshow(buffer, panorama);

      if (level == 0) {
        cv::imwrite(file_io.GetPanoramaImage(p), panorama);
        ofstream ofstr;
        ofstr.open(file_io.GetImageAlignmentCalibration(p));
        ofstr << "CALIBRATION" << endl;
        for (const auto value : global_params) {
          ofstr << value << ' ';
        }
        ofstr << endl;
        ofstr.close();
      }
      
      
      // cv::waitKey(0);
    }
  }    
  return 0; 
}
