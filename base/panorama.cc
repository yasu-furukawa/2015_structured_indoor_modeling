#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>

#include "panorama.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

Panorama::Panorama() {
  only_background_black = false;
}

bool Panorama::Init(const FileIO& file_io,
                    const int panorama) {
  rgb_image = cv::imread(file_io.GetPanoramaImage(panorama), 1);
  if (rgb_image.cols == 0 && rgb_image.rows == 0) {
    cerr << "Panorama image cannot be loaded: " << file_io.GetPanoramaImage(panorama) << endl;
    return false;
  }
  width  = rgb_image.cols;
  height = rgb_image.rows;
  InitDepthImage(file_io, panorama);
  InitCameraParameters(file_io, panorama);
  phi_per_pixel = phi_range / height;
  phi_per_depth_pixel = phi_range / depth_height;
  return true;
}

bool Panorama::InitWithoutLoadingImages(const FileIO& file_io, const int panorama) {
  InitCameraParameters(file_io, panorama);
  phi_per_pixel = phi_range / height;
  phi_per_depth_pixel = phi_range / depth_height;
  return true;
}

bool Panorama::InitWithoutDepths(const FileIO& file_io, const int panorama) {
  rgb_image = cv::imread(file_io.GetPanoramaImage(panorama), 1);
  if (rgb_image.cols == 0 && rgb_image.rows == 0) {
    cerr << "Panorama image cannot be loaded: " << file_io.GetPanoramaImage(panorama) << endl;
    return false;
  }
  width  = rgb_image.cols;
  height = rgb_image.rows;
  InitCameraParameters(file_io, panorama);
  phi_per_pixel = phi_range / height;
  return true;
}  

Eigen::Vector2d Panorama::Project(const Eigen::Vector3d& global) const {
  const Vector3d local = GlobalToLocal(global);

  // x coordinate.
  double theta = -atan2(local.y(), local.x());
  if (theta < 0.0)
    theta += 2 * M_PI;
  
  double theta_ratio = max(0.0, min(1.0, theta / (2 * M_PI)));
  if (theta_ratio == 1.0)
    theta_ratio = 0.0;

  Vector2d uv;
  uv[0] = theta_ratio * width;
  const double depth = sqrt(local.x() * local.x() +
                            local.y() * local.y());
  double phi = atan2(local.z(), depth);
  const double pixel_offset_from_center = phi / phi_per_pixel;
  // uv[1] = height / 2.0 - pixel_offset_from_center;
  uv[1] = max(0.0, min(height - 1.1, height / 2.0 - pixel_offset_from_center));

  return uv;
}

Eigen::Vector3d Panorama::Unproject(const Eigen::Vector2d& pixel,
                                    const double distance) const {
  const double theta = -2.0 * M_PI * pixel[0] / width;
  const double phi   = (height / 2.0 - pixel[1]) * phi_per_pixel;

  Vector3d local;
  local[2] = distance * sin(phi);
  local[0] = distance * cos(phi) * cos(theta);
  local[1] = distance * cos(phi) * sin(theta);

  return LocalToGlobal(local);
}

Eigen::Vector2d Panorama::ProjectToDepth(const Eigen::Vector3d& global) const {
  const Eigen::Vector2d pixel = Project(global);
  return RGBToDepth(pixel);
}

Eigen::Vector3d Panorama::GlobalToLocal(const Eigen::Vector3d& global) const {
  const Vector4d global4(global[0], global[1], global[2], 1.0);
  const Vector4d local4 = global_to_local * global4;
  return Vector3d(local4[0], local4[1], local4[2]); 
}

Eigen::Vector3d Panorama::LocalToGlobal(const Eigen::Vector3d& local) const {
  const Vector4d local4(local[0], local[1], local[2], 1.0);
  const Vector4d global4 = local_to_global * local4;
  return Vector3d(global4[0], global4[1], global4[2]); 
}

Eigen::Vector2d Panorama::RGBToDepth(const Eigen::Vector2d& pixel) const {
  return Vector2d(pixel[0] * depth_width / width,
                  min(depth_height - 1.1, pixel[1] * depth_height / height));
}

Eigen::Vector2d Panorama::DepthToRGB(const Eigen::Vector2d& depth_pixel) const {
  return Vector2d(depth_pixel[0] * width / depth_width,
                  min(height - 1.1, depth_pixel[1] * height / depth_height));
}

Eigen::Vector3f Panorama::GetRGB(const Eigen::Vector2d& pixel) const {
  if (!IsInsideRGB(pixel)) {
    cerr << "Pixel outside: " << pixel[0] << ' ' << pixel[1] << ' '
         << width << ' ' << height << endl;
    exit (1);
  }
  
  // Bilinear interpolation.
  const int u0 = static_cast<int>(floor(pixel[0]));
  const int v0 = static_cast<int>(floor(pixel[1]));
  const int u1 = u0 + 1;
  int v1 = v0 + 1;
  
  const double weight00 = (u1 - pixel[0]) * (v1 - pixel[1]);
  const double weight01 = (pixel[0] - u0) * (v1 - pixel[1]);
  const double weight10 = (u1 - pixel[0]) * (pixel[1] - v0);
  const double weight11 = (pixel[0] - u0) * (pixel[1] - v0);
  const int u1_corrected = (u1 % width);

  v1 = min(v1, height - 1);
      
  const cv::Vec3b& color00 = rgb_image.at<cv::Vec3b>(v0, u0);
  const cv::Vec3b& color01 = rgb_image.at<cv::Vec3b>(v0, u1_corrected);
  const cv::Vec3b& color10 = rgb_image.at<cv::Vec3b>(v1, u0);
  const cv::Vec3b& color11 = rgb_image.at<cv::Vec3b>(v1, u1_corrected);

  const cv::Vec3b kHole(0, 0, 0);
  if (only_background_black) {
    double weight_sum = 0.0;
    Vector3f color_sum(0, 0, 0);
    if (color00 != kHole) {
      color_sum += Vector3f(weight00 * color00[0],
                            weight00 * color00[1],
                            weight00 * color00[2]);
      weight_sum += weight00;
    }
    if (color01 != kHole) {
      color_sum += Vector3f(weight01 * color01[0],
                            weight01 * color01[1],
                            weight01 * color01[2]);
      weight_sum += weight01;
    }
    if (color10 != kHole) {
      color_sum += Vector3f(weight10 * color10[0],
                            weight10 * color10[1],
                            weight10 * color10[2]);
      weight_sum += weight10;
    }
    if (color11 != kHole) {
      color_sum += Vector3f(weight11 * color11[0],
                            weight11 * color11[1],
                            weight11 * color11[2]);
      weight_sum += weight11;
    }
    if (weight_sum == 0.0)
      return Vector3f(0, 0, 0);
    else
      return color_sum / weight_sum;
  } else {
    return Vector3f((weight00 * color00[0] + weight01 * color01[0] +
                     weight10 * color10[0] + weight11 * color11[0]),
                    (weight00 * color00[1] + weight01 * color01[1] +
                     weight10 * color10[1] + weight11 * color11[1]),
                    (weight00 * color00[2] + weight01 * color01[2] +
                     weight10 * color10[2] + weight11 * color11[2]));
  }
}

double Panorama::GetDepth(const Eigen::Vector2d& depth_pixel) const {
  if (!IsInsideDepth(depth_pixel)) {
    cerr << "Depth pixel outside: " << depth_pixel[0] << ' ' << depth_pixel[1] << ' '
         << depth_width << ' ' << depth_height << endl;
    exit (1);
  }
  
  // Bilinear interpolation.
  const int u0 = static_cast<int>(floor(depth_pixel[0]));
  const int v0 = static_cast<int>(floor(depth_pixel[1]));
  const int u1 = u0 + 1;
  int v1 = v0 + 1;
  
  const double weight00 = (u1 - depth_pixel[0]) * (v1 - depth_pixel[1]);
  const double weight01 = (depth_pixel[0] - u0) * (v1 - depth_pixel[1]);
  const double weight10 = (u1 - depth_pixel[0]) * (depth_pixel[1] - v0);
  const double weight11 = (depth_pixel[0] - u0) * (depth_pixel[1] - v0);
  const int u1_corrected = (u1 % depth_width);

  v1 = min(v1, depth_height - 1);
  
  return
    weight00 * depth_image[v0 * depth_width + u0] +
    weight01 * depth_image[v0 * depth_width + u1_corrected] +
    weight10 * depth_image[v1 * depth_width + u0] +
    weight11 * depth_image[v1 * depth_width + u1_corrected];
}

double Panorama::GetPhiRange() const {
  return phi_range;
}

double Panorama::GetPhiPerPixel() const {
  return phi_per_pixel;
}  

Eigen::Matrix4d Panorama::GetGlobalToLocal() const {
  return global_to_local;
}
  
Eigen::Matrix4d Panorama::GetLocalToGlobal() const {
  return local_to_global;
}
  
bool Panorama::IsInsideRGB(const Eigen::Vector2d& pixel) const {
  if (pixel[0] < 0.0 || width <= pixel[0] ||
      pixel[1] < 0.0 || height - 1 < pixel[1]) {
    return false;
  } else {
    return true;
  }
}

bool Panorama::IsInsideDepth(const Eigen::Vector2d& depth_pixel) const {
  if (depth_pixel[0] < 0.0 || depth_width <= depth_pixel[0] ||
      depth_pixel[1] < 0.0 || depth_height - 1 < depth_pixel[1]) {
    return false;
  } else {
    return true;
  }
}

void Panorama::Resize(const Eigen::Vector2i& size) {
  const int new_width = size[0];
  const int new_height = size[1];
  
  const int x_scale = width  / new_width;
  const int y_scale = height / new_height;

  const int new_depth_width = depth_width / x_scale;
  const int new_depth_height = depth_height / y_scale;
  
  if (only_background_black) {
    const cv::Vec3b kHole(0, 0, 0);
    cv::Mat new_rgb_image(new_height, new_width, CV_8UC3);
    
    for (int y = 0; y < new_height; ++y) {
      const int start_y = y * y_scale;
      const int end_y   = (y + 1) * y_scale;
      for (int x = 0; x < new_width; ++x) {
        const int start_x = x * x_scale;
        const int end_x   = (x + 1) * x_scale;

        Vector3f color(0, 0, 0);
        int denom = 0;
        for (int j = start_y; j < end_y; ++j) {
          for (int i = start_x; i < end_x; ++i) {
            const cv::Vec3b v3b = rgb_image.at<cv::Vec3b>(j, i);
            if (v3b != kHole) {
              color += Vector3f(v3b[0], v3b[1], v3b[2]);
              ++denom;
            }
          }
        }
        if (denom != 0)
          color /= denom;
        new_rgb_image.at<cv::Vec3b>(y, x) =
          cv::Vec3b(static_cast<int>(round(color[0])),
                    static_cast<int>(round(color[1])),
                    static_cast<int>(round(color[2])));
      }
    }
    rgb_image = new_rgb_image;
  } else {
    cv::Mat new_rgb_image;
    cv::resize(rgb_image, new_rgb_image, cv::Size(new_width, new_height));
    rgb_image = new_rgb_image;
  }

  // Resize depth.
  {
    const double kInvalid = -1.0;
    vector<double> new_depth_image(new_depth_width * new_depth_height);
    
    for (int y = 0; y < new_depth_height; ++y) {
      const int start_y = y * y_scale;
      const int end_y   = (y + 1) * y_scale;
      for (int x = 0; x < new_depth_width; ++x) {
        const int start_x = x * x_scale;
        const int end_x   = (x + 1) * x_scale;
        
        double sum_depth = 0.0;
        int denom = 0;
        for (int j = start_y; j < end_y; ++j) {
          for (int i = start_x; i < end_x; ++i) {
            const double depth = depth_image[j * depth_width + i];
            if (depth != kInvalid) {
              sum_depth += depth;
              ++denom;
            }
          }
        }
        if (denom != 0)
          sum_depth /= denom;
        else
          sum_depth = kInvalid;
        new_depth_image[y * new_depth_width + x] = sum_depth;
      }
    }
    depth_image.swap(new_depth_image);
  }

  width  = new_width;
  height = new_height;
    
  phi_per_pixel = phi_range / height;

  depth_width = new_depth_width;
  depth_height = new_depth_height;

  phi_per_depth_pixel = phi_range / depth_height;
}

void Panorama::AdjustCenter(const Eigen::Vector3d& new_center) {
  const Vector3d translation = center - new_center;
  Matrix4d adjustment;
  adjustment.setIdentity();
  for (int i = 0; i < 3; ++i)
    adjustment(i, 3) = translation[i];
  
  global_to_local = global_to_local * adjustment;
  
  SetGlobalToLocalFromLocalToGlobal();
}
  
  /*
void Panorama::ReleaseMemory() {
  rgb_image.release();
  vector<double>().swap(depth_image);
}
  */

void Panorama::InitDepthImage(const FileIO& file_io,
                              const int panorama) {
  // Interpolate myself.
  ifstream ifstr;
  ifstr.open(file_io.GetDepthPanorama(panorama));
  if (!ifstr.is_open()) {
    cerr << "Cannot open a file: " << file_io.GetDepthPanorama(panorama) << endl;
    exit (1);
  }

  string header;
  double min_depth, max_depth;
  ifstr >> header >> depth_width >> depth_height >> min_depth >> max_depth;
    
  depth_image.resize(depth_width * depth_height);
  
  int index = 0;
  average_distance = 0.0;
  for (int y = 0; y < depth_height; ++y) {
    for (int x = 0; x < depth_width; ++x, ++index) {
      ifstr >> depth_image[index];
      average_distance += depth_image[index];
    }
  }
  ifstr.close();

  average_distance /= depth_width * depth_height;
}
  
void Panorama::InitCameraParameters(const FileIO& file_io,
                                    const int panorama) {
  const string buffer = file_io.GetPanoramaToGlobalTransformation(panorama);

  ifstream ifstr;
  ifstr.open(buffer.c_str());
  string stmp;
  ifstr >> stmp;
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x)
      local_to_global(y, x) = 0;
  
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x)
      ifstr >> local_to_global(y, x);
  }
  for (int y = 0; y < 3; ++y)
    center(y) = local_to_global(y, 3);

  SetGlobalToLocalFromLocalToGlobal();
    
  ifstr >> phi_range;
  
  ifstr.close();
}

void Panorama::SetGlobalToLocalFromLocalToGlobal() {
  const Matrix3d rotation = local_to_global.block(0, 0, 3, 3);
  global_to_local.block(0, 0, 3, 3) = rotation.transpose();
  global_to_local.block(0, 3, 3, 1) =
    - rotation.transpose() * local_to_global.block(0, 3, 3, 1);
  global_to_local(3, 0) = 0.0;
  global_to_local(3, 1) = 0.0;
  global_to_local(3, 2) = 0.0;
  global_to_local(3, 3) = 1.0;
}  

void Panorama::MakeOnlyBackgroundBlack() {
  // Background must be in [0, kTopRato] or [kBottomRatio, 1].
  const double kTopRatio = 170 / 1500.0;
  const double kBottomRatio = 1250 / 1500.0;

  const int top_height = static_cast<int>(round(height * kTopRatio));
  const int bottom_height = static_cast<int>(round(height * kBottomRatio));

  // Starting from the top most or the bottom most pixel, identify the black pixels.
  for (int x = 0; x < width; ++x) {
    int top_index, bottom_index;
    for (top_index = 0; top_index < top_height; ++top_index) {
      if (rgb_image.at<cv::Vec3b>(top_index, x) == cv::Vec3b(0, 0, 0))
        continue;
      else
        break;
    }
    for (bottom_index = height - 1; bottom_index > bottom_height; --bottom_index) {
      if (rgb_image.at<cv::Vec3b>(bottom_index, x) == cv::Vec3b(0, 0, 0))
        continue;
      else
        break;
    }

    // Make black pixels between top_index and bottom_index to (1, 1, 1).
    for (int y = top_index; y < bottom_index; ++y) {
      if (rgb_image.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 0, 0))
        rgb_image.at<cv::Vec3b>(y, x) = cv::Vec3b(1, 1, 1);
    }
  }
  only_background_black = true;
}

//----------------------------------------------------------------------
// Utility functions.  
void ReadPanoramas(const FileIO& file_io,
                   vector<Panorama>* panoramas) {
  const int num_panoramas = GetNumPanoramas(file_io);
  panoramas->clear();
  panoramas->resize(num_panoramas);
  cerr << "ReadPanoramas:" << flush;
  for (int p = 0; p < num_panoramas; ++p) {
    cerr << '.' << flush;
    panoramas->at(p).Init(file_io, p);
  }
  cerr << endl;
}

void ReadPanoramasWithoutDepths(const FileIO& file_io,
                                vector<Panorama>* panoramas) {
  const int num_panoramas = GetNumPanoramas(file_io);
  panoramas->clear();
  panoramas->resize(num_panoramas);
  cerr << "ReadPanoramasWithoutDepths:" << flush;
  for (int p = 0; p < num_panoramas; ++p) {
    cerr << '.' << flush;
    panoramas->at(p).InitWithoutDepths(file_io, p);
  }
  cerr << endl;
}

void ReadPanoramaPyramids(const FileIO& file_io,
                          const int num_levels,
                          std::vector<std::vector<Panorama> >* panorama_pyramids) {
  const int num_panoramas = GetNumPanoramas(file_io);

  panorama_pyramids->clear();
  panorama_pyramids->resize(num_panoramas);
  cout << "Reading panorama_pyramids" << flush;
  for (int p = 0; p < num_panoramas; ++p) {
    cout << '.' << flush;
    panorama_pyramids->at(p).resize(num_levels);
    for (int level = 0; level < num_levels; ++level) {
      panorama_pyramids->at(p)[level].Init(file_io, p);
      panorama_pyramids->at(p)[level].MakeOnlyBackgroundBlack();
      if (level != 0) {
        const int new_width  = panorama_pyramids->at(p)[level].Width()  / (0x01 << level);
        const int new_height = panorama_pyramids->at(p)[level].Height() / (0x01 << level);
        panorama_pyramids->at(p)[level].Resize(Vector2i(new_width, new_height));
      }
    }
  }
  cout << " done." << endl;
}
  
}  // namespace structured_indoor_modeling
  
