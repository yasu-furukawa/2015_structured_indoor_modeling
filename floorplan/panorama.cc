#include "panorama.h"

using namespace Eigen;

Panorama::Panorama();

void Panorama::Init(const file_io::FileIO& file_io,
                    const int panorama) {
  rgb_image.load(file_io.GetPanoramaImage(panorama));
  if (rgb_image.isNull()) {
    cerr << "Panorama image cannot be loaded: " << file_io.GetPanoramaImage(panorama) << endl;
    exit (1);
  }
  width  = rgb_image.cols();
  height = rgb_image.rows();

  InitDepthImage(file_io, panorama);
  InitCameraParameters(file_io, panorama);


  phi_per_pixel = phi_range / height;
  phi_per_depth_pixel = phi_range / depth_height;
}

Eigen::Vector2d Panorama::Project(const Eigen::Vector3d& global) const {


}

Eigen::Vector3d Panorama::Unproject(const Eigen::Vector2d& pixel,
                                    const double distance) const {

}

Eigen::Vector3d Panorama::GlobalToLocal(const Eigen::Vector3d& global_xyz) const {
  Vector4d xyz1(global_xyz[0], global_xyz[1], global_xyz[2], 1.0);
  
  

  Vector3d projected_coordinate = global_to_local * (xyz - center);
  // x coordinate.
  double theta = -atan2(projected_coordinate.y(), projected_coordinate.x());
  if (theta < 0.0)
    theta += 2 * M_PI;
  double theta_ratio = max(0.0, min(1.0, theta / (2 * M_PI)));
  if (theta_ratio == 1.0)
    theta_ratio = 0.0;

  Vector2d uv;
  uv[0] = theta_ratio * rgb_image.width();
  const double depth = sqrt(projected_coordinate.x() * projected_coordinate.x() +
                           projected_coordinate.y() * projected_coordinate.y());
  double phi = atan2(projected_coordinate.z(), depth);
  const double pixel_offset_from_center = phi / phi_per_pixel;
  uv[1] = rgb_image.height() / 2.0 - pixel_offset_from_center;

  return uv;

  
}

Eigen::Vector3d Panorama::LocalToGlobal(const Eigen::Vector3d& local_xyz) const {

}

Eigen::Vector2d Panorama::RGBToDepth(const Eigen::Vector2d& pixel) const {

}

Eigen::Vector2d Panorama::DepthToRGB(const Eigen::Vector2d& depth_pixel) const {

}

Eigen::Vector3f Panorama::GetRGB(const Eigen::Vector2d& pixel) const {

}

double Panorama::GetDepth(const Eigen::Vector2d& depth_pixel) const {

}

void Panorama::InitDepthImage(const file_io::FileIO& file_io,
                              const int panorama) {
  ifstream ifstr;
  ifstr.open(file_io.GetDepthPanorama(panorama));

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
  
void Panorama::InitCameraParameters(const file_io::FileIO& file_io,
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
    center(y) = local_to_global(y, 3);
  }

  const Matrix3d rotation = local_to_global.block(0, 0, 3, 3);
  global_to_local.block(0, 0, 3, 3) = rotation.transpose();
  global_to_local.block(0, 3, 3, 1) =
    - rotation.transpose() * local_to_global.block(0, 3, 3, 1);
  global_to_local(3, 0) = 0.0;
  global_to_local(3, 1) = 0.0;
  global_to_local(3, 2) = 0.0;
  global_to_local(3, 3) = 1.0;
    
  ifstr >> phi_range;
  
  ifstr.close();
}  
