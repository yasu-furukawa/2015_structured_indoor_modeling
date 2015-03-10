#include <Eigen/Dense>

using namespace Eigen;

struct CameraParameters {
  Eigen::Vector3d center;
  int width;
  int height;
  
  double phi_range;
};


Eigen::Vector2d Project(const CameraParameters& camera_parameters,
                        const Eigen::Vector3d& global) {  
  const Vector3d local = global - center;
  const double phi_per_pixel =
    camera_parameters.phi_range / camera_parameters.height;

  // x coordinate.
  double theta = -atan2(local.y(), local.x());
  if (theta < 0.0)
    theta += 2 * M_PI;
  double theta_ratio = max(0.0, min(1.0, theta / (2 * M_PI)));
  if (theta_ratio == 1.0)
    theta_ratio = 0.0;

  Vector2d uv;
  uv[0] = theta_ratio * camera_parameters.width;
  const double depth = sqrt(local.x() * local.x() +
                            local.y() * local.y());
  double phi = atan2(local.z(), depth);
  const double pixel_offset_from_center = phi / phi_per_pixel;
  // uv[1] = height / 2.0 - pixel_offset_from_center;
  uv[1] = max(0.0, min(camera_parameters.height - 1.1,
                       camera_parameters.height / 2.0 -
                       pixel_offset_from_center));

  return uv;
}


int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[1] << " pano_centers.txt" << endl;
    exit (1);
  }

  ifstream ifstr;
  ifstr.open(argv[1]);
  if (!ifstr.is_open()) {
    cerr << "Cannot open a file." << argv[1] << endl;
    exit (1);
  }
  vector<CameraParameters> camera_parameters;
  while (true) {
    CameraParameters parameters;
    for (int i = 0; i < 3; ++i)
      ifstr >> parameters.center[i];
    if (ifstr.eof())
      break;

    parameters.width  = kWidth;
    parameters.height = kHeight;
    parameters.kPhiRange = kPhiRange;
    camera_parameters.push_back(parameters);
  }

  // Read a 3D model.
  vector<Vecor3dL> vertices;
  vectorVector3i> triangles;
  ReadMesh(

}

  
