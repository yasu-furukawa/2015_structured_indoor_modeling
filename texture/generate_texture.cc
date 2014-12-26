#include <fstream>
#include <iostream>

#include "generate_texture.h"
#include "../calibration/file_io.h"
#include "../floorplan/panorama.h"

using namespace Eigen;
using namespace std;

namespace texture {

void ReadPanoramas(const file_io::FileIO& file_io,
                   const int num_panoramas,
                   const int num_pyramid_levels,
                   vector<vector<Panorama> >* panoramas) {
  panoramas->resize(num_panoramas);
  for (int p = 0; p < num_panoramas; ++p) {
    panoramas->at(p).resize(num_pyramid_levels);
    for (int level = 0; level < num_pyramid_levels; ++level) {
      panoramas->at(p)[level].Init(file_io, p);
      if (level != 0) {
        const int new_width  = panoramas->at(p)[level].Width()  / (0x01 << level);
        const int new_height = panoramas->at(p)[level].Height() / (0x01 << level);
        panoramas->at(p)[level].ResizeRGB(Vector2i(new_width, new_height));
      }
    }
  }
}

void ReadPanoramaToGlobals(const file_io::FileIO& file_io,
                           const int num_panoramas,
                           vector<Matrix4d>* panorama_to_globals) {
  panorama_to_globals->resize(num_panoramas);
  for (int p = 0; p < num_panoramas; ++p) {
    const string filename = file_io.GetPanoramaToGlobalTransformation(p);
    ifstream ifstr;
    ifstr.open(filename.c_str());
    string header;
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        ifstr >> (*panorama_to_globals)[p](y, x);
      }
    }
    ifstr.close();
  }
}

void Invert(const vector<Matrix4d>& panorama_to_globals,
            vector<Matrix4d>* global_to_panoramas) {
  global_to_panoramas->resize(panorama_to_globals.size());
  for (int i = 0; i < panorama_to_globals.size(); ++i) {
    Matrix3d rotation = panorama_to_globals[i].block(0, 0, 3, 3);
    global_to_panoramas->at(i).block(0, 0, 3, 3) = rotation.transpose();
    global_to_panoramas->at(i).block(0, 3, 3, 1) =
      - rotation.transpose() * panorama_to_globals[i].block(0, 3, 3, 1);
    global_to_panoramas->at(i)(3, 0) = 0.0;
    global_to_panoramas->at(i)(3, 1) = 0.0;
    global_to_panoramas->at(i)(3, 2) = 0.0;
    global_to_panoramas->at(i)(3, 3) = 1.0;
    
    // debug.
    Matrix4d a = global_to_panoramas->at(i) * panorama_to_globals[i];
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x)
        cout << a(y, x) << ' ';
      cout << endl;
    }
  }
}

void FindVisiblePanoramas(const std::vector<std::vector<Panorama> >& panoramas,
                          const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                          const Patch& patch,
                          vector<pair<double, int> >* visible_panoramas_weights) {
  visible_panoramas_weights->clear();
  for (int p = 0; p < panoramas.size(); ++p) {
    const int kFirstLevel = 0;
    const Vector3d& center = panoramas[p][kFirstLevel].GetCenter();

    double weight = 0.0;
    // Sample points on the patch, and check the visibility for each panorama.
    const int kNumSamples1D = 5;
    for (int j = 0; j < kNumSamples1D; ++j) {
      const double v = (j + 0.5) / kNumSamples1D;
      for (int i = 0; i < kNumSamples1D; ++i) {
        const double u = (i + 0.5) / kNumSamples1D;
        const Vector3d sample = patch.Interpolate(Vector2d(u, v));

        const double patch_distance = (center - sample).norm();

        const Vector2d pixel = panoramas[p][kFirstLevel].Project(sample);
        const Vector2d depth_pixel = panoramas[p][kFirstLevel].RGBToDepth(pixel);
        const double depth_distance = panoramas[p][kFirstLevel].GetDepth(depth_pixel);

        weight += depth_distance - patch_distance;
      }
    }
    visible_panoramas_weights->push_back(make_pair(weight, p));
  }

  sort(visible_panoramas_weights->rbegin(), visible_panoramas_weights->rend());
}

double ComputeTexelSize(const std::vector<std::vector<Panorama> >& panoramas) {
  const int kFirstLevel = 0;
  double average_distance = 0.0;
  for (const auto& item : panoramas) {
    average_distance += panoramas[kFirstLevel].GetAverageDistance();
  }

  if (panoramas.empty()) {
    cerr << "Empty panoramas." << endl;
    exit (1);
  }
  average_distance /= panoramas.size();

  // 3000 pixels for 360 degree coverage. 90 degrees equal 750 pixels.
  return average_distance / 750;
}

void SetTextureSize(const double texel_size,
                    const int max_texture_size_per_patch,
                    Patch* patch) {
  patch->texture_width  =
    static_cast<int>(round((patch->vertices[1] - patch->vertices[0]).norm() / texel_size));
  patch->texture_height =
    static_cast<int>(round((patch->vertices[3] - patch->vertices[0]).norm() / texel_size));

  patch->texel_size = texel_size;
  {
    const int max_dimension = max(patch->texture_width, patch->texture_height);
    if (max_dimension > max_texture_size_per_match) {
      patch->texel_size *= max_dimension / static_cast<double>(max_texture_size_per_match);
    }
  }

  patch->texture_width  =
    static_cast<int>(round((patch->vertices[1] - patch->vertices[0]).norm() / texel_size));
  patch->texture_height =
    static_cast<int>(round((patch->vertices[3] - patch->vertices[0]).norm() / texel_size));
  
  patch->texture_width  = max(1, min(max_texture_size_per_patch, patch->texture_width));
  patch->texture_height = max(1, min(max_texture_size_per_patch, patch->texture_height));
}

void GrabTexture(const std::vector<Panorama>& panorama,
                 Patch* patch) {
  const int level = ChoosePyramidLevel(panorama, *patch);
  const Panorama& pano = panorama[level];

  patch->texture.clear();
  for (int y = 0; y < patch->texture_height; ++y) {
    for (int x = 0; x < patch->texture_width; ++x) {
      const Vector3d& point = patch->Interpolate((x + 0.5) / patch->texture_width,
                                                 (y + 0.5) / patch->texture_height);
      const Vecto3f rgb = pano.GetRGB(pano.Project(point));
      Matrix<unsigned char, 3, 1> color;
      for (int i = 0; i < 3; ++i) {
        color[i] = static_cast<unsigned char>(round(rgb[i] * 255.0));
      }
      patch->texture.push_back(color);
    }
  }
}

int ChoosePyramidLevel(const std::vector<Panorama>& panorama,
                       const Patch& patch) {
  const int kLevelZero = 0;
  const Vector3d center = patch.Interpolate(Vector2d(0.5, 0.5));
  const Vector3d center_right = center + patch.x_axis * patch.texel_size;

  const Vector2d center_pixel = patch.Project(center);
  const Vector2d center_right = patch.Project(center_right);
  int pixel_diff = abs(center_right - center_pixel);
  if (pixel_diff > panorama[kLevelZero].Width() / 2)
    pixel_diff = abs(pixel_diff - panorama[kLevelZero].Width());

  const int num_levels = panorama.size();
  const int level = max(0, min(num_levels - 1, static_cast<int>(floor(log(pixel_diff) / log(2)))));

  return level;
}
  
}  // namespace texture
