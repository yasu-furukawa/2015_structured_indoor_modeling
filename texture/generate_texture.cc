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
  cerr << "Reading panoramas" << flush;
  for (int p = 0; p < num_panoramas; ++p) {
    cerr << '.' << flush;
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
  cerr << " done." << endl;
}

void ReadPanoramaToGlobals(const file_io::FileIO& file_io,
                           const int num_panoramas,
                           vector<Matrix4d>* panorama_to_globals) {
  panorama_to_globals->resize(num_panoramas);
  for (int p = 0; p < num_panoramas; ++p) {
    const string filename = file_io.GetPanoramaToGlobalTransformation(p);
    ifstream ifstr;
    ifstr.open(filename.c_str());
    if (!ifstr.is_open()) {
      cerr << "Cannot open a file: " << filename << endl;
      exit (1);
    }
    string header;
    ifstr >> header;
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
  }
}

void FindVisiblePanoramas(const std::vector<std::vector<Panorama> >& panoramas,
                          const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                          const Patch& patch,
                          vector<pair<double, int> >* visible_panoramas_weights) {
  const int kFirstLevel = 0;
  visible_panoramas_weights->clear();
  for (int p = 0; p < panoramas.size(); ++p) {
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
        weight += (depth_distance - patch_distance) / panoramas[p][kFirstLevel].GetAverageDistance();
      }
    }
    weight /= kNumSamples1D * kNumSamples1D;
    visible_panoramas_weights->push_back(make_pair(weight, p));
  }

  sort(visible_panoramas_weights->rbegin(), visible_panoramas_weights->rend());

  const int kMinimumKeep = 1;
  const double kVisibilityThreshold = -0.4;
  int num_to_keep;
  for (num_to_keep = kMinimumKeep; num_to_keep < visible_panoramas_weights->size(); ++num_to_keep) {
    if (kVisibilityThreshold < visible_panoramas_weights->at(num_to_keep).first)
      continue;
    else
      break;
  }

  visible_panoramas_weights->resize(num_to_keep);

  const double kOptimalAngle = 70.0 * M_PI / 180.0;
  
  // Find the one with the best vertical angle.
  for (auto& item : *visible_panoramas_weights) {
    const int panorama = item.second;
    const Vector3d center = panoramas[panorama][kFirstLevel].GetCenter();

    Vector3d top    = patch.Interpolate(Vector2d(0, 0));
    Vector3d bottom = patch.Interpolate(Vector2d(0, 1));

    top -= center;
    bottom -= center;
    
    top.normalize();
    bottom.normalize();

    item.first = -fabs(acos(top.dot(bottom)) - kOptimalAngle);
  }
  sort(visible_panoramas_weights->rbegin(), visible_panoramas_weights->rend());
}

  /*
double ComputeTexelSize(const std::vector<std::vector<Panorama> >& panoramas) {
  const int kFirstLevel = 0;
  double average_distance = 0.0;
  for (const auto& panorama : panoramas) {
    average_distance += panorama[kFirstLevel].GetAverageDistance();
  }

  if (panoramas.empty()) {
    cerr << "Empty panoramas." << endl;
    exit (1);
  }
  average_distance /= panoramas.size();

  // 3000 pixels for 360 degree coverage. 120 degrees equal 1000 pixels.
  return average_distance / 512; // / 1024
}
  */

void SetTextureSize(const int max_texture_size_per_patch,
                    const int texture_height_per_wall,
                    Patch* patch) {
  const double patch_width_3d = (patch->vertices[1] - patch->vertices[0]).norm();
  const double patch_height_3d = (patch->vertices[3] - patch->vertices[0]).norm();

  patch->texture_height = texture_height_per_wall;
  patch->texture_width  =
    static_cast<int>(round(patch->texture_height * patch_width_3d / patch_height_3d));

  if (patch->texture_width > max_texture_size_per_patch) {
    patch->texture_height = max_texture_size_per_patch * patch->texture_height / patch->texture_width;
    patch->texture_width = max_texture_size_per_patch;
  }
  
  /*
  patch->texture_width  =
    static_cast<int>(round((patch->vertices[1] - patch->vertices[0]).norm() / texel_size));
  patch->texture_height =
    static_cast<int>(round((patch->vertices[3] - patch->vertices[0]).norm() / texel_size));

  patch->texel_size = texel_size;
  {
    const int max_dimension = max(patch->texture_width, patch->texture_height);
    if (max_dimension > max_texture_size_per_patch) {
      patch->texel_size *= max_dimension / static_cast<double>(max_texture_size_per_patch);
    }
  }

  patch->texture_width  =
    static_cast<int>(round((patch->vertices[1] - patch->vertices[0]).norm() / patch->texel_size));
  patch->texture_height =
    static_cast<int>(round((patch->vertices[3] - patch->vertices[0]).norm() / patch->texel_size));

  patch->texture_width  = max(1, min(max_texture_size_per_patch, patch->texture_width));
  patch->texture_height = max(1, min(max_texture_size_per_patch, patch->texture_height));
  */
}

void GrabTexture(const std::vector<Panorama>& panorama,
                 Patch* patch) {
  const int level = ChoosePyramidLevel(panorama, *patch);
  const Panorama& pano = panorama[level];

  patch->texture.clear();
  for (int y = 0; y < patch->texture_height; ++y) {
    for (int x = 0; x < patch->texture_width; ++x) {
      const Vector3d& point = patch->Interpolate(Vector2d((x + 0.5) / patch->texture_width,
                                                          (y + 0.5) / patch->texture_height));
      const Vector3f rgb = pano.GetRGB(pano.Project(point));
      for (int i = 0; i < 3; ++i) {
        patch->texture.push_back(static_cast<unsigned char>(round(rgb[i])));
      }
    }
  }
}

int ChoosePyramidLevel(const std::vector<Panorama>& panorama,
                       const Patch& patch) {
  const int kLevelZero = 0;
  const Vector3d center = patch.Interpolate(Vector2d(0.5, 0.5));
  const Vector3d center_right = center + patch.x_axis * patch.texel_size;

  const Vector2d center_pixel = panorama[kLevelZero].Project(center);
  const Vector2d center_right_pixel = panorama[kLevelZero].Project(center_right);

  double pixel_diff = (center_right_pixel - center_pixel).norm();
  if (pixel_diff > panorama[kLevelZero].Width() / 2)
    pixel_diff = abs(pixel_diff - panorama[kLevelZero].Width());

  const int num_levels = panorama.size();
  const int level = max(0, min(num_levels - 1, static_cast<int>(floor(log(pixel_diff) / log(2)))));

  return level;
}

void ConvertPatchToMat(const Patch& patch, cv::Mat* mat) {
  mat->create(patch.texture_height, patch.texture_width,CV_8UC3);
  int index = 0;
  for (int y = 0; y < patch.texture_height; ++y)
    for (int x = 0; x < patch.texture_width; ++x)
      for (int i = 0; i < 3; ++i, ++index)
        mat->at<cv::Vec3b>(y, x)[i] = patch.texture[index];
}


void SetFloorPatch(const Floorplan& floorplan,
                   const std::vector<std::vector<Panorama> >& panoramas,
                   const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                   const int max_texture_size_per_floor_patch,
                   std::vector<std::vector<Patch> >* wall_patches) {
  // ????
}

void SetWallPatches(const Floorplan& floorplan,
                    const std::vector<std::vector<Panorama> >& panoramas,
                    const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                    const int max_texture_size_per_wall_patch,
                    const int texture_height_per_wall,
                    std::vector<std::vector<Patch> >* wall_patches) {
  wall_patches->clear();
  wall_patches->resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    wall_patches->at(room).resize(floorplan.GetNumWalls(room));
    for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
      const int next_wall = (wall + 1) % floorplan.GetNumWalls(room);
      Patch& patch = wall_patches->at(room)[wall];
      patch.vertices[0] = Vector3d(floorplan.GetRoomVertexLocal(room, wall)[0],
                                   floorplan.GetRoomVertexLocal(room, wall)[1],
                                   floorplan.GetCeilingHeight(room));
      patch.vertices[1] = Vector3d(floorplan.GetRoomVertexLocal(room, next_wall)[0],
                                   floorplan.GetRoomVertexLocal(room, next_wall)[1],
                                   floorplan.GetCeilingHeight(room));
      patch.vertices[2] = Vector3d(floorplan.GetRoomVertexLocal(room, next_wall)[0],
                                   floorplan.GetRoomVertexLocal(room, next_wall)[1],
                                   floorplan.GetFloorHeight(room));
      patch.vertices[3] = Vector3d(floorplan.GetRoomVertexLocal(room, wall)[0],
                                   floorplan.GetRoomVertexLocal(room, wall)[1],
                                   floorplan.GetFloorHeight(room));

      for (int i = 0; i < 4; ++i)
        patch.vertices[i] = floorplan_to_global * patch.vertices[i];

      patch.x_axis = (patch.vertices[1] - patch.vertices[0]).normalized();
      patch.y_axis = (patch.vertices[3] - patch.vertices[0]).normalized();
      
      // Identify visible panoramas.
      vector<pair<double, int> > visible_panoramas_weights;
      FindVisiblePanoramas(panoramas, global_to_panoramas, patch, &visible_panoramas_weights);
      
      // Keep the best one only.
      const int best_panorama = visible_panoramas_weights[0].second;
      SetTextureSize(FLAGS_max_texture_size_per_wall_patch, FLAGS_texture_height_per_wall, &patch);
      // Grab texture.
      GrabTexture(panoramas[best_panorama], &patch);

      /*
      cv::Mat patch_mat;
      ConvertPatchToMat(patch, &patch_mat);
      cv::imshow("Patch.", patch_mat);
      cv::waitKey(0);
      */
    }
  }
}

void PackFloorTexture(const Patch& floor_patch,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::pair<int, Eigen::Vector2i>* iuv) {
  

}
  
void PackWallTextures(const std::vector<std::vector<Patch> >& patches,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::pair<int, Eigen::Vector2i>* iuv) {  
  for (int room = 0; room < patches.size(); ++room) {
    for (int wall = 0; wall < patches[room].size(); ++wall) {
      const Patch& patch = patches[room][wall];
      const WallTriangulation& wall_triangulation = floorplan->GetWallTriangulation(room, wall);

      PackWallTexture(patch, wall_triangulation, texture_image_size, iuv);


    }
  }
}

void PackWallTexture(const Patch& patch,
                     const WallTriangulation& wall_triangulation,
                     const int texture_image_size,
                     std::pair<int, Eigen::Vector2i>* iuv) {
  // Checks if the texture fits into the current image.
  Eigen::Vector2i new_uv = iuv->second + size;
  if (new_uv[0] <= texture_image_size && new_uv[1] <= texture_image_size) {
    triangle->image_index = iuv->first;

    

    triangle->
  }
  
}
  

void CopyTextures(const Floorplan& floorplan,
                  const std::vector<std::vector<Patch> >& patches,
                  std::vector<cv::Mat>* texture_images) {



}

}  // namespace texture
