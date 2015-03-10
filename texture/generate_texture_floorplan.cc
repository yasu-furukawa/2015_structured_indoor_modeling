#include <fstream>
#include <iostream>

#include "generate_texture_floorplan.h"
#include "synthesize.h"
#include "../base/imageProcess/morphological_operation.h"
#include "../base/point_cloud.h"
#include "../base/floorplan.h"
#include "../base/file_io.h"
#include "../base/panorama.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

void FindVisiblePanoramas(const std::vector<std::vector<Panorama> >& panoramas,
                          const Patch& patch,
                          vector<pair<double, int> >* visible_panoramas_weights);

void SetTextureSize(const int max_texture_size_per_patch,
                    const int texture_height_per_wall,
                    Patch* patch);

void GrabTexture(const std::vector<Panorama>& panorama,
                 Patch* patch);

int ChoosePyramidLevel(const std::vector<Panorama>& panorama,
                       const Patch& patch);

void ConvertPatchToMat(const Patch& patch, cv::Mat* mat);

void PackWallTexture(const Patch& patch,
                     const int texture_image_size,
                     WallTriangulation* wall_triangulation,
                     std::vector<std::vector<unsigned char> >* texture_images,
                     std::pair<int, Eigen::Vector2i>* iuv,
                     int* max_texture_height);
  
void UpdateIUV(const Eigen::Vector2i& size,
               const int texture_image_size,
               std::pair<int, Eigen::Vector2i>* iuv,
               int* max_texture_height);

void CopyTexel(const Patch& patch,
               const int texture_image_size,
               const std::pair<int, Eigen::Vector2i>& iuv,
               std::vector<std::vector<unsigned char> >* texture_images);
  
void SetIUVInWall(const Vector2i& texture_size,
                  const int texture_image_size,
                  const std::pair<int, Eigen::Vector2i>& iuv,
                  WallTriangulation* wall_triangulation);

void MarkWalls(const Floorplan& floorplan,
               const Patch& floor_patch,
               const Vector2d& min_xy_local,
               const Vector2d& max_xy_local,
               const int wall,
               const int id_width,
               vector<int>* panorama_id_for_texture_mapping);

void SetManhattanDelaunay(const Floorplan& floorplan,
                          const std::vector<std::vector<Panorama> >& panoramas,
                          const Patch& floor_patch,
                          const Eigen::Vector2d& min_xy_local,
                          const Eigen::Vector2d& max_xy_local,
                          vector<int>* panorama_id_for_texture_mapping);

void SetIUVInFloor(const Patch& floor_patch,
                   const int texture_image_size,
                   const std::pair<int, Eigen::Vector2i>& iuv,
                   Floorplan* floorplan);

void SetBoundingBox(const Floorplan& floorplan,
                    Eigen::Vector2d* min_xy_local,
                    Eigen::Vector2d* max_xy_local);

void SetRoomSegments(const Floorplan& floorplan,
                     const Patch& floor_patch,
                     const unsigned char background,
                     cv::Mat* room_segments);

void ComputeAverageFloorCeilingHeights(const Floorplan& floorplan,
                                       double* average_floor_height,
                                       double* average_ceiling_height);
  
Eigen::Vector2i ComputeTextureSize(const Eigen::Vector2d& min_xy_local,
                                   const Eigen::Vector2d& max_xy_local,
                                   const int max_texture_size_per_floor_patch);
  
Eigen::Vector2d ConvertLocalToTexel(const Eigen::Vector2d& local,
                                    const Patch& patch);

Eigen::Vector2i ConvertLocalToTexelInt(const Eigen::Vector2d& local,
                                       const Patch& patch);

Eigen::Vector2d ConvertLocalToTexel(const Eigen::Vector2d& local,
                                    const Eigen::Vector2d& min_xy_local,
                                    const Eigen::Vector2d& max_xy_local,
                                    const Eigen::Vector2i& texture_size);

Eigen::Vector2i ConvertLocalToTexelInt(const Eigen::Vector2d& local,
                                       const Eigen::Vector2d& min_xy_local,
                                       const Eigen::Vector2d& max_xy_local,
                                       const Eigen::Vector2i& texture_size);
  
int FindClosestPanoramaToRoom(const TextureInput& texture_input,
                              const Patch& floor_patch,
                              const cv::Mat& room_segments,
                              const int room);

void ComputeProjectedTextures(const TextureInput& texture_input,
                              const Patch& floor_patch,
                              const std::vector<double>& floor_heights,
                              const std::vector<double>& ceiling_heights,
                              std::vector<cv::Mat>* projected_textures);
  
void GenerateFloorTexture(const int room,
                          const TextureInput& texture_input,
                          const std::vector<cv::Mat>& projected_textures,
                          const cv::Mat& room_segments,
                          const Patch& floor_patch,
                          cv::Mat* floor_texture);

void SynthesizePatch(Patch* patch);

void ShrinkTexture(const int shrink_pixels, Patch* patch);

}  // namespace

bool IsOnFloor(const Floorplan& floorplan,
               const Patch& floor_patch,
               const vector<double>& floor_heights,
               const vector<double>& ceiling_heights,
               const Point& point,
               const double position_error) {
  const Vector3d local_position = floorplan.GetFloorplanToGlobal().transpose() * point.position;
  const Vector2d texture_position = floor_patch.LocalToTexture(Vector2d(local_position[0],
                                                                        local_position[1]));
  const int x = static_cast<int>(round(texture_position[0]));
  const int y = static_cast<int>(round(texture_position[1]));
  if (x < 0 || floor_patch.texture_size[0] <= x || y < 0 || floor_patch.texture_size[1] <= y)
    return false;

  const int index = y * floor_patch.texture_size[0] + x;
  const double floor_height   = floor_heights[index];
  const double ceiling_height = ceiling_heights[index];  
  const double margin = (ceiling_height - floor_height) * position_error;

  if (fabs(local_position[2] - floor_height) > margin)
    return false;

  return true;
}
  
void SetFloorPatch(const TextureInput& texture_input, Patch* floor_patch) {
  const Floorplan& floorplan = texture_input.floorplan;
  const std::vector<std::vector<Panorama> >& panoramas = texture_input.panoramas;
  const std::vector<PointCloud>& point_clouds = texture_input.point_clouds;
  
  SetBoundingBox(floorplan, &floor_patch->min_xy_local, &floor_patch->max_xy_local);

  double average_floor_height, average_ceiling_height;
  ComputeAverageFloorCeilingHeights(floorplan,
                                    &average_floor_height,
                                    &average_ceiling_height);

  floor_patch->InitVertices(average_floor_height);

  floor_patch->texture_size =
    ComputeTextureSize(floor_patch->min_xy_local, floor_patch->max_xy_local,
                       texture_input.max_texture_size_per_floor_patch);

  // Compute a room segmentation.
  cv::Mat room_segments;
  const unsigned char kBackground = 255;
  SetRoomSegments(floorplan, *floor_patch, kBackground, &room_segments);
  
  vector<double> floor_heights(floor_patch->texture_size[0] * floor_patch->texture_size[1], 0);
  vector<double> ceiling_heights(floor_patch->texture_size[0] * floor_patch->texture_size[1], 0);

  int index = 0;
  for (int y = 0; y < floor_patch->texture_size[1]; ++y) {
    for (int x = 0; x < floor_patch->texture_size[0]; ++x, ++index) {
      const int room = room_segments.at<unsigned char>(y, x);
      if (room == kBackground)
        continue;
      
      floor_heights[index]   = floorplan.GetFloorHeight(room);
      ceiling_heights[index] = floorplan.GetCeilingHeight(room);
    }
  }
 
  vector<cv::Mat> projected_textures;
  cout << "Compute projected texture..." <<flush;
  ComputeProjectedTextures(texture_input, *floor_patch, floor_heights, ceiling_heights,
                           &projected_textures);
  cout << "done."<< endl;

  // For each room, put texture.
  // const int kLevel = 0;
  cv::Mat floor_texture(floor_patch->texture_size[1], floor_patch->texture_size[0],
                        CV_8UC3, cv::Scalar(0));
  
  cout << floorplan.GetNumRooms() << " rooms: " << flush;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    cout << room << '.' << flush;
    GenerateFloorTexture(room, texture_input, projected_textures, room_segments,
                         *floor_patch, &floor_texture);
  }
  cout << "done." << endl;

  floor_patch->texture.clear();
  for (int y = 0; y < floor_patch->texture_size[1]; ++y) {
    for (int x = 0; x < floor_patch->texture_size[0]; ++x) {
      const cv::Vec3b& color = floor_texture.at<cv::Vec3b>(y, x);
      for (int c = 0; c < 3; ++c)
        floor_patch->texture.push_back(color[c]);
    }
  }
}  

void SetWallPatches(const TextureInput& texture_input,
                    std::vector<std::vector<Patch> >* wall_patches) {
  const Floorplan& floorplan = texture_input.floorplan;
    
  wall_patches->clear();
  wall_patches->resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    cout << room << '/' << floorplan.GetNumRooms() << ' ' << flush;
    wall_patches->at(room).resize(floorplan.GetNumWalls(room));
    for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
      const int next_wall = (wall + 1) % floorplan.GetNumWalls(room);
      Patch& patch = wall_patches->at(room)[wall];
      patch.vertices[0] = floorplan.GetCeilingVertexGlobal(room, wall);
      patch.vertices[1] = floorplan.GetCeilingVertexGlobal(room, next_wall);
      patch.vertices[2] = floorplan.GetFloorVertexGlobal(room, next_wall);
      patch.vertices[3] = floorplan.GetFloorVertexGlobal(room, wall);

      // Identify visible panoramas.
      vector<pair<double, int> > visible_panoramas_weights;
      FindVisiblePanoramas(texture_input.panoramas, patch, &visible_panoramas_weights);
      
      // Keep the best one only.
      const int best_panorama = visible_panoramas_weights[0].second;
      SetTextureSize(texture_input.max_texture_size_per_wall_patch,
                     texture_input.texture_height_per_wall,
                     &patch);
      // Grab texture.
      GrabTexture(texture_input.panoramas[best_panorama], &patch);

      const int kShrinkPixels = 6;
      ShrinkTexture(kShrinkPixels, &patch);

      bool hole = false;
      for (int i = 0; i < patch.texture.size(); i+=3) {
        if (patch.texture[i] == 0 &&
            patch.texture[i + 1] == 0 &&
            patch.texture[i + 2] == 0) {
          hole = true;
          break;
        }
      }
      if (hole)
        SynthesizePatch(&patch);
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
                      std::vector<std::vector<unsigned char> >* texture_images,
                      std::pair<int, Eigen::Vector2i>* iuv,
                      int* max_texture_height) {
  UpdateIUV(floor_patch.texture_size, texture_image_size, iuv, max_texture_height);
  // Copy texel data.
  CopyTexel(floor_patch, texture_image_size, *iuv, texture_images);
  // Update IUV in floorplan.
  SetIUVInFloor(floor_patch, texture_image_size, *iuv, floorplan);
  // Update iuv.
  iuv->second[0] += floor_patch.texture_size[0];
}
  
void PackWallTextures(const std::vector<std::vector<Patch> >& patches,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::vector<std::vector<unsigned char> >* texture_images,
                      std::pair<int, Eigen::Vector2i>* iuv,
                      int* max_texture_height) {
  for (int room = 0; room < patches.size(); ++room) {
    for (int wall = 0; wall < patches[room].size(); ++wall) {
      const Patch& patch = patches[room][wall];
      WallTriangulation& wall_triangulation = floorplan->GetWallTriangulation(room, wall);
      PackWallTexture(patch, texture_image_size,
                      &wall_triangulation, texture_images, iuv, max_texture_height);
    }
  }
}

void WriteTextureImages(const FileIO& file_io,
                        const int texture_image_size,
                        const std::vector<std::vector<unsigned char> >& texture_images) {
  for (int t = 0; t < texture_images.size(); ++t) {
    cv::Mat image(texture_image_size, texture_image_size, CV_8UC3);
    int index = 0;
    for (int y = 0; y < texture_image_size; ++y) {
      for (int x = 0; x < texture_image_size; ++x) {
        for (int i = 0; i < 3; ++i) {
          image.at<cv::Vec3b>(y, x)[i] = texture_images[t][index++];
        }
      }
    }

    imwrite(file_io.GetTextureImage(t), image);
  }
}


//----------------------------------------------------------------------  
namespace {
void FindVisiblePanoramas(const std::vector<std::vector<Panorama> >& panoramas,
                          const Patch& patch,
                          vector<pair<double, int> >* visible_panoramas_weights) {
  const Vector3d patch_normal =
    (patch.vertices[3] - patch.vertices[0]).cross(patch.vertices[1] - patch.vertices[0]).normalized();
  const double kHolePenalty = 0.5;
  const double kNormalScale = 0.5;
  const int kFirstLevel = 0;
  visible_panoramas_weights->clear();
  for (int p = 0; p < panoramas.size(); ++p) {
    const Vector3d& center = panoramas[p][kFirstLevel].GetCenter();
    double weight = 0.0;
    // Sample points on the patch, and check the visibility for each panorama.
    const int kNumSamples1D = 10;
    for (int j = 0; j < kNumSamples1D; ++j) {
      const double v = (j + 0.5) / kNumSamples1D;
      for (int i = 0; i < kNumSamples1D; ++i) {
        const double u = (i + 0.5) / kNumSamples1D;
        const Vector3d sample = patch.Interpolate(Vector2d(u, v));
        Vector3d diff = center - sample;
        const double patch_distance = diff.norm();

        const Vector2d pixel = panoramas[p][kFirstLevel].Project(sample);
        if (panoramas[p][kFirstLevel].GetRGB(pixel) == Vector3f(0, 0, 0)) {
          weight -= kHolePenalty;
        }

        const double dot = patch_normal.dot(diff.normalized());
        weight += kNormalScale * dot;

        const Vector2d depth_pixel = panoramas[p][kFirstLevel].RGBToDepth(pixel);
        const double depth_distance = panoramas[p][kFirstLevel].GetDepth(depth_pixel);
        weight += min(0.0,
                      (depth_distance - patch_distance) / panoramas[p][kFirstLevel].GetAverageDistance());
      }
    }
    weight /= kNumSamples1D * kNumSamples1D;
    visible_panoramas_weights->push_back(make_pair(weight, p));
  }
  
  sort(visible_panoramas_weights->rbegin(), visible_panoramas_weights->rend());
}

void SetTextureSize(const int max_texture_size_per_patch,
                    const int texture_height_per_wall,
                    Patch* patch) {
  const double patch_width_3d = (patch->vertices[1] - patch->vertices[0]).norm();
  const double patch_height_3d = (patch->vertices[3] - patch->vertices[0]).norm();

  patch->texture_size[1] = texture_height_per_wall;
  patch->texture_size[0]  =
    static_cast<int>(round(patch->texture_size[1] * patch_width_3d / patch_height_3d));

  if (patch->texture_size[0] > max_texture_size_per_patch) {
    patch->texture_size[1] = max_texture_size_per_patch * patch->texture_size[1] / patch->texture_size[0];
    patch->texture_size[0] = max_texture_size_per_patch;
  }
}

void GrabTexture(const std::vector<Panorama>& panorama,
                 Patch* patch) {
  const int level = ChoosePyramidLevel(panorama, *patch);
  const Panorama& pano = panorama[level];

  patch->texture.clear();
  for (int y = 0; y < patch->texture_size[1]; ++y) {
    for (int x = 0; x < patch->texture_size[0]; ++x) {
      const Vector3d& point = patch->Interpolate(Vector2d((x + 0.5) / patch->texture_size[0],
                                                          (y + 0.5) / patch->texture_size[1]));
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
  const Vector3d center_right = patch.Interpolate(Vector2d(0.5 + 1.0 / patch.texture_size[0], 0.5));

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
  mat->create(patch.texture_size[1], patch.texture_size[0], CV_8UC3);
  int index = 0;
  for (int y = 0; y < patch.texture_size[1]; ++y)
    for (int x = 0; x < patch.texture_size[0]; ++x)
      for (int i = 0; i < 3; ++i, ++index)
        mat->at<cv::Vec3b>(y, x)[i] = patch.texture[index];
}
  
void PackWallTexture(const Patch& patch,
                     const int texture_image_size,
                     WallTriangulation* wall_triangulation,
                     std::vector<std::vector<unsigned char> >* texture_images,
                     std::pair<int, Eigen::Vector2i>* iuv,
                     int* max_texture_height) {
  UpdateIUV(patch.texture_size, texture_image_size, iuv, max_texture_height);
  // Copy texel data.
  CopyTexel(patch, texture_image_size, *iuv, texture_images);
  // Update IUV in floorplan (wall_triangulation).
  SetIUVInWall(patch.texture_size, texture_image_size, *iuv, wall_triangulation);
  // Update iuv.
  iuv->second[0] += patch.texture_size[0];
}

void UpdateIUV(const Eigen::Vector2i& size,
               const int texture_image_size,
               std::pair<int, Eigen::Vector2i>* iuv,
               int* max_texture_height) {
  // New row.
  if (iuv->second[0] + size[0] > texture_image_size) {
    iuv->second[0] = 0;
    iuv->second[1] += *max_texture_height;
    *max_texture_height = 0;
  }
  
  // New image.
  if (iuv->second[1] + size[1] > texture_image_size) {
    ++iuv->first;
    iuv->second[0] = 0;
    iuv->second[1] = 0;
    *max_texture_height = 0;
  }

  // Add a patch.
  *max_texture_height = max(*max_texture_height, size[1]);
}

void CopyTexel(const Patch& patch,
               const int texture_image_size,
               const std::pair<int, Eigen::Vector2i>& iuv,
               std::vector<std::vector<unsigned char> >* texture_images) {
  const int kNumChannels = 3;
  if (iuv.first >= texture_images->size()) {
    texture_images->push_back(vector<unsigned char>());
    texture_images->back().resize(kNumChannels * texture_image_size * texture_image_size);
  }
  
  const int index = iuv.first;
  const Vector2i& start_uv = iuv.second;

  if (texture_images->size() <= index) {
    cerr << "Impossible." << endl;
    exit (1);
  }

  for (int y = 0; y < patch.texture_size[1]; ++y) {
    const int ytmp = y + start_uv[1];
    for (int x = 0; x < patch.texture_size[0]; ++x) {
      const int xtmp = x + start_uv[0];
      for (int c = 0; c < kNumChannels; ++c) {
        const int texture_index = kNumChannels * (ytmp * texture_image_size + xtmp);
        const int patch_index = kNumChannels * (y * patch.texture_size[0] + x);
        texture_images->at(index)[texture_index + c] = patch.texture[patch_index + c];
      }
    }
  }
}

void SetIUVInWall(const Vector2i& texture_size,
                  const int texture_image_size,
                  const std::pair<int, Eigen::Vector2i>& iuv,
                  WallTriangulation* wall_triangulation) {
  Vector2d top_left_uv(iuv.second[0], iuv.second[1]);
  Vector2d bottom_right_uv(iuv.second[0] + texture_size[0],
                           iuv.second[1] + texture_size[1]);

  // Convert to [0, 1].
  top_left_uv /= texture_image_size;
  bottom_right_uv /= texture_image_size;
  Vector2d uv_diff = bottom_right_uv - top_left_uv;
  
  for (auto& triangle : wall_triangulation->triangles) {
    triangle.image_index = iuv.first;
    for (int i = 0; i < 3; ++i) {
      Vector2d uv_in_wall = wall_triangulation->vertices_in_uv[triangle.indices[i]];
      // May need to change depending on the definition of uv(0, 0).
      triangle.uvs[i] = top_left_uv + Vector2d(uv_in_wall[0] * uv_diff[0],
                                               (1.0 - uv_in_wall[1]) * uv_diff[1]);

      for (int j = 0; j < 2; ++j)
        triangle.uvs[i][j] = min(1.0, triangle.uvs[i][j]);
    }
  }
}                

void MarkWalls(const Floorplan& floorplan,
               const Patch& floor_patch,
               const Vector2d& min_xy_local,
               const Vector2d& max_xy_local,
               const int wall,
               const int id_width,
               vector<int>* panorama_id_for_texture_mapping) {
  const Vector2d xy_diff = max_xy_local - min_xy_local;
  
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
      const int next_vertex = (vertex + 1) % floorplan.GetNumRoomVertices(room);
      const Vector2d start_local = floorplan.GetRoomVertexLocal(room, vertex);
      const Vector2d end_local = floorplan.GetRoomVertexLocal(room, next_vertex);
      const int step_num = max(1, static_cast<int>(round(2.0 * (end_local - start_local).norm())));

      const Vector2d unit = (end_local - start_local) / step_num;
      for (int i = 0; i <= step_num; ++i) {
        const Vector2d pos = start_local + i * unit;

        const Vector2d diff = pos - min_xy_local;

        const double u = diff[0] / xy_diff[0];
        const double v = diff[1] / xy_diff[1];
        
        const int x = min(floor_patch.texture_size[0] - 1,
                          static_cast<int>(round(u * floor_patch.texture_size[0])));
        const int y = min(floor_patch.texture_size[1] - 1,
                          static_cast<int>(round(v * floor_patch.texture_size[1])));

        panorama_id_for_texture_mapping->at(y * id_width + x) = wall;
      }
    }
  }
}

void SetManhattanDelaunay(const Floorplan& floorplan,
                          const std::vector<std::vector<Panorama> >& panoramas,
                          const Patch& floor_patch,
                          const Eigen::Vector2d& min_xy_local,
                          const Eigen::Vector2d& max_xy_local,
                          vector<int>* panorama_id_for_texture_mapping) {
  const int kLevelZero = 0;

  const int kFirstRoom = 0;
  const double floor_height = floorplan.GetFloorHeight(kFirstRoom);
  
  // Minimum valid angle to see texture.
  const double kMinimumValidAngle = 40.0 * M_PI / 180.0;

  vector<Vector3d> panorama_centers_local(panoramas.size());
  vector<Vector3d> panorama_centers_on_floor_local(panoramas.size());
  for (int p = 0; p < panoramas.size(); ++p) {
    panorama_centers_local[p] =
      floorplan.GetFloorplanToGlobal().transpose() * panoramas[p][kLevelZero].GetCenter();
    panorama_centers_on_floor_local[p] = panorama_centers_local[p];
    panorama_centers_on_floor_local[p][2] = floor_height;
  }

  const Vector2d xy_diff = max_xy_local - min_xy_local;
  for (int y = 0; y < floor_patch.texture_size[1]; ++y) {
    for (int x = 0; x < floor_patch.texture_size[0]; ++x) {
      const Vector2d local(min_xy_local[0] + xy_diff[0] * x / floor_patch.texture_size[0],
                           min_xy_local[1] + xy_diff[1] * y / floor_patch.texture_size[1]);
      const Vector3d floor_point(local[0], local[1], floor_height);
                           
      int best_panorama = -1;
      double best_distance = 0.0;
      for (int p = 0; p < panoramas.size(); ++p) {
        // Angle.
        const Vector3d diff0 = floor_point - panorama_centers_local[p];
        Vector3d diff0_x(diff0[0], 0.0, diff0[2]);
        Vector3d diff0_y(0.0, diff0[1], diff0[2]);

        Vector3d diff1 = panorama_centers_on_floor_local[p] - panorama_centers_local[p];

        const double distance = max(fabs(diff0[0]), fabs(diff0[1]));

        diff0_x.normalize();
        diff0_y.normalize();
        diff1.normalize();
        
        const double angle_x = acos(diff0_x.dot(diff1));
        const double angle_y = acos(diff0_y.dot(diff1));
        if (max(angle_x, angle_y) < kMinimumValidAngle)
          continue;
        
        if (best_panorama == -1 || distance < best_distance) {
          best_panorama = p;
          best_distance = distance;
        }
      }

      panorama_id_for_texture_mapping->at(y * floor_patch.texture_size[0] + x) = best_panorama;
    }
  }
}

void SetIUVInFloor(const Patch& floor_patch,
                   const int texture_image_size,
                   const std::pair<int, Eigen::Vector2i>& iuv,
                   Floorplan* floorplan) {
  const Vector2d& min_xy_local = floor_patch.min_xy_local;
  const Vector2d& max_xy_local = floor_patch.max_xy_local;
  
  Vector2d top_left_uv(iuv.second[0], iuv.second[1]);
  Vector2d bottom_right_uv(iuv.second[0] + floor_patch.texture_size[0],
                           iuv.second[1] + floor_patch.texture_size[1]);

  // Convert to [0, 1].
  top_left_uv /= texture_image_size;
  bottom_right_uv /= texture_image_size;
  Vector2d uv_diff = bottom_right_uv - top_left_uv;

  //----------------------------------------------------------------------
  // Set triangulation for room floors.
  for (int room = 0; room < floorplan->GetNumRooms(); ++room) {
    FloorCeilingTriangulation& triangulation = floorplan->GetFloorTriangulation(room);
    for (auto& triangle : triangulation.triangles) {
      triangle.image_index = iuv.first;
      for (int i = 0; i < 3; ++i) {
        const Vector2d local = floorplan->GetRoomVertexLocal(room, triangle.indices[i]);
        Vector2d uv_in_floor((local[0] - min_xy_local[0]) / (max_xy_local[0] - min_xy_local[0]),
                             (local[1] - min_xy_local[1]) / (max_xy_local[1] - min_xy_local[1]));
        
        triangle.uvs[i] = top_left_uv + Vector2d(uv_in_floor[0] * uv_diff[0],
                                                 uv_in_floor[1] * uv_diff[1]);
        for (int j = 0; j < 2; ++j)
          triangle.uvs[i][j] = min(1.0, triangle.uvs[i][j]);
      }
    }
  }

  //----------------------------------------------------------------------
  // Set triangulation for door floors.
}

void SetBoundingBox(const Floorplan& floorplan,
                    Eigen::Vector2d* min_xy_local,
                    Eigen::Vector2d* max_xy_local) {
  // Collect min-max x-y in local.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
      const Vector2d local = floorplan.GetRoomVertexLocal(room, vertex);
      if (room == 0 && vertex == 0) {
        *min_xy_local = *max_xy_local = local;
      } else {
        for (int i = 0; i < 2; ++i) {
          (*min_xy_local)[i] = min((*min_xy_local)[i], local[i]);
          (*max_xy_local)[i] = max((*max_xy_local)[i], local[i]);
        }
      }      
    }
  }
}

void SetRoomSegments(const Floorplan& floorplan,
                     const Patch& floor_patch,
                     const unsigned char background,
                     cv::Mat* room_segments) {
  room_segments->create(floor_patch.texture_size[1], floor_patch.texture_size[0], CV_8UC1);
  
  for (int y = 0; y < floor_patch.texture_size[1]; ++y)
    for (int x = 0; x < floor_patch.texture_size[0]; ++x)
      room_segments->at<unsigned char>(y, x) = background;

  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    vector<cv::Point> points;
    for (int p = 0; p < floorplan.GetNumRoomVertices(room); ++p) {
      const Vector2d local = floorplan.GetRoomVertexLocal(room, p);
      const Vector2i texel_int = ConvertLocalToTexelInt(local, floor_patch);
      points.push_back(cv::Point(texel_int[0], texel_int[1]));
    }
    // reverse(points.begin(), points.end());

    const int length = points.size();
    const int kNumContour = 1;
    const cv::Point* begin = &points[0];
    cv::fillPoly(*room_segments, &begin, &length, kNumContour, room);
  }
}

Eigen::Vector2i ComputeTextureSize(const Eigen::Vector2d& min_xy_local,
                                   const Eigen::Vector2d& max_xy_local,
                                   const int max_texture_size_per_floor_patch) {
  const double floor_width_3d  = max_xy_local[0] - min_xy_local[0];
  const double floor_height_3d = max_xy_local[1] - min_xy_local[1];
  const double max_floor_size  = max(floor_width_3d, floor_height_3d);
  return Vector2i(static_cast<int>(round(floor_width_3d / max_floor_size * max_texture_size_per_floor_patch)),
                  static_cast<int>(round(floor_height_3d / max_floor_size * max_texture_size_per_floor_patch)));
}

void ComputeAverageFloorCeilingHeights(const Floorplan& floorplan,
                                       double* average_floor_height,
                                       double* average_ceiling_height) {
  *average_floor_height = *average_ceiling_height = 0.0;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    *average_floor_height += floorplan.GetFloorHeight(room);
    *average_ceiling_height += floorplan.GetCeilingHeight(room);
  }
  *average_floor_height /= floorplan.GetNumRooms();
  *average_ceiling_height /= floorplan.GetNumRooms();
}

Eigen::Vector2d ConvertLocalToTexel(const Eigen::Vector2d& local,
                                    const Patch& patch) {
  return ConvertLocalToTexel(local, patch.min_xy_local, patch.max_xy_local, patch.texture_size);
}  

Eigen::Vector2i ConvertLocalToTexelInt(const Eigen::Vector2d& local,
                                       const Patch& patch) {
  return ConvertLocalToTexelInt(local, patch.min_xy_local, patch.max_xy_local, patch.texture_size);
}  
  
Eigen::Vector2d ConvertLocalToTexel(const Eigen::Vector2d& local,
                                    const Eigen::Vector2d& min_xy_local,
                                    const Eigen::Vector2d& max_xy_local,
                                    const Eigen::Vector2i& texture_size) {
  const Vector2d diff_3d = max_xy_local - min_xy_local;
  const Vector2d diff = local - min_xy_local;

  return Vector2d(diff[0] / diff_3d[0] * texture_size[0],
                  diff[1] / diff_3d[1] * texture_size[1]);
}  

Eigen::Vector2i ConvertLocalToTexelInt(const Eigen::Vector2d& local,
                                       const Eigen::Vector2d& min_xy_local,
                                       const Eigen::Vector2d& max_xy_local,
                                       const Eigen::Vector2i& texture_size) {
  const Vector2d texel = ConvertLocalToTexel(local, min_xy_local, max_xy_local, texture_size);
  
  return Vector2i(max(0, min(texture_size[0] - 1, static_cast<int>(round((texel[0]))))),
                  max(0, min(texture_size[1] - 1, static_cast<int>(round((texel[1]))))));
}  

int FindClosestPanoramaToRoom(const TextureInput& texture_input,
                              const Patch& floor_patch,
                              const cv::Mat& room_segments,
                              const int room) {
  Vector2d room_center(0, 0);
  int denom = 0;
  for (int y = 0; y < floor_patch.texture_size[1]; ++y) {
    for (int x = 0; x < floor_patch.texture_size[0]; ++x) {
      if (room_segments.at<unsigned char>(y, x) == room) {
        room_center += Vector2d(x, y);
        ++denom;
      }
    }
  }
  if (denom == 0) {
    cerr << "No room pixels..." << room << endl;
    exit (1);
  }
  room_center /= denom;
      
  // Find the closest one to the room.
  const int kInvalid = -1;
  const int kLevel = 0;
  int best_panorama = kInvalid;
  double best_distance = -1.0;
  for (int p = 0; p < texture_input.panoramas.size(); ++p) {
    const Vector3d local_center =
      texture_input.floorplan.GetFloorplanToGlobal().transpose() *
      texture_input.panoramas[p][kLevel].GetCenter();
    const Vector2d texel =
      ConvertLocalToTexel(Vector2d(local_center[0], local_center[1]),
                          floor_patch.min_xy_local, floor_patch.max_xy_local, floor_patch.texture_size);
    const double distance = (texel - room_center).norm();
    if (best_panorama == kInvalid || distance < best_distance) {
      best_panorama = p;
      best_distance = distance;
    }
  }
  if (best_panorama == kInvalid) {
    cerr << "Impossible in finding the closest panorama." << endl;
    exit (1);
  }
  return best_panorama;
}

void ComputeProjectedTextures(const TextureInput& texture_input,
                              const Patch& floor_patch,
                              const std::vector<double>& floor_heights,
                              const std::vector<double>& ceiling_heights,
                              std::vector<cv::Mat>* projected_textures) {

  const Eigen::Vector2d& min_xy_local = floor_patch.min_xy_local;
  const Eigen::Vector2d& max_xy_local = floor_patch.max_xy_local;
  const Eigen::Vector2i& texture_size = floor_patch.texture_size;
  const int level = texture_input.pyramid_level_for_floor;

  if (texture_input.panoramas.size() != texture_input.point_clouds.size()) {
    cerr << "Impossible." << endl;
    exit (1);
  }
  
  for (int p = 0; p < (int)texture_input.panoramas.size(); ++p) {
    const Panorama& panorama = texture_input.panoramas[p][level];
    const int depth_width  = panorama.DepthWidth();
    const int depth_height = panorama.DepthHeight();
    vector<bool> floor_mask(depth_width * depth_height, false);
    // For speed up.

    const int kSkip = 1;
    for (int q = 0; q < texture_input.point_clouds[p].GetNumPoints(); q += kSkip) {
      const auto& point = texture_input.point_clouds[p].GetPoint(q);
      const Vector2d depth_pixel = panorama.ProjectToDepth(point.position);
      const int x = static_cast<int>(round(depth_pixel[0]));
      const int y = static_cast<int>(round(depth_pixel[1]));
      
      if (0 <= x && x < depth_width && 0 <= y && y < depth_height) {
        if (IsOnFloor(texture_input.floorplan, floor_patch, floor_heights, ceiling_heights, point,
                       texture_input.position_error_for_floor)) {
          floor_mask[y * depth_width + x] = true;
        }
      }
    }

    {    
      const int kKernelWidth = 5;
      const int kTime = 3;
      for (int t = 0; t < kTime; ++t)
        image_process::Erode(depth_width, depth_height, kKernelWidth, &floor_mask);
    }

    cv::Mat projected_texture(texture_size[1], texture_size[0], CV_8UC3, cv::Scalar(0));
    const Vector2d xy_diff = max_xy_local - min_xy_local;
    for (int y = 0; y < texture_size[1]; ++y) {
      for (int x = 0; x < texture_size[0]; ++x) {
        const Vector2d local(min_xy_local[0] + xy_diff[0] * x / texture_size[0],
                             min_xy_local[1] + xy_diff[1] * y / texture_size[1]);
        const Vector3d floor_point(local[0], local[1], floor_heights[y * texture_size[0] + x]);
        const Vector3d global = texture_input.floorplan.GetFloorplanToGlobal() * floor_point;
        
        // Project to the depth_mask.
        const Vector2d pixel = panorama.Project(global);
        const Vector2d depth_pixel = panorama.RGBToDepth(pixel);
        const int depth_x = min(depth_width - 1, static_cast<int>(round(depth_pixel[0])));
        const int depth_y = min(depth_height - 1, static_cast<int>(round(depth_pixel[1])));
        
        if (floor_mask[depth_y * depth_width + depth_x]) {
          Vector3f rgb = panorama.GetRGB(pixel);
          for (int i = 0; i < 3; ++i)
            projected_texture.at<cv::Vec3b>(y, x)[i] = rgb[i];
        }
      }
    }
    projected_textures->push_back(projected_texture);
    /*
    char buffer[1024];
    sprintf(buffer, "projected_texture_%03d.png", p);
    cv::imwrite(buffer, projected_texture);
    */
  }
}

void GenerateFloorTexture(const int room,
                          const TextureInput& texture_input,
                          const std::vector<cv::Mat>& projected_textures,
                          const cv::Mat& room_segments,
                          const Patch& floor_patch,
                          cv::Mat* floor_texture) {
  const int kMinPatchSize = 6;
  SynthesisData synthesis_data(projected_textures);
  synthesis_data.num_cg_iterations = texture_input.num_cg_iterations;
  synthesis_data.texture_size = floor_patch.texture_size;
  synthesis_data.patch_size   = max(kMinPatchSize, texture_input.patch_size_for_synthesis);
  synthesis_data.margin       = synthesis_data.patch_size / 6;
  synthesis_data.mask.resize(floor_patch.texture_size[0] * floor_patch.texture_size[1], false);
  int index = 0;
  for (int y = 0; y < floor_patch.texture_size[1]; ++y) {
    for (int x = 0; x < floor_patch.texture_size[0]; ++x, ++index) {
      synthesis_data.mask[index] = room_segments.at<unsigned char>(y, x) == room;
    }
  }

  vector<cv::Mat> patches;
  vector<Eigen::Vector2i> patch_positions;
  const int kTimes = 3;
  for (int t = 0; t < kTimes; ++t) {
    CollectCandidatePatches(synthesis_data, &patches, &patch_positions);
    if (!patches.empty()) {
      break;
    }
    
    cerr << "No patches! Cut patch size by half." << endl;
    if (t != kTimes - 1) {
      synthesis_data.patch_size = max(kMinPatchSize, synthesis_data.patch_size / 2);
      synthesis_data.margin     = max(1, synthesis_data.patch_size / 6);
    }
  }
  if (patches.empty()) {
    cerr << "Do not find any texture. Gave up. Paint light gray." << endl;
    const cv::Vec3b kLightGray(200, 200, 200);
    int index = 0;
    for (int y = 0; y < floor_patch.texture_size[1]; ++y) {
      for (int x = 0; x < floor_patch.texture_size[0]; ++x, ++index) {
        if (synthesis_data.mask[index]) {
          floor_texture->at<cv::Vec3b>(y, x) = kLightGray;
        }
      }
    }
    
    return;
  }

  const bool kNoVerticalConstraint = false;
  SynthesizePoisson(synthesis_data, patches, patch_positions, kNoVerticalConstraint, floor_texture);
  
  cv::imshow("result", *floor_texture);
}

void SynthesizePatch(Patch* patch) {
  vector<cv::Mat> projected_textures;
  cv::Mat projected_texture(patch->texture_size[1],
                            patch->texture_size[0],
                            CV_8UC3);
  {
    int index = 0;
    for (int y = 0; y < patch->texture_size[1]; ++y)
      for (int x = 0; x < patch->texture_size[0]; ++x, ++index)
        projected_texture.at<cv::Vec3b>(y, x) = cv::Vec3b(patch->texture[3 * index + 0],
                                                          patch->texture[3 * index + 1],
                                                          patch->texture[3 * index + 2]);
  }
  
  projected_textures.push_back(projected_texture);
  SynthesisData synthesis_data(projected_textures);

  synthesis_data.num_cg_iterations = 50;
  synthesis_data.texture_size = patch->texture_size;
  synthesis_data.patch_size =
    min(80, min(patch->texture_size[0], patch->texture_size[1]));
  synthesis_data.margin = max(1, synthesis_data.patch_size / 4);
  synthesis_data.mask.resize(patch->texture_size[0] * patch->texture_size[1], true);

  vector<cv::Mat> patches;
  vector<Eigen::Vector2i> patch_positions;
  CollectCandidatePatches(synthesis_data, &patches, &patch_positions);
  if (patches.empty())
    return;
  
  cv::Mat synthesized_texture(patch->texture_size[1],
                              patch->texture_size[0],
                              CV_8UC3,
                              cv::Scalar(0));
  const bool kVerticalConstraint = true;
  SynthesizePoisson(synthesis_data, patches, patch_positions, kVerticalConstraint,
                    &synthesized_texture);
  cv::imshow("result", synthesized_texture);

  {
    int index = 0;
    for (int y = 0; y < patch->texture_size[1]; ++y) {
      for (int x = 0; x < patch->texture_size[0]; ++x, ++index) {
        patch->texture[3 * index + 0] = synthesized_texture.at<cv::Vec3b>(y, x)[0];
        patch->texture[3 * index + 1] = synthesized_texture.at<cv::Vec3b>(y, x)[1];
        patch->texture[3 * index + 2] = synthesized_texture.at<cv::Vec3b>(y, x)[2];
      }
    }
  }
}

void ShrinkTexture(const int shrink_pixels, Patch* patch) {
  vector<bool> valids(patch->texture_size[0] * patch->texture_size[1], false);
  int index = 0;
  for (int y = 0; y < patch->texture_size[1]; ++y) {
    for (int x = 0; x < patch->texture_size[0]; ++x, ++index) {
      if (patch->texture[3 * index + 0] != 0 ||
          patch->texture[3 * index + 1] != 0 ||
          patch->texture[3 * index + 2] != 0) {
        valids[index] = true;
      }
    }
  }

  for (int t = 0; t < shrink_pixels; ++t) {
    vector<bool> valids_org = valids;
    int index = 0;
    for (int y = 0; y < patch->texture_size[1]; ++y) {
      for (int x = 0; x < patch->texture_size[0]; ++x, ++index) {
        if (!valids_org[index]) {
          if ((x != 0                          && valids_org[index - 1]) ||
              (x != patch->texture_size[0] - 1 && valids_org[index + 1]) ||
              (y != 0                          && valids_org[index - patch->texture_size[0]]) ||
              (y != patch->texture_size[1] - 1 && valids_org[index + patch->texture_size[0]])) {
            valids[index] = true;
          }
        }
      }
    }
  }

  index = 0;
  for (int y = 0; y < patch->texture_size[1]; ++y) {
    for (int x = 0; x < patch->texture_size[0]; ++x, ++index) {
      if (!valids[index]) {
        patch->texture[3 * index + 0] = 0;
        patch->texture[3 * index + 1] = 0;
        patch->texture[3 * index + 2] = 0;
      }
    }
  }
}
  
}  // namespace
}  // namespace structured_indoor_modeling
