#include <fstream>
#include <iostream>

#include "generate_texture.h"
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
                   const Vector2d& min_xy_local,
                   const Vector2d& max_xy_local,
                   const int texture_image_size,
                   const std::pair<int, Eigen::Vector2i>& iuv,
                   Floorplan* floorplan);

void SetBoundingBox(const Floorplan& floorplan,
                    Eigen::Vector2d* min_xy_local,
                    Eigen::Vector2d* max_xy_local);

void SetRoomSegments(const Floorplan& floorplan,
                     const Eigen::Vector2i& texture_size,
                     const unsigned char background,
                     const Eigen::Vector2d& min_xy_local,
                     const Eigen::Vector2d& max_xy_local,
                     cv::Mat* room_segments);

void ComputeAverageFloorCeilingHeights(const Floorplan& floorplan,
                                       double* average_floor_height,
                                       double* average_ceiling_height);
  
Eigen::Vector2i ComputeTextureSize(const Eigen::Vector2d& min_xy_local,
                                   const Eigen::Vector2d& max_xy_local,
                                   const int max_texture_size_per_floor_patch);
  
Eigen::Vector2d ConvertLocalToTexel(const Eigen::Vector2d& local,
                                    const Eigen::Vector2d& min_xy_local,
                                    const Eigen::Vector2d& max_xy_local,
                                    const Eigen::Vector2i& texture_size);

Eigen::Vector2i ConvertLocalToTexelInt(const Eigen::Vector2d& local,
                                       const Eigen::Vector2d& min_xy_local,
                                       const Eigen::Vector2d& max_xy_local,
                                       const Eigen::Vector2i& texture_size);

int FindClosestPanoramaToRoom(const Floorplan& floorplan,
                              const std::vector<std::vector<Panorama> >& panoramas,
                              const Eigen::Vector2d& min_xy_local,
                              const Eigen::Vector2d& max_xy_local,
                              const Eigen::Vector2i& texture_size,
                              const cv::Mat& room_segments,
                              const int room);

void GenerateFloorTexture(const Floorplan& floorplan,
                          const std::vector<std::vector<Panorama> >& panoramas,
                          const std::vector<PointCloud>& point_clouds,
                          const std::set<int>& panorama_ids,
                          const Eigen::Vector2d& min_xy_local,
                          const Eigen::Vector2d& max_xy_local,
                          const Eigen::Vector2i& texture_size,
                          const cv::Mat& room_segments,
                          const int room,
                          const double average_floor_height,
                          const double average_ceiling_height,
                          cv::Mat* floor_texture);
  
}  // namespace

int GetEndPanorama(const FileIO& file_io, const int start_panorama) {
  int panorama = start_panorama;
  while (1) {
    const string filename = file_io.GetPanoramaImage(panorama);
    ifstream ifstr;
    ifstr.open(filename.c_str());
    if (!ifstr.is_open()) {
      ifstr.close();
      return panorama;
    }
    ifstr.close();
    ++panorama;
  }
}
  
void ReadPanoramas(const FileIO& file_io,
                   const int start_panorama,
                   const int end_panorama,
                   const int num_pyramid_levels,
                   vector<vector<Panorama> >* panoramas) {
  panoramas->resize(end_panorama - start_panorama);
  cerr << "Reading panoramas" << flush;
  for (int p = start_panorama; p < end_panorama; ++p) {
    cerr << '.' << flush;
    const int p_index = p - start_panorama;
    panoramas->at(p_index).resize(num_pyramid_levels);
    for (int level = 0; level < num_pyramid_levels; ++level) {
      panoramas->at(p_index)[level].Init(file_io, p);
      if (level != 0) {
        const int new_width  = panoramas->at(p_index)[level].Width()  / (0x01 << level);
        const int new_height = panoramas->at(p_index)[level].Height() / (0x01 << level);
        panoramas->at(p_index)[level].ResizeRGB(Vector2i(new_width, new_height));
      }
    }
  }
  cerr << " done." << endl;
}

void ReadPanoramaToGlobals(const FileIO& file_io,
                           const int start_panorama,
                           const int end_panorama,
                           vector<Matrix4d>* panorama_to_globals) {
  panorama_to_globals->resize(end_panorama - start_panorama);
  for (int p = start_panorama; p < end_panorama; ++p) {
    const int p_index = p - start_panorama;
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
        ifstr >> (*panorama_to_globals)[p_index](y, x);
      }
    }
    ifstr.close();
  }
}

void ReadPointClouds(const FileIO& file_io,
                     const int start_panorama,
                     const int end_panorama,
                     std::vector<PointCloud>* point_clouds) {
  point_clouds->resize(end_panorama - start_panorama);
  for (int p = start_panorama; p < end_panorama; ++p) {
    const int p_index = p - start_panorama;
    point_clouds->at(p_index).Init(file_io, p);
    point_clouds->at(p_index).ToGlobal(file_io, p);
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

bool IsOnFloor(const Floorplan& floorplan,
               const double average_floor_height,
               const double average_ceiling_height,
               const Point& point) {
  const Vector3d local_position = floorplan.GetFloorplanToGlobal().transpose() * point.position;
  Vector3d local_normal = floorplan.GetFloorplanToGlobal().transpose() * point.normal;

  //???? critical.
  const double kPositionError = 0.08;
  const double kNormalError = cos(30 * M_PI / 180.0);

  const double margin = (average_ceiling_height - average_floor_height) * kPositionError;
  
  if (fabs(local_position[2] - average_floor_height) > margin)
    return false;

  /*
  if (local_normal.normalized().dot(Vector3d(0, 0, 1)) < kNormalError)
    return false;
  */
  
  return true;
}
  
void SetFloorPatch(const Floorplan& floorplan,
                   const std::vector<std::vector<Panorama> >& panoramas,
                   const std::vector<PointCloud>& point_clouds,
                   const int max_texture_size_per_floor_patch,
                   Patch* floor_patch,
                   Eigen::Vector2d* min_xy_local,
                   Eigen::Vector2d* max_xy_local) {
  SetBoundingBox(floorplan, min_xy_local, max_xy_local);
  
  double average_floor_height, average_ceiling_height;
  ComputeAverageFloorCeilingHeights(floorplan, &average_floor_height, &average_ceiling_height);

  floor_patch->vertices[0] = Vector3d((*min_xy_local)[0], (*min_xy_local)[1], average_floor_height);
  floor_patch->vertices[1] = Vector3d((*max_xy_local)[0], (*min_xy_local)[1], average_floor_height);
  floor_patch->vertices[2] = Vector3d((*max_xy_local)[0], (*max_xy_local)[1], average_floor_height);
  floor_patch->vertices[3] = Vector3d((*min_xy_local)[0], (*max_xy_local)[1], average_floor_height);

  floor_patch->texture_size =
    ComputeTextureSize(*min_xy_local, *max_xy_local, max_texture_size_per_floor_patch);
  
  // Compute a room segmentation.
  const unsigned char kBackground = 255;
  cv::Mat room_segments;
  SetRoomSegments(floorplan, floor_patch->texture_size, kBackground, *min_xy_local, *max_xy_local,
                  &room_segments);
  
  // For each room, put texture.
  const int kLevel = 0;
  cv::Mat floor_texture(floor_patch->texture_size[1], floor_patch->texture_size[0],
                        CV_8UC3, cv::Scalar(0));
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    // Collect panoramas for room.
    set<int> panorama_ids;
    for (int p = 0; p < panoramas.size(); ++p) {
      const Vector3d local_center =
        floorplan.GetFloorplanToGlobal().transpose() * panoramas[p][kLevel].GetCenter();
      const Vector2i texel_int =
        ConvertLocalToTexelInt(Vector2d(local_center[0], local_center[1]),
                               *min_xy_local, *max_xy_local, floor_patch->texture_size);
      if (room_segments.at<unsigned char>(texel_int[1], texel_int[0]) == room)
        panorama_ids.insert(p);
    }

    if (panorama_ids.empty()) {
      panorama_ids.insert(FindClosestPanoramaToRoom(floorplan, panoramas,
                                                    *min_xy_local, *max_xy_local,
                                                    floor_patch->texture_size,
                                                    room_segments, room));
    }

    GenerateFloorTexture(floorplan, panoramas, point_clouds, panorama_ids,
                         *min_xy_local, *max_xy_local, floor_patch->texture_size, room_segments,
                         room, average_floor_height, average_ceiling_height, &floor_texture);

  }
}

void SetFloorPatchGlobal(const Floorplan& floorplan,
                         const std::vector<std::vector<Panorama> >& panoramas,
                         const std::vector<PointCloud>& point_clouds,
                         const int max_texture_size_per_floor_patch,
                         Patch* floor_patch,
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
  const int kFirstRoom = 0;
  floor_patch->vertices[0] =
    Vector3d((*min_xy_local)[0], (*min_xy_local)[1], floorplan.GetFloorHeight(kFirstRoom));
  floor_patch->vertices[1] =
    Vector3d((*max_xy_local)[0], (*min_xy_local)[1], floorplan.GetFloorHeight(kFirstRoom));
  floor_patch->vertices[2] =
    Vector3d((*max_xy_local)[0], (*max_xy_local)[1], floorplan.GetFloorHeight(kFirstRoom));
  floor_patch->vertices[3] =
    Vector3d((*min_xy_local)[0], (*max_xy_local)[1], floorplan.GetFloorHeight(kFirstRoom));

  // Map (min_xy_local, max_xy_local) to the image.
  const double floor_width_3d  = (*max_xy_local)[0] - (*min_xy_local)[0];
  const double floor_height_3d = (*max_xy_local)[1] - (*min_xy_local)[1];
  const double max_floor_size  = max(floor_width_3d, floor_height_3d);
  floor_patch->texture_size[0] =
    static_cast<int>(round(floor_width_3d / max_floor_size * max_texture_size_per_floor_patch));
  floor_patch->texture_size[1] =
    static_cast<int>(round(floor_height_3d / max_floor_size * max_texture_size_per_floor_patch));
  const int texture_width  = floor_patch->texture_size[0];
  const int texture_height = floor_patch->texture_size[1];

  double average_floor_height = 0.0;
  double average_ceiling_height = 0.0;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    average_floor_height += floorplan.GetFloorHeight(room);
    average_ceiling_height += floorplan.GetCeilingHeight(room);
  }
  average_floor_height /= floorplan.GetNumRooms();
  average_ceiling_height /= floorplan.GetNumRooms();
  
  const int kLevel = 0;
  const int kNumChannels = 3;
  vector<float> average_floor_texture(texture_width * texture_height * kNumChannels, 0.0);
  vector<float> weights(texture_width * texture_height, 0);
  // vector<int> denoms(texture_width * texture_height, 0);
  for (int i = 0; i < panoramas.size(); ++i) {
    const Panorama& panorama = panoramas[i][kLevel];
    cout << i << ' ' << flush;
    const int width  = panorama.Width();
    const int height = panorama.Height();
    const int depth_width  = panorama.DepthWidth();
    const int depth_height = panorama.DepthHeight();
    vector<bool> floor_mask(depth_width * depth_height, false);

    for (int p = 0; p < point_clouds[i].GetNumPoints(); ++p) {
      const auto& point = point_clouds[i].GetPoint(p);
      if (!IsOnFloor(floorplan, average_floor_height, average_ceiling_height, point)) {
        continue;
      }
      const Vector2d depth_pixel = panorama.ProjectToDepth(point.position);
      const int x = static_cast<int>(round(depth_pixel[0]));
      const int y = static_cast<int>(round(depth_pixel[1]));

      if (0 <= x && x < depth_width && 0 <= y && y < depth_height) {
        floor_mask[y * depth_width + x] = true;
      }
    }

    {    
      const int kKernelWidth = 5;
      const int kTime = 3;
      for (int t = 0; t < kTime; ++t)
        image_process::Erode(depth_width, depth_height, kKernelWidth, &floor_mask);
    }

    // cv::Mat texture_from_panorama(texture_height, texture_width, CV_8UC3);
    const Vector2d xy_diff = *max_xy_local - *min_xy_local;
    for (int y = 0; y < texture_height; ++y) {
      for (int x = 0; x < texture_width; ++x) {
        const Vector2d local((*min_xy_local)[0] + xy_diff[0] * x / texture_width,
                             (*min_xy_local)[1] + xy_diff[1] * y / texture_height);
        const Vector3d floor_point(local[0], local[1], average_floor_height);
        const Vector3d global = floorplan.GetFloorplanToGlobal() * floor_point;
        
        // Project to the depth_mask.
        const Vector2d pixel = panorama.Project(global);
        const Vector2d depth_pixel = panorama.RGBToDepth(pixel);
        const int depth_x = min(depth_width - 1, static_cast<int>(round(depth_pixel[0])));
        const int depth_y = min(depth_height - 1, static_cast<int>(round(depth_pixel[1])));
        
        if (floor_mask[depth_y * depth_width + depth_x]) {
          Vector3f rgb = panorama.GetRGB(pixel);

          const double weight = 1 / (panorama.GetCenter() - global).norm();
          if (weights[y * texture_width + x] < weight) {
            weights[y * texture_width + x] = weight;
            for (int j = 0; j < 3; ++j)
              average_floor_texture[kNumChannels * (y * texture_width + x) + j] = rgb[j];
          }
            /*
            for (int i = 0; i < 3; ++i)
            average_floor_texture[kNumChannels * (y * texture_width + x) + i] += rgb[i];
          denoms[y * texture_width + x] += 1;
            */
        }
        /*
          for (int i = 0; i < 3; ++i)
            texture_from_panorama.at<cv::Vec3b>(y, x)[i] = static_cast<unsigned char>(rgb[i]);
        } else {
          for (int i = 0; i < 3; ++i)
            texture_from_panorama.at<cv::Vec3b>(y, x)[i] = 0;
        }
          */
      }
    }
  }

  cv::Mat average(texture_height, texture_width, CV_8UC3);
  int index = 0;
  for (int y = 0; y < texture_height; ++y) {
    for (int x = 0; x < texture_width; ++x, ++index) {
      if (weights[index] == 0) {
	for (int i = 0; i < 3; ++i)
	  average.at<cv::Vec3b>(y, x)[i] = 0;
      } else {
	for (int i = 0; i < 3; ++i)
	  average.at<cv::Vec3b>(y, x)[i] = 
	    static_cast<unsigned char>(round(average_floor_texture[3 * index + i]));
      }
      /*
      if (denoms[index] == 0) {
	for (int i = 0; i < 3; ++i)
	  average.at<cv::Vec3b>(y, x)[i] = 0;
      } else {
	for (int i = 0; i < 3; ++i)
	  average.at<cv::Vec3b>(y, x)[i] = 
	    static_cast<unsigned char>(round(average_floor_texture[3 * index + i] / denoms[index]));
      }
      */
    }
  }
  
  cv::imshow("texture", average);
  
  char buffer[1024];
  sprintf(buffer, "average_floor.png");
  cv::imwrite(buffer, average);

    /*
    {
      // Compute a depthimage that corresponds to the floor.
      cv::Mat texture(height, width, CV_8UC3);
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const Vector2d pixel(x, y);
          Vector3f color = panorama.GetRGB(pixel);
          const Vector2d depth_pixel = panorama.RGBToDepth(pixel);
          const int mask_index =
            static_cast<int>(round(depth_pixel[1])) * depth_width +
            static_cast<int>(round(depth_pixel[0]));
          
          if (!floor_mask[mask_index]) {
            color /= 3.0;
            color[2] = 0.0;
          }
          
          texture.at<cv::Vec3b>(y, x) = cv::Vec3b(static_cast<unsigned char>(color[0]),
                                                  static_cast<unsigned char>(color[1]),
                                                  static_cast<unsigned char>(color[2]));
        }
      }
      cv::imshow("before", texture);
    }

    {
      // Compute a depthimage that corresponds to the floor.
      cv::Mat texture(height, width, CV_8UC3);
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const Vector2d pixel(x, y);
          Vector3f color = panorama.GetRGB(pixel);
          const Vector2d depth_pixel = panorama.RGBToDepth(pixel);
          const int mask_index =
            static_cast<int>(round(depth_pixel[1])) * depth_width +
            static_cast<int>(round(depth_pixel[0]));
          
          if (!floor_mask[mask_index]) {
            color /= 3.0;
            color[2] = 0.0;
          }
          
          texture.at<cv::Vec3b>(y, x) = cv::Vec3b(static_cast<unsigned char>(color[0]),
                                                  static_cast<unsigned char>(color[1]),
                                                  static_cast<unsigned char>(color[2]));
        }
      }
      cv::imshow("after", texture);
      //char buffer[1024];
      //sprintf(buffer, "floor_%02d.png", i);
      //cv::imwrite(buffer, texture);
    }
    cv::waitKey(0);
    */
  cout << "done" << endl;
}
  
void SetFloorPatchOld(const Floorplan& floorplan,
                      const std::vector<std::vector<Panorama> >& panoramas,
                      const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                      const int max_texture_size_per_floor_patch,
                      Patch* floor_patch,
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
  const int kFirstRoom = 0;
  floor_patch->vertices[0] =
    Vector3d((*min_xy_local)[0], (*min_xy_local)[1], floorplan.GetFloorHeight(kFirstRoom));
  floor_patch->vertices[1] =
    Vector3d((*max_xy_local)[0], (*min_xy_local)[1], floorplan.GetFloorHeight(kFirstRoom));
  floor_patch->vertices[2] =
    Vector3d((*max_xy_local)[0], (*max_xy_local)[1], floorplan.GetFloorHeight(kFirstRoom));
  floor_patch->vertices[3] =
    Vector3d((*min_xy_local)[0], (*max_xy_local)[1], floorplan.GetFloorHeight(kFirstRoom));

  const int kInitial = -2;
  const int kWall = -1;
  // Map (min_xy_local, max_xy_local) to the image.
  const double floor_width_3d  = (*max_xy_local)[0] - (*min_xy_local)[0];
  const double floor_height_3d = (*max_xy_local)[1] - (*min_xy_local)[1];
  const double max_floor_size  = max(floor_width_3d, floor_height_3d);
  floor_patch->texture_size[0] =
    static_cast<int>(round(floor_width_3d / max_floor_size * max_texture_size_per_floor_patch));
  floor_patch->texture_size[1] =
    static_cast<int>(round(floor_height_3d / max_floor_size * max_texture_size_per_floor_patch));

  vector<int> panorama_id_for_texture_mapping(floor_patch->texture_size[0] * floor_patch->texture_size[1], kInitial);
  
  // Mark walls to kWall.
  // MarkWalls(floorplan, floor_patch, min_xy_local, max_xy_local, kWall,
  // max_texture_size_per_floor_patch, &panorama_id_for_texture_mapping);

  // Set panorama ID.
  SetManhattanDelaunay(floorplan, panoramas, *floor_patch, *min_xy_local, *max_xy_local,
                       &panorama_id_for_texture_mapping);

  const int kLevelZero = 0;
  const double floor_height = floorplan.GetFloorHeight(kFirstRoom);

  // Grab color.
  int index = 0;
  floor_patch->texture.clear();
  const Vector2d xy_diff = *max_xy_local - *min_xy_local;
  for (int y = 0; y < floor_patch->texture_size[1]; ++y) {
    for (int x = 0; x < floor_patch->texture_size[0]; ++x, ++index) {
      const Vector2d local((*min_xy_local)[0] + xy_diff[0] * x / floor_patch->texture_size[0],
                           (*min_xy_local)[1] + xy_diff[1] * y / floor_patch->texture_size[1]);
      const Vector3d floor_point(local[0], local[1], floor_height);
      const Vector3d global = floorplan.GetFloorplanToGlobal() * floor_point;

      const int panorama = panorama_id_for_texture_mapping[index];
      if (panorama < 0) {
        floor_patch->texture.push_back(0);
      } else {
        Vector2d pixel = panoramas[panorama][kLevelZero].Project(global);
        Vector3f rgb = panoramas[panorama][kLevelZero].GetRGB(pixel);
        for (int i = 0; i < 3; ++i)
          floor_patch->texture.push_back(static_cast<unsigned char>(round(rgb[i])));
      }
    }
  }
  
  /*
  {
    vector<Vector3i> colors(panoramas.size());
    for (int p = 0; p < panoramas.size(); ++p) {
      for (int i = 0; i < 3; ++i)
        colors[p][i] = rand() % 255;
    }

    cv::Mat image(max_texture_size_per_floor_patch,
                  max_texture_size_per_floor_patch,
                  CV_8UC3);
    int index = 0;
    for (int y = 0; y < max_texture_size_per_floor_patch; ++y)
      for (int x = 0; x < max_texture_size_per_floor_patch; ++x, ++index) {
        for (int i = 0; i < 3; ++i)
          image.at<cv::Vec3b>(y, x)[i] = colors[panorama_id_for_texture_mapping[index]][i];
      }
    cv::imshow("test", image);
    cv::waitKey(0);
    }
  */
}

void SetWallPatches(const Floorplan& floorplan,
                    const std::vector<std::vector<Panorama> >& panoramas,
                    const int max_texture_size_per_wall_patch,
                    const int texture_height_per_wall,
                    std::vector<std::vector<Patch> >* wall_patches) {
  wall_patches->clear();
  wall_patches->resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    cerr << room << '/' << floorplan.GetNumRooms() << ' ' << flush;
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
      FindVisiblePanoramas(panoramas, patch, &visible_panoramas_weights);
      
      // Keep the best one only.
      const int best_panorama = visible_panoramas_weights[0].second;
      SetTextureSize(max_texture_size_per_wall_patch, texture_height_per_wall, &patch);
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
                      const Eigen::Vector2d& min_xy_local,
                      const Eigen::Vector2d& max_xy_local,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::vector<std::vector<unsigned char> >* texture_images,
                      std::pair<int, Eigen::Vector2i>* iuv,
                      int* max_texture_height) {
  UpdateIUV(floor_patch.texture_size, texture_image_size, iuv, max_texture_height);
  // Copy texel data.
  CopyTexel(floor_patch, texture_image_size, *iuv, texture_images);
  // Update IUV in floorplan.
  SetIUVInFloor(floor_patch, min_xy_local, max_xy_local, texture_image_size, *iuv, floorplan);
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

  patch->texture_size[1] = texture_height_per_wall;
  patch->texture_size[0]  =
    static_cast<int>(round(patch->texture_size[1] * patch_width_3d / patch_height_3d));

  if (patch->texture_size[0] > max_texture_size_per_patch) {
    patch->texture_size[1] = max_texture_size_per_patch * patch->texture_size[1] / patch->texture_size[0];
    patch->texture_size[0] = max_texture_size_per_patch;
  }
  
  /*
  patch->texture_size[0]  =
    static_cast<int>(round((patch->vertices[1] - patch->vertices[0]).norm() / texel_size));
  patch->texture_size[1] =
    static_cast<int>(round((patch->vertices[3] - patch->vertices[0]).norm() / texel_size));

  patch->texel_size = texel_size;
  {
    const int max_dimension = max(patch->texture_size[0], patch->texture_size[1]);
    if (max_dimension > max_texture_size_per_patch) {
      patch->texel_size *= max_dimension / static_cast<double>(max_texture_size_per_patch);
    }
  }

  patch->texture_size[0]  =
    static_cast<int>(round((patch->vertices[1] - patch->vertices[0]).norm() / patch->texel_size));
  patch->texture_size[1] =
    static_cast<int>(round((patch->vertices[3] - patch->vertices[0]).norm() / patch->texel_size));

  patch->texture_size[0]  = max(1, min(max_texture_size_per_patch, patch->texture_size[0]));
  patch->texture_size[1] = max(1, min(max_texture_size_per_patch, patch->texture_size[1]));
  */
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
                   const Vector2d& min_xy_local,
                   const Vector2d& max_xy_local,
                   const int texture_image_size,
                   const std::pair<int, Eigen::Vector2i>& iuv,
                   Floorplan* floorplan) {
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
                     const Eigen::Vector2i& texture_size,
                     const unsigned char background,
                     const Eigen::Vector2d& min_xy_local,
                     const Eigen::Vector2d& max_xy_local,
                     cv::Mat* room_segments) {
  room_segments->create(texture_size[1], texture_size[0], CV_8UC1);
  
  for (int y = 0; y < texture_size[1]; ++y)
    for (int x = 0; x < texture_size[0]; ++x)
      room_segments->at<unsigned char>(y, x) = background;

  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    vector<cv::Point> points;
    for (int p = 0; p < floorplan.GetNumRoomVertices(room); ++p) {
      const Vector2d local = floorplan.GetRoomVertexLocal(room, p);
      const Vector2i texel_int =
        ConvertLocalToTexelInt(local, min_xy_local, max_xy_local, texture_size);
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

int FindClosestPanoramaToRoom(const Floorplan& floorplan,
                              const std::vector<std::vector<Panorama> >& panoramas,
                              const Eigen::Vector2d& min_xy_local,
                              const Eigen::Vector2d& max_xy_local,
                              const Eigen::Vector2i& texture_size,
                              const cv::Mat& room_segments,
                              const int room) {
  Vector2d room_center(0, 0);
  int denom = 0;
  for (int y = 0; y < texture_size[1]; ++y) {
    for (int x = 0; x < texture_size[0]; ++x) {
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
  for (int p = 0; p < panoramas.size(); ++p) {
    const Vector3d local_center =
      floorplan.GetFloorplanToGlobal().transpose() * panoramas[p][kLevel].GetCenter();
    const Vector2d texel =
      ConvertLocalToTexel(Vector2d(local_center[0], local_center[1]),
                          min_xy_local, max_xy_local, texture_size);
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

void GenerateFloorTexture(const Floorplan& floorplan,
                          const std::vector<std::vector<Panorama> >& panoramas,
                          const std::vector<PointCloud>& point_clouds,
                          const std::set<int>& panorama_ids,
                          const Eigen::Vector2d& min_xy_local,
                          const Eigen::Vector2d& max_xy_local,
                          const Eigen::Vector2i& texture_size,
                          const cv::Mat& room_segments,
                          const int room,
                          const double average_floor_height,
                          const double average_ceiling_height,
                          cv::Mat* floor_texture) {
  const int kLevel = 0;

  vector<cv::Mat> projected_textures;
  for (const auto panorama_id : panorama_ids) {
    const Panorama& panorama = panoramas[panorama_id][kLevel];
    const int width  = panorama.Width();
    const int height = panorama.Height();
    const int depth_width  = panorama.DepthWidth();
    const int depth_height = panorama.DepthHeight();
    vector<bool> floor_mask(depth_width * depth_height, false);
    
    for (int p = 0; p < point_clouds[panorama_id].GetNumPoints(); ++p) {
      const auto& point = point_clouds[panorama_id].GetPoint(p);
      if (!IsOnFloor(floorplan, average_floor_height, average_ceiling_height, point)) {
        continue;
      }
      const Vector2d depth_pixel = panorama.ProjectToDepth(point.position);
      const int x = static_cast<int>(round(depth_pixel[0]));
      const int y = static_cast<int>(round(depth_pixel[1]));
      if (0 <= x && x < depth_width && 0 <= y && y < depth_height) {
        floor_mask[y * depth_width + x] = true;
      }
    }
    {    
      const int kKernelWidth = 5;
      const int kTime = 1;
      for (int t = 0; t < kTime; ++t)
        image_process::Erode(depth_width, depth_height, kKernelWidth, &floor_mask);
    }

    cv::Mat projected_texture(texture_size[1], texture_size[0], CV_8UC3, cv::Scalar(0));
    const Vector2d xy_diff = max_xy_local - min_xy_local;
    for (int y = 0; y < texture_size[1]; ++y) {
      for (int x = 0; x < texture_size[0]; ++x) {
        if (room_segments.at<unsigned char>(y, x) != room)
          continue;
        const Vector2d local(min_xy_local[0] + xy_diff[0] * x / texture_size[0],
                             min_xy_local[1] + xy_diff[1] * y / texture_size[1]);
        const Vector3d floor_point(local[0], local[1], average_floor_height);
        const Vector3d global = floorplan.GetFloorplanToGlobal() * floor_point;
        
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
    projected_textures.push_back(projected_texture);
  }

  for (const auto& item : projected_textures) {
    cv::imshow("test", item);
    cv::waitKey(0);
  }
  
}
  
}  // namespace
}  // namespace structured_indoor_modeling
