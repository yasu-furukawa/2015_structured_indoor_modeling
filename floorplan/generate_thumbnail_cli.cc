#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "floorplan.h"
#include "../calibration/file_io.h"
#include "../viewer/configuration.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace Eigen;
using namespace std;

struct Input {
  string data_directory;
  
  // Output (thumbnail specification).
  int thumbnail_width;
  int thumbnail_height;
  double thumbnail_horizontal_angle;

  // Input (panorama data).
  int panorama_width;
  int panorama_height;
  vector<cv::Mat> panoramas;

  vector<PanoramaConfiguration> panorama_configurations;

  // Input (floorplan).
  Floorplan floorplan;
};

Eigen::Vector2d Project(const Eigen::Vector3d& xyz,
                        const PanoramaConfiguration& panorama_configuration,
                        const int width,
                        const int height) {
  Vector3d projected_coordinate =
    panorama_configuration.local_to_global.transpose() * (xyz - panorama_configuration.center);
  // x coordinate.
  double theta = -atan2(projected_coordinate.y(), projected_coordinate.x());
  if (theta < 0.0)
    theta += 2 * M_PI;
  double theta_ratio = max(0.0, min(1.0, theta / (2 * M_PI)));
  if (theta_ratio == 1.0)
    theta_ratio = 0.0;

  Vector2d uv;
  uv[0] = theta_ratio * width;
  const double depth = sqrt(projected_coordinate.x() * projected_coordinate.x() +
                           projected_coordinate.y() * projected_coordinate.y());
  double phi = atan2(projected_coordinate.z(), depth);
  const double phi_per_pixel = panorama_configuration.phi_range / height;
  const double pixel_offset_from_center = phi / phi_per_pixel;
  uv[1] = height / 2.0 - pixel_offset_from_center;

  return uv;
}

void Init(const string& data_directory, Input *input) {
  input->data_directory = data_directory;

  const file_io::FileIO file_io(data_directory);
  for (int p = 0; ; ++p) {
    string buffer = file_io.GetPanoramaImage(p);
    cv::Mat panorama_raw = cv::imread(buffer, 1);
    if (panorama_raw.empty())
      break;

    cv::Mat panorama;
    cv::resize(panorama_raw, panorama, cv::Size(input->panorama_width, input->panorama_height));
    input->panoramas.push_back(panorama);
  }

  input->panorama_configurations.resize((int)input->panoramas.size());
  for (int p = 0; p < (int)input->panoramas.size(); ++p) {
    ReadPanoramaConfiguration(data_directory, p, &input->panorama_configurations[p]);
  }

  {
    ifstream ifstr;
    ifstr.open(file_io.GetFloorplan().c_str());
    ifstr >> input->floorplan;
    ifstr.close();
  }
}

int FindClosestPanorama(const vector<PanoramaConfiguration>& panorama_configurations,
                        const Vector3d& room_center) {
  double best_distance = 0.0;
  int best_panorama = -1;
  for (int p = 0; p < panorama_configurations.size(); ++p) {
    const double distance = (panorama_configurations[p].center - room_center).norm();
    if (distance < best_distance || best_panorama == -1) {
      best_distance = distance;
      best_panorama = p;
    }
  }
  return best_panorama;
}

int FindInsidePanorama(const vector<PanoramaConfiguration>& panorama_configurations,
                       const Vector3d& room_center,
                       const Floorplan& floorplan,
                       const int room) {
  vector<cv::Point> contour;
  for (int w = 0; w < floorplan.GetNumWalls(room); ++w) {
    const Vector2d& point = floorplan.GetRoomVertexLocal(room, w);
    contour.push_back(cv::Point(point[0], point[1]));
  }
  
  int best_panorama = -1;
  double best_distance = 0.0;
  for (int p = 0; p < panorama_configurations.size(); ++p) {
    const Vector3d panorama_center = floorplan.GetFloorplanToGlobal().transpose() * panorama_configurations[p].center;
    const cv::Point2f panorama_center2(panorama_center[0], panorama_center[1]);
    
    if (cv::pointPolygonTest(contour, panorama_center2, true) >= 0.0) {
      const double distance = (panorama_configurations[p].center - room_center).norm();
      if (best_panorama == -1 || distance > best_distance) {
        best_distance = distance;
        best_panorama = p;
      }
    }
  }
  return best_panorama;
}

void Render(const PanoramaConfiguration& panorama_configuration,
            const cv::Mat& panorama,
            const int panorama_width,
            const int panorama_height,
            const Vector3d& look_at,
            const double horizontal_angle,
            const int width,
            const int height,
            cv::Mat* thumbnail) {
  const int kOffset = height * 0.2;
  *thumbnail = cv::Mat(height, width, CV_8UC3);
  // Render.
  Vector3d optical_center = panorama_configuration.center;
  Vector3d optical_axis = look_at - optical_center;
  optical_axis[2] = 0.0;
  optical_axis.normalize();
  Vector3d y_axis(0, 0, -1);
  Vector3d x_axis = -optical_axis.cross(y_axis);
  
  const double x_diameter = 2.0 * tan(horizontal_angle / 2.0);
  const double pixel_size = x_diameter / width;
  x_axis *= pixel_size;
  y_axis *= pixel_size;
  
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Vector3d coordinate =
        optical_center + optical_axis + (x - width / 2) * x_axis + (y - height / 2 + kOffset) * y_axis;
      const Vector2d pixel = Project(coordinate,
                                     panorama_configuration,
                                     panorama_width,
                                     panorama_height);
      
      const int u0 = static_cast<int>(floor(pixel[0]));
      const int v0 = static_cast<int>(floor(pixel[1]));
      const int u1 = u0 + 1;
      const int v1 = v0 + 1;
      
      const double weight00 = (u1 - pixel[0]) * (v1 - pixel[1]);
      const double weight01 = (pixel[0] - u0) * (v1 - pixel[1]);
      const double weight10 = (u1 - pixel[0]) * (pixel[1] - v0);
      const double weight11 = (pixel[0] - u0) * (pixel[1] - v0);
      const int u1_corrected = (u1 % panorama_width);
      
      const cv::Vec3b& color00 = panorama.at<cv::Vec3b>(v0, u0);
      const cv::Vec3b& color01 = panorama.at<cv::Vec3b>(v0, u1_corrected);
      const cv::Vec3b& color10 = panorama.at<cv::Vec3b>(v1, u0);
      const cv::Vec3b& color11 = panorama.at<cv::Vec3b>(v1, u1_corrected);
      
      cv::Vec3b interpolated_color(static_cast<unsigned char>(weight00 * color00[0] +
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
        
      thumbnail->at<cv::Vec3b>(y, x) = interpolated_color;
    }
  }
}

bool IsInside(const Floorplan& floorplan, const int room, const Vector2d& point) {
  const cv::Point2f point_tmp(point[0], point[1]);

  vector<cv::Point> contour;
  for (int w = 0; w < floorplan.GetNumWalls(room); ++w) {
    const Eigen::Vector2d& point = floorplan.GetRoomVertexLocal(room, w);
    contour.push_back(cv::Point(point[0], point[1]));
  }

  if (cv::pointPolygonTest(contour, point_tmp, true) >= 0.0)
    return true;
  else
    return false;
}

//----------------------------------------------------------------------
// Using the panorama closest to the center. The thumbnail points to the center of the room.
void FindFarPanoramaInRoom(const Input& input) {
  const int panorama_num = input.panorama_configurations.size();
  for (int p = 0; p < panorama_num; ++p) {
      const PanoramaConfiguration& panorama_configuration = input.panorama_configurations[p];
      const cv::Mat& panorama = input.panoramas[p];
      
      Vector3d optical_center = panorama_configuration.center;
      Vector3d optical_axis(1, 0, 0);
      Matrix3d rotation;
      const int kRotationNum = 10;
      const double angle = 2 * M_PI / kRotationNum;
      rotation <<
        cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
        0, 0, 1;
      
      for (int r = 0; r < kRotationNum; ++r) {
        Vector3d look_at = optical_center + optical_axis;
        optical_axis = rotation * optical_axis;
        cv::Mat thumbnail;
        Render(panorama_configuration, panorama, input.panorama_width, input.panorama_height,
               look_at, input.thumbnail_horizontal_angle, input.thumbnail_width, input.thumbnail_height,
               &thumbnail);
        
        // const file_io::FileIO file_io(argv[1]);
        char buffer[1024];
        sprintf(buffer, "%s/panorama/thumbnail_%03d_%02d.png", input.data_directory.c_str(), p, r);
        cv::imwrite(buffer, thumbnail);
      }
  }
}

void FindPanoramaClosestToTheRoomCenter(const Input& input) {
  // For each room, identify the best panorama and the angle.
  for (int room = 0; room < input.floorplan.GetNumRooms(); ++room) {
    Vector2d center_before_rotation(0, 0);
    for (int w = 0; w < input.floorplan.GetNumWalls(room); ++w) {
      center_before_rotation += input.floorplan.GetRoomVertexLocal(room, w);
    }
    center_before_rotation /= input.floorplan.GetNumWalls(room);
    const Vector3d room_center =
      input.floorplan.GetFloorplanToGlobal() * Vector3d(center_before_rotation[0],
                                                        center_before_rotation[1],
                                                        (input.floorplan.GetFloorHeight(room) +
                                                         (input.floorplan.GetCeilingHeight(room)) / 2.0));

    // Find the best panorama. Inside the room, but most outside.
    const int kFindPanoramaMethod = 1;
    int best_panorama = -1;
    // Find the one closest to the center.
    if (kFindPanoramaMethod == 0) {
      best_panorama = FindClosestPanorama(input.panorama_configurations, room_center);
    } else if (kFindPanoramaMethod == 1) {
      best_panorama =
        FindInsidePanorama(input.panorama_configurations,
                           room_center,
                           input.floorplan,
                           room);
      if (best_panorama == -1) {
        cerr << "Cannot find a panorama inside a room." << endl;
        best_panorama = FindClosestPanorama(input.panorama_configurations, room_center);
      }
    }
    
    cv::Mat thumbnail;
    Render(input.panorama_configurations[best_panorama],
           input.panoramas[best_panorama],
           input.panorama_width,
           input.panorama_height,
           room_center,
           input.thumbnail_horizontal_angle,
           input.thumbnail_width,
           input.thumbnail_height,
           &thumbnail);

    const file_io::FileIO file_io(input.data_directory);
    cv::imwrite(file_io.GetRoomThumbnail(room), thumbnail);
  }
}

void FindThumbnailPerRoomFromEachPanorama(const Input& input) {
  for (int room = 0; room < input.floorplan.GetNumRooms(); ++room) {
    double length_unit = 0.0;
    const int num_walls = input.floorplan.GetNumWalls(room);
    for (int w = 0; w < num_walls; ++w) {
      const int next_w = (w + 1) % num_walls;
      length_unit += (input.floorplan.GetRoomVertexLocal(room, w) -
                      input.floorplan.GetRoomVertexLocal(room, next_w)).norm();
    }
    length_unit /= 100;
    
    for (int p = 0; p < (int)input.panoramas.size(); ++p) {
      const Vector3d panorama_center =
        input.floorplan.GetFloorplanToGlobal().transpose() * input.panorama_configurations[p].center;
      const Vector2d panorama_center2(panorama_center[0], panorama_center[1]);
      if (!IsInside(input.floorplan, room, panorama_center2))
        continue;

      // Compute the area of a room that is visible from a panorama.
      const int kNumAngleSamples = 360;
      vector<int> visible(kNumAngleSamples, 0);
      for (int a = 0; a < kNumAngleSamples; ++a) {
        Vector2d ray(cos(2 * M_PI * a / kNumAngleSamples),
                     sin(2 * M_PI * a / kNumAngleSamples));
        for (int radius = 1; ;++radius) {
          const Vector2d point = panorama_center2 + radius * length_unit * ray;
          if (IsInside(input.floorplan, room, point))
            ++visible[a];
          else
            break;
        }
      }

      // Find the best angle with the most visible region in the given horizontal angle.
      const int range_radius =
        static_cast<int>(round(input.thumbnail_horizontal_angle / (2 * M_PI / kNumAngleSamples))) / 2;

      int best_angle_index = -1;
      int best_area = 0;
      for (int a = 0; a < kNumAngleSamples; ++a) {
        int area = 0;
        for (int r = -range_radius; r <= range_radius; ++r) {
          const int atmp = (a + r + kNumAngleSamples) % kNumAngleSamples;
          area += visible[atmp];
        }
        if (best_angle_index == -1 || area > best_area) {
          best_angle_index = a;
          best_area = area;
        }
      }

      Vector3d ray(cos(2 * M_PI * best_angle_index / kNumAngleSamples),
                   sin(2 * M_PI * best_angle_index / kNumAngleSamples),
                   0.0);
      ray = input.floorplan.GetFloorplanToGlobal() * ray;
      ray[2] = 0.0;
      const Vector3d look_at = input.panorama_configurations[p].center + ray;

      cv::Mat thumbnail;
      Render(input.panorama_configurations[p], input.panoramas[p],
             input.panorama_width, input.panorama_height,
             look_at, input.thumbnail_horizontal_angle,
             input.thumbnail_width, input.thumbnail_height,
             &thumbnail);

      char buffer[1024];
      sprintf(buffer, "%s/panorama/thumbnail_%03d_%02d_%08d.png",
              input.data_directory.c_str(), room, p, best_area);
      cv::imwrite(buffer, thumbnail);
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    exit (1);
  }

  Input input;
  input.thumbnail_width            = 400;
  input.thumbnail_height           = 300;
  input.thumbnail_horizontal_angle = 100.0 * M_PI / 180.0;
  // Shrink panorama to this size before sampling.
  input.panorama_width = 1024;
  input.panorama_height = input.panorama_width / 2;

  Init(argv[1], &input);

  if (0) {
    FindPanoramaClosestToTheRoomCenter(input);
  };
    
  if (1) {
    FindFarPanoramaInRoom(input);
  }

  if (0) {
    FindThumbnailPerRoomFromEachPanorama(input);
  }
  
  return 0;
}
