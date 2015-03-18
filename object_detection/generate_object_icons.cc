#include <fstream>
#include <iostream>
#include "generate_object_icons.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const ObjectId kInitialObject  = make_pair<int, int>(-1, -1);
const ObjectId kNoObject = make_pair<int, int>(-2, -2);
  
void RasterizeObjectIds(const std::vector<Panorama>& panoramas,
                        const std::vector<PointCloud>& object_point_clouds,
                        std::vector<std::vector<ObjectId> >* object_id_maps) {
  const double kThresholdRatio = 0.05;
  
  const int num_panoramas = (int)panoramas.size();
  const int num_rooms = (int)object_point_clouds.size();
  object_id_maps->clear();
  object_id_maps->resize(num_panoramas);

  for (int p = 0; p < num_panoramas; ++p) {
    const Panorama& panorama = panoramas[p];
    const double visibility_threshold = panorama.GetAverageDistance() * kThresholdRatio;

    const int width  = panorama.DepthWidth();
    const int height = panorama.DepthHeight();
    object_id_maps->at(p).resize(width * height, kInitialObject);

    for (int room = 0; room < num_rooms; ++room) {
      const PointCloud& point_cloud = object_point_clouds[room];

      for (int q = 0; q < point_cloud.GetNumPoints(); ++q) {
        const Point& point = point_cloud.GetPoint(q);
        if (point.object_id == -1) {
          cerr << "No object id assigned to a point." << endl;
          exit (1);
        }
        const ObjectId object_id = make_pair<int, int>(room, point.object_id);

        const Vector2d depth_pixel = panorama.ProjectToDepth(point.position);
        const double depth = panorama.GetDepth(depth_pixel);
        const double distance = (point.position - panorama.GetCenter()).norm();

        // Invisible.
        if (distance > depth + visibility_threshold)
          continue;

        const int u = static_cast<int>(round(depth_pixel[0]));
        const int v = static_cast<int>(round(depth_pixel[1]));
        object_id_maps->at(p)[v * width + u] = object_id;
      }
    }


    {
      map<ObjectId, Vector3i> color_table;
      char buffer[1024];
      sprintf(buffer, "%03d.ppm", p);
      ofstream ofstr;
      ofstr.open(buffer);
      ofstr << "P3" << endl
            << width << ' ' << height << endl
            << 255 << endl;
      int index = 0;
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x, ++index) {
          if (object_id_maps->at(p)[index] == kInitialObject) {
            ofstr << "255 255 255 ";
          } else {
            if (color_table.find(object_id_maps->at(p)[index]) == color_table.end()) {
              Vector3i color(rand() % 255, rand() % 255, rand() % 255);
              color_table[object_id_maps->at(p)[index]] = color;
            }
            ofstr << color_table[object_id_maps->at(p)[index]][0] << ' '
                  << color_table[object_id_maps->at(p)[index]][1] << ' '
                  << color_table[object_id_maps->at(p)[index]][2] << ' ';
          }
        }
      }
      ofstr.close();
    }
  }
}

void AssociateObjectId(const std::vector<Panorama>& panoramas,
                       const std::vector<Detection>& detections,
                       const std::vector<std::vector<ObjectId> >& object_id_maps,
                       std::vector<ObjectId>* associated_object_ids) {
  // For each detection, corresponding object id.
  associated_object_ids->clear();
  associated_object_ids->resize(detections.size(), kInitialObject);
  // For each object id, corresponding detection index.
  const int kNoMatch = -1;
  map<ObjectId, int> object_id_to_detection_index;

  //----------------------------------------------------------------------
  // Greedy assignment.
  while (true) {
    // Pick the detection whose value is kInitial and has the highest score.
    int best_detection_index = -1;
    double best_detection_score = -100.0;

    for (int index = 0; index < (int)detections.size(); ++index) {
      const Detection& detection = detections[index];
      if (associated_object_ids->at(index) != kInitial)
        continue;
      if (detection.score > best_detection_score) {
        best_detection_score = detection.score;
        best_detection_index = index;
      }
    }
    if (best_detection_index == -1)
      break;

    // Find the object inside the detection.
    const ObjectId best_object = FindObject(panoramas, object_id_maps, detections[best_detection_index]);
    if (best_object.first == kNoObject) {
      associated_object_ids->at(best_detection_index) = kNoObject;
    } else {
      // If best_object has already an associated detection ignore.
      if (object_id_to_detection_index.find(best_object) != object_id_to_detection_index.end())
        continue;
      else
        object_id_to_detection_index[best_object] = best_detection_index;
    }
  }
}  

}  // namespace structured_indoor_modeling
  
