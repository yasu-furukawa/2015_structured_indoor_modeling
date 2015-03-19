#include <iostream>
#include <fstream>
#include <limits>
#include <vector>

#include <gflags/gflags.h>

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "../base/panorama.h"
#include "../base/point_cloud.h"
#include "detection.h"
#include "generate_object_icons.h"

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

DEFINE_double(score_threshold, 0.0, "Ignore detections below this threshold.");
DEFINE_double(area_threshold, 0.3, "How many pixels in a bounding box must be of the object.");

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
#ifdef __APPLE__
  google::ParseCommandLineFlags(&argc, &argv, true);
#else
  gflags::ParseCommandLineFlags(&argc, &argv, true);
#endif

  FileIO file_io(argv[1]);

  IndoorPolygon indoor_polygon(file_io.GetIndoorPolygon());
  
  vector<Panorama> panoramas;
  {
    ReadPanoramas(file_io, &panoramas);
  }

  vector<Detection> detections;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetObjectDetections().c_str());
    if (!ifstr.is_open()) {
      cerr << "File cannot be opened: " << file_io.GetObjectDetections() << endl;
      return 1;
    }
    ifstr >> detections;
    ifstr.close();
  }
  
  vector<PointCloud> object_point_clouds;
  {
    Floorplan floorplan(file_io.GetFloorplan());
    ReadObjectPointClouds(file_io, floorplan.GetNumRooms(), &object_point_clouds);
  }
  
  // Compute an object id map for each panorama.
  vector<vector<ObjectId> > object_id_maps;
  {
    RasterizeObjectIds(panoramas, object_point_clouds, &object_id_maps);
  }
  
  // For each detection, find the most relevant object id.
  map<ObjectId, int> object_to_detection;
  AssociateObjectId(panoramas,
                    detections,
                    object_id_maps,
                    FLAGS_score_threshold,
                    FLAGS_area_threshold,
                    &object_to_detection);

  vector<Detection> detections_with_icon;
  AddIconInformationToDetections(indoor_polygon,
                                 object_point_clouds,
                                 object_to_detection,
                                 detections,
                                 &detections_with_icon);

  ofstream ofstr;
  ofstr.open(file_io.GetObjectDetectionsFinal().c_str());
  ofstr << detections_with_icon;
  ofstr.close();

  /*
  vecor<PointCloud> input_point_clouds, object_point_clouds;
  ReadPointClouds(file_io, &input_point_clouds);
  ReadObjectPointClouds(file_io, floorplan.GetNumRooms(), &object_point_clouds);

  // Accuracy and completeness.
  const RasterizedGeometry kInitial(numeric_limits<double>::max(), Vector3d(0, 0, 0), kHole);

  std::vector<std::vector<RasterizedGeometry> > rasterized_geometries;

  double depth_unit = 0.0;
  for (int p = 0; p < panoramas.size(); ++p)
    depth_unit += panoramas[p].GetAverageDistance();
  depth_unit /= panoramas.size();
  
  //----------------------------------------------------------------------
  if (FLAGS_evaluate_floorplan) {
    // Floorplan only.
    Initialize(panoramas, kInitial, &rasterized_geometries);
    RasterizeFloorplan(floorplan, panoramas, &rasterized_geometries);
    VisualizeResults(file_io, "floorplan", input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
    ReportErrors(input_point_clouds,
                 rasterized_geometries,
                 panoramas,
                 kInitial,
                 depth_unit);
  }

  
  // Indoor polygon only.
  if (FLAGS_evaluate_indoor_polygon) {
    Initialize(panoramas, kInitial, &rasterized_geometries);
    RasterizeIndoorPolygon(indoor_polygon, panoramas, &rasterized_geometries);
    VisualizeResults(file_io, "indoor_polygon", input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
    ReportErrors(input_point_clouds,
                 rasterized_geometries,
                 panoramas,
                 kInitial,
                 depth_unit);

    if (FLAGS_evaluate_object_point_clouds) {
      // Plus objects.
      RasterizeObjectPointClouds(object_point_clouds, panoramas, &rasterized_geometries);
      VisualizeResults(file_io, "object_point_clouds", input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
      ReportErrors(input_point_clouds,
                   rasterized_geometries,
                   panoramas,
                   kInitial,
                   depth_unit);
    }
  }
  */  
  return 0;
}
