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
#include "evaluate.h"

DEFINE_string(floorplan_file, "", "Floorplan filename.");
DEFINE_string(indoor_polygon_file, "", "Indoor_polygon filename.");

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

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
  
  string floorplan_file;
  if (FLAGS_floorplan_file == "") {
    floorplan_file = file_io.GetFloorplan();
  } else {
    char buffer[1024];
    sprintf(buffer, "%s/%s", argv[1], FLAGS_floorplan_file.c_str());
    floorplan_file = buffer;
  }

  string indoor_polygon_file;
  if (FLAGS_indoor_polygon_file == "") {
    indoor_polygon_file = file_io.GetIndoorPolygon();
  } else {
    char buffer[1024];
    sprintf(buffer, "%s/%s", argv[1], FLAGS_indoor_polygon_file.c_str());
    indoor_polygon_file = buffer;
  }
  
  Floorplan floorplan(floorplan_file);
  IndoorPolygon indoor_polygon(indoor_polygon_file);

  vector<Panorama> panoramas;
  ReadPanoramas(file_io, &panoramas);
  // Adjust the center of panorama to the center of the laser scanner.
  for (int p = 0; p < panoramas.size(); ++p) {
    panoramas[p].AdjustCenter(GetLaserCenter(file_io, p));
  }  

  vector<PointCloud> input_point_clouds, object_point_clouds;
  ReadInputPointClouds(file_io, &input_point_clouds);
  ReadObjectPointClouds(file_io, floorplan.GetNumRooms(), &object_point_clouds);

  // Accuracy and completeness.
  const RasterizedGeometry kInitial(numeric_limits<double>::max(), Vector3d(0, 0, 0), kHole);

  std::vector<std::vector<RasterizedGeometry> > rasterized_geometries;
  //----------------------------------------------------------------------
  {
    // Floorplan only.
    Initialize(panoramas, kInitial, &rasterized_geometries);
    RasterizeFloorplan(floorplan, panoramas, &rasterized_geometries);

    {
      for (int p = 0; p < rasterized_geometries.size(); ++p) {
        const Panorama& panorama = panoramas[p];
        const int width  = panorama.DepthWidth();
        const int height = panorama.DepthHeight();
        bool first = true;
        double max_depth, min_depth;
        for (int i = 0; i < rasterized_geometries[p].size(); ++i) {
          if (rasterized_geometries[p][i].depth != kInitial.depth) {
            if (first) {
              max_depth = min_depth = rasterized_geometries[p][i].depth;
              first = false;
            } else {
              max_depth = max(max_depth, rasterized_geometries[p][i].depth);
              min_depth = min(min_depth, rasterized_geometries[p][i].depth);
            }
          }
        }

        ofstream ofstr;
        char buffer[1024];
        sprintf(buffer, "%03d.ppm", p);
        ofstr.open(buffer);
        ofstr << "P3" << endl
              << width << ' ' << height << endl
              << 255 << endl;
        for (int i = 0; i < rasterized_geometries[p].size(); ++i) {
          if (rasterized_geometries[p][i].depth == kInitial.depth) {
            ofstr << "255 0 0 ";
          } else {
            const int itmp = (int)(255 * (rasterized_geometries[p][i].depth - min_depth) / (max_depth - min_depth));
            ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
          }
        }
        ofstr.close();
      }
    }
    
    ReportErrors(input_point_clouds,
                 rasterized_geometries,
                 panoramas,
                 kInitial);
  }
  /*
  // Indoor polygon only.
  {
    Initialize(panoramas, kInitial, &rasterized_geometries);
    RasterizeIndoorPolygon(indoor_polygon, panoramas, &rasterized_geometries);
    ReportErrors(input_point_clouds,
                 rasterized_geometries,
                 panoramas,
                 kInitial);
    
    // Plus objects.
    RasterizeObjectPointClouds(object_point_clouds, panoramas, &rasterized_geometries);

    ReportErrors(input_point_clouds,
                 rasterized_geometries,
                 panoramas,
                 kInitial);
  }
  */
  return 0;
}
