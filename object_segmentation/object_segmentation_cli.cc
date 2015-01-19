#include <Eigen/Dense>
#include <fstream>
#include "gflags/gflags.h"
#include <iostream>
#include <map>
#include <vector>

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/point_cloud.h"
#include "object_segmentation.h"

DEFINE_int32(start_panorama, 0, "start panorama index");
DEFINE_int32(end_panorama, 1, "end panorama index");
DEFINE_double(point_subsampling_ratio, 0.5, "Make the point set smaller.");
DEFINE_double(centroid_subsampling_ratio, 0.005, "Ratio of centroids in each segment.");
DEFINE_double(num_initial_clusters, 100, "Initial cluster.");

using namespace Eigen;
using namespace structured_indoor_modeling;
using namespace std;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);

  FileIO file_io(argv[1]);
  
  Floorplan floorplan;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetFloorplan());
    ifstr >> floorplan;
    ifstr.close();
  }

  vector<PointCloud> point_clouds(FLAGS_end_panorama - FLAGS_start_panorama);
  cerr << "Reading point clouds..." << flush;
  for (int p = FLAGS_start_panorama; p < FLAGS_end_panorama; ++p) {
    cerr << '.' << flush;
    const int index = p - FLAGS_start_panorama;
    if (!point_clouds[index].Init(file_io, p)) {
      cerr << "Failed in loading the point cloud." << endl;
      exit (1);
    }
    // Make the 3D coordinates into the floorplan coordinate system.
    point_clouds[index].ToGlobal(file_io, p);
    const Matrix3d global_to_floorplan = floorplan.GetFloorplanToGlobal().transpose();
    point_clouds[index].Rotate(global_to_floorplan);
  }
  cerr << "done." << endl;
  
  // Make a 2D image with room occupancy information.
  vector<int> room_occupancy;
  SetRoomOccupancy(floorplan, &room_occupancy);

  // Per room processing.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    cout << "Room: " << room << endl;
    vector<Point> points;
    CollectPointsInRoom(point_clouds, floorplan, room_occupancy, room, &points);
    cerr << "Filtering... " << points.size() << " -> " << flush;
    FilterNoisyPoints(&points);
    cerr << points.size() << " done." << endl
         << "Subsampling... " << points.size() << " -> " << flush;
    Subsample(FLAGS_point_subsampling_ratio, &points);
    cerr << points.size() << " done." << endl;
    if (points.empty())
      continue;
    
    // For each point, initial segmentation.
    vector<int> segments;
    IdentifyFloorWallCeiling(points, floorplan, room_occupancy, room, &segments);

    SegmentObjects(points, FLAGS_centroid_subsampling_ratio, FLAGS_num_initial_clusters, &segments);


    char buffer[1024];
    sprintf(buffer, "%s/object_%03d.ply", argv[1], room);
    {
      map<int, Vector3i> color_table;
      WriteObjectPointsWithColor(points, segments, buffer, &color_table);
    }
    sprintf(buffer, "%s/other_%03d.ply", argv[1], room);
    {
      WriteOtherPointsWithColor(points, segments, buffer);
    }
  }
}
