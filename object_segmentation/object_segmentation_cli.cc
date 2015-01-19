#include <Eigen/Dense>
#include <fstream>
#include "gflags/gflags.h"
#include <iostream>
#include <vector>

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/point_cloud.h"
#include "object_segmentation.h"

DEFINE_int32(start_panorama, 0, "start panorama index");
DEFINE_int32(end_panorama, 1, "end panorama index");
DEFINE_double(point_subsampling_ratio, 0.2, "Make the point set smaller.");
DEFINE_double(centroid_subsampling_ratio, 0.005, "Ratio of centroids in each segment.");
DEFINE_double(num_initial_clusters, 200, "Initial cluster.");

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
  //???
  // for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
  for (int room = 0; room < 1; ++room) {
    cout << "Room: " << room << endl;
    vector<Point> points;
    CollectPointsInRoom(point_clouds, floorplan, room_occupancy, room, &points);
    cerr << "Filtering... " << points.size() << " -> " << flush;
    FilterNoisyPoints(&points);
    cerr << points.size() << " done." << endl
         << "Subsampling... " << points.size() << " -> " << flush;
    Subsample(FLAGS_point_subsampling_ratio, &points);
    cerr << points.size() << " done." << endl;
    // For each point, initial segmentation.
    vector<int> segments;
    IdentifyFloorWallCeiling(points, floorplan, room_occupancy, room, &segments);
    
    SegmentObjects(points, FLAGS_centroid_subsampling_ratio, FLAGS_num_initial_clusters, &segments);


    /*
    vector<Point> object_points;
    for (int p = 0; p < points.size(); ++p) {
      switch (segments[p]) {
      case kFloor: {
        points[p].color = Vector3f(0, 0, 255);
        break;
      }
      case kWall: {
        points[p].color = Vector3f(0, 255, 0);
        break;
      }
      case kCeiling: {
        points[p].color = Vector3f(255, 0, 0);
        break;
      }
      default: {
        points[p].color[0] = segments[p] * 30 % 255;
        points[p].color[1] = segments[p] * 50 % 255;
        points[p].color[2] = segments[p] * 70 % 255;
        
        
        object_points.push_back(points[p]);
      }
      }
    }

    PointCloud pc;
    pc.SetPoints(points);
    char buffer[1024];
    sprintf(buffer, "room_%03d.ply", room);
    pc.Write(buffer);


    pc.SetPoints(object_points);
    sprintf(buffer, "object_%03d.ply", room);
    pc.Write(buffer);
    */
  }

  
}
