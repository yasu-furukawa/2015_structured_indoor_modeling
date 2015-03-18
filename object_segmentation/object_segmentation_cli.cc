#include <Eigen/Dense>
#include <fstream>
#include "gflags/gflags.h"
#include <iostream>
#include <map>
#include <vector>

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "../base/point_cloud.h"
#include "object_segmentation.h"

DEFINE_double(point_subsampling_ratio, 1.0, "Make the point set smaller.");
DEFINE_double(centroid_subsampling_ratio, 0.005, "Ratio of centroids in each segment.");
DEFINE_double(num_initial_clusters, 100, "Initial cluster.");

using namespace Eigen;
using namespace structured_indoor_modeling;
using namespace std;

namespace {

void ReportSegments(const std::vector<int>& segments) {
  int object = 0;
  int initial = 0;
  int floor = 0;
  int wall = 0;
  int ceiling = 0;
  int detail = 0;
  for (int s = 0; s < segments.size(); ++s) {
    if (segments[s] == kInitial)
      ++initial;
    else if (segments[s] == kFloor)
      ++floor;
    else if (segments[s] == kWall)
      ++wall;
    else if (segments[s] == kCeiling)
      ++ceiling;
    else if (segments[s] == kDetail)
      ++detail;
    else {
      ++object;
    }
  }
  
  cerr << "Object " << object
       << " Initial " << initial
       << " Floor " << floor
       << " wall " << wall
       << " ceiling " << ceiling
       << " detail " << detail << endl;
}
    
bool ProcessRoom(const FileIO& file_io,
                 const int room,
                 const Floorplan& floorplan,
                 const IndoorPolygon& indoor_polygon,
                 const std::vector<PointCloud>& point_clouds,
                 const std::vector<int>& room_occupancy) {
  cout << "Room: " << room << endl;
  vector<Point> points;
  CollectPointsInRoom(point_clouds, floorplan, room_occupancy, room, &points);
  if (points.empty())
    return false;
  cout << "Filtering... " << points.size() << " -> " << flush;
  FilterNoisyPoints(&points);
  cout << points.size() << " done." << endl;
  
  if (points.empty())
    return false;
  
  if (FLAGS_point_subsampling_ratio != 1.0) {
    cout << "Subsampling... " << points.size() << " -> " << flush;
    Subsample(FLAGS_point_subsampling_ratio, &points);
    cout << points.size() << " done." << endl;
    if (points.empty())
      return false;
  }
  
  // For each point, initial segmentation.
  vector<int> segments;
  cerr << "Checking floor/wall/ceiling..." << flush;
  IdentifyFloorWallCeiling(points, floorplan, room, &segments);
  cerr << "done." << endl
       << "Checking details..." << flush;
  IdentifyDetails(points, floorplan, indoor_polygon, room, &segments);
  cerr << "done." << endl;

  ReportSegments(segments);
  
  // Compute neighbors.
  vector<vector<int> > neighbors;
  const int kNumNeighbors = 8;
  cout << "SetNeighbors..." << flush;
  SetNeighbors(points, kNumNeighbors, &neighbors);
  cout << "done." << endl;

  ReportSegments(segments);

  cout << "SegmentObjects..." << flush;
  SegmentObjects(points, FLAGS_centroid_subsampling_ratio, FLAGS_num_initial_clusters, neighbors,
                 &segments);
  cout << "done." << endl;
  
  ReportSegments(segments);
  
  cout << "SmoothObjects..." << flush;
  const int kSmoothTime = 5;
  for (int t = 0; t < kSmoothTime; ++t)
    SmoothObjects(neighbors, &points);
  cout << "done." << endl;

  ReportSegments(segments);
  
  // This may not be necessary.
  //DensifyObjects(neighbors, &points, &segments);
  // Neighbors are dirty here.
  
  char buffer[1024];
  {
    map<int, Vector3i> color_table;
    printf("%s\n", file_io.GetObjectPointClouds(room).c_str());
    WriteObjectPointsWithColor(points, segments, file_io.GetObjectPointClouds(room),
                               floorplan.GetFloorplanToGlobal(),
                               &color_table);
    cout << "done." << endl;
  }
  {
    printf("%s\n", file_io.GetFloorWallPointClouds(room).c_str());
    WriteOtherPointsWithColor(points, segments, file_io.GetFloorWallPointClouds(room),
                              floorplan.GetFloorplanToGlobal());
    cout << "done." << endl;
  }
  return true;
}

Vector3d GetCenter(const FileIO& file_io, const int panorama) {
  ifstream ifstr;
  ifstr.open(file_io.GetLocalToGlobalTransformation(panorama).c_str());
  
  char ctmp;
  ifstr >> ctmp;
  Vector3d center;
  for (int i = 0; i < 3; ++i) {
    double dtmp;
    for (int x = 0; x < 3; ++x)
      ifstr >> dtmp;
    ifstr >> center[i];
  }
  ifstr.close();

  return center;
}
  
}  // namespace

//----------------------------------------------------------------------
// Main
//----------------------------------------------------------------------
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
  
  Floorplan floorplan;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetFloorplan().c_str());
    ifstr >> floorplan;
    ifstr.close();
  }
  IndoorPolygon indoor_polygon;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetIndoorPolygon().c_str());
    ifstr >> indoor_polygon;
    ifstr.close();
  }

  // Make a 2D image with room occupancy information.
  vector<int> room_occupancy;
  SetRoomOccupancy(floorplan, &room_occupancy);
  vector<int> room_occupancy_with_doors = room_occupancy;
  SetDoorOccupancy(floorplan, &room_occupancy_with_doors);
  
  const int num_panoramas = GetNumPanoramas(file_io);
  vector<PointCloud> point_clouds(num_panoramas);
  cout << "Reading point clouds..." << flush;
  for (int p = 0; p < num_panoramas; ++p) {
    cout << '.' << flush;
    const int index = p;
    if (!point_clouds[index].Init(file_io, p)) {
      cerr << "Failed in loading the point cloud." << endl;
      exit (1);
    }
    // Make the 3D coordinates into the floorplan coordinate system.
    point_clouds[index].ToGlobal(file_io, p);
    const Matrix3d global_to_floorplan = floorplan.GetFloorplanToGlobal().transpose();
    point_clouds[index].Rotate(global_to_floorplan);

    const Vector3d global_center = GetCenter(file_io, p);

    RemoveWindowAndMirror(floorplan,
                          room_occupancy_with_doors,
                          global_to_floorplan * global_center,
                          &point_clouds[index]);
  }
  cout << "done." << endl;
    
  // Per room processing.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    ProcessRoom(file_io, room, floorplan, indoor_polygon, point_clouds, room_occupancy);
  }
}
