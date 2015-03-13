#include <fstream>
#include <iostream>
#include "evaluate.h"
#include "../base/file_io.h"
#include "../base/panorama.h"
#include "../base/point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

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
  
}  // namespace  

void ReadInputPointClouds(const FileIO& file_io, std::vector<PointCloud>* input_point_clouds) {
  const int kStartPanorama = 0;
  const int end_panorama = GetEndPanorama(file_io, kStartPanorama);

  input_point_clouds->clear();
  input_point_clouds->resize(end_panorama);
  {
    cout << "Reading point clouds..." << flush;
    for (int p = kStartPanorama; p < end_panorama; ++p) {
      cout << '.' << flush;
      if (!input_point_clouds->at(p).Init(file_io, p)) {
        cerr << "Failed in loading the point cloud." << endl;
        exit (1);
      }
      input_point_clouds->at(p).ToGlobal(file_io, p);
    }
  }
}

void ReadObjectPointClouds(const FileIO& file_io,
                           const int num_rooms,
                           std::vector<PointCloud>* object_point_clouds) {
  object_point_clouds->clear();
  object_point_clouds->resize(num_rooms);
  for (int room = 0; room < num_rooms; ++room) {
    object_point_clouds->at(room).Init(file_io.GetRefinedObjectClouds(room));
  }
}

void ReadPanoramas(const FileIO& file_io,
                   vector<Panorama>* panoramas) {
  const int kStartPanorama = 0;
  const int end_panorama = GetEndPanorama(file_io, kStartPanorama);
  panoramas->clear();
  panoramas->resize(end_panorama);
  for (int p = kStartPanorama; p < end_panorama; ++p) {
    panoramas->at(p).Init(file_io, p);
  }
}

Vector3d GetLaserCenter(const FileIO& file_io, const int panorama) {
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

void Initialize(const vector<Panorama>& panoramas,
                const RasterizedGeometry& initial_value,
                std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {
  rasterized_geometries->clear();
  rasterized_geometries->resize(panoramas.size());

  for (int p = 0; p < (int)panoramas.size(); ++p) {
    const int width  = panoramas[p].DepthWidth();
    const int height = panoramas[p].DepthHeight();

    rasterized_geometries->at(p).resize(width * height, initial_value);
  }
}

void RasterizeFloorplan(const Floorplan& floorplan,
                        const vector<Panorama>& panoramas,
                        std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {

}

void RasterizeIndoorPolygon(const IndoorPolygon& indoor_polygon,
                            const vector<Panorama>& panoramas,
                            std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {


}

void RasterizeObjectPointClouds(const std::vector<PointCloud>& object_point_clouds,
                                const vector<Panorama>& panoramas,
                                std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {


}
  
}  // namespace structured_indoor_modeling
