/*
  Files for each dataset are stored in the same format. This file
  specifies the naming schemes and provides APIs to get filenames.

  Whenenever one adds a new directory or defines new data files, one
  must adds an entry to this file.  Never directly construct a
  filename and read/write, unless that is for temporary debugging purposs. 

  Numbers start from 0 (C++ convention). So, the first panorama image
  name can be accessed by FileIO::GetPanoramaImage(0).
  
  < Example >
  FileIO file_io("/Users/furukawa/data/office0");
  cout << "First panorama: " << file_io.GetPanoramaImage(0) << endl;
*/

#ifndef FILE_IO_H__
#define FILE_IO_H__

#include <fstream>
#include <stdio.h>
#include <string>
namespace structured_indoor_modeling {

class FileIO {
 public:
 FileIO(const std::string data_directory) : data_directory(data_directory) {
  }

  std::string GetDataDirectory() const {
    return data_directory;
  }
  std::string GetRawImage(const int panorama, const int image, const int dynamic_range_index) const {
    sprintf(buffer, "%s/data/%03d/%02d_%d.jpg",
            data_directory.c_str(), panorama + 1, image + 1, dynamic_range_index);
    return buffer;
  }
  std::string GetLocalPly(const int panorama) const {
    sprintf(buffer, "%s/input/ply/%03d.ply", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetSuperPixelFile(const int panorama) const{
      sprintf(buffer, "%s/input/panorama/SLIC%03d", data_directory.c_str(), panorama);
      return buffer;
  }
  std::string GetLocalToGlobalTransformation(const int panorama) const {
    sprintf(buffer, "%s/input/transformations/%03d.txt", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetMeta(const int panorama) const {
    sprintf(buffer, "%s/data/%03d/meta.txt", data_directory.c_str(), panorama + 1);
    return buffer;
  }

  std::string GetPanoramaImage(const int panorama) const {
    sprintf(buffer, "%s/input/panorama/%03d.png", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetImageAlignmentCalibration(const int panorama) const {
    sprintf(buffer, "%s/input/calibration/%03d.calibration", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetPanoramaDepthAlignmentCalibration(const int panorama) const {
    sprintf(buffer, "%s/input/calibration/%03d.calibration2", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetPanoramaDepthAlignmentVisualization(const int panorama) const {
    sprintf(buffer, "%s/input/panorama/%03d.jpg", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetPanoramaToGlobalTransformation(const int panorama) const {
    sprintf(buffer, "%s/input/calibration/%03d.camera_to_global", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetDepthPanorama(const int panorama) const {
    sprintf(buffer, "%s/input/panorama/%03d_raw.depth", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetDepthVisualization(const int panorama) const {
    sprintf(buffer, "%s/input/panorama/%03d_raw_depth.png", data_directory.c_str(), panorama);
    return buffer;
  }
  
  std::string GetSmoothDepthPanorama(const int panorama) const {
    sprintf(buffer, "%s/input/panorama/%03d.depth", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetSmoothDepthVisualization(const int panorama) const {
    sprintf(buffer, "%s/input/panorama/%03d_depth.png", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetFloorplan() const {
    sprintf(buffer, "%s/input/floorplan.txt", data_directory.c_str());
    return buffer;
  }
  std::string GetFloorplanSVG() const {
    sprintf(buffer, "%s/floorplan/floorplan.svg", data_directory.c_str());
    return buffer;
  }
  std::string GetIndoorPolygon() const {
    // sprintf(buffer, "%s/indoor_polygon.txt", data_directory.c_str());
    sprintf(buffer, "%s/input/floorplan_detailed.txt", data_directory.c_str());
    return buffer;
  }
  std::string GetIndoorPolygonWithCeiling() const {
    sprintf(buffer, "%s/input/floorplan_detailed_ceil.txt", data_directory.c_str());
    return buffer;
  }
  std::string GetFloorplanFinal() const {
    sprintf(buffer, "%s/floorplan/floorplan_final.txt", data_directory.c_str());
    return buffer;
  }
  std::string GetIndoorPolygonFinal() const {
    // sprintf(buffer, "%s/indoor_polygon_final.txt", data_directory.c_str());
    sprintf(buffer, "%s/floorplan/floorplan_detailed_final.txt", data_directory.c_str());
    return buffer;
  }  

  std::string GetTextureImage(const int index) const {
    sprintf(buffer, "%s/texture_atlas/texture_image_%03d.png", data_directory.c_str(), index);
    return buffer;
  }

  std::string GetTextureImageIndoorPolygon(const int index) const {
    sprintf(buffer, "%s/texture_atlas/texture_image_detailed_%03d.png", data_directory.c_str(), index);
    return buffer;
  }
  
  std::string GetRoomThumbnail(const int room) const {
    sprintf(buffer, "%s/thumbnail/room_thumbnail%03d.png", data_directory.c_str(), room);
    return buffer;
  }
  std::string GetRoomThumbnailPerPanorama(const int room, const int panorama) const {
    sprintf(buffer, "%s/thumbnail/room_thumbnail_per_panorama_%03d_%03d.png",
            data_directory.c_str(), room, panorama);
    return buffer;
  }
  

  std::string GetObjectPointCloudsWithColor() const {
    sprintf(buffer, "%s/object/object_color.ply", data_directory.c_str());
    return buffer;
  }    
  std::string GetObjectPointClouds(const int room) const {
    sprintf(buffer, "%s/object/object_%03d.ply", data_directory.c_str(), room);
    return buffer;
  }
  
  std::string GetObjectPointCloudsFinal(const int room) const {
    sprintf(buffer, "%s/object/object_final_%03d.ply", data_directory.c_str(), room);
    return buffer;
  }    

  std::string GetFloorWallPointClouds(const int room) const {
    sprintf(buffer, "%s/object/floor_wall_%03d.ply", data_directory.c_str(), room);
    return buffer;
  }
  std::string GetRefinedObjectClouds(const int room) const{
    sprintf(buffer, "%s/object/object_refined_room%03d.ply", data_directory.c_str(),room);
    return buffer;
  }

  std::string GetEvaluationDirectory() const {
    sprintf(buffer, "%s/evaluation", data_directory.c_str());
    return buffer;
  }

  std::string GetObjectDetections() const {
    sprintf(buffer, "%s/input/detections.txt", data_directory.c_str());
    return buffer;
  }
  std::string GetObjectDetectionsFinal() const {
    sprintf(buffer, "%s/object_detection/detections_final.txt", data_directory.c_str());
    return buffer;
  }

  std::string GetPoissonInput() const {
    sprintf(buffer, "%s/evaluation/poisson_input.npts", data_directory.c_str());
    return buffer;
  }
  std::string GetPoissonMesh() const {
    sprintf(buffer, "%s/evaluation/poisson_mesh.ply", data_directory.c_str());
    return buffer;
  }
  std::string GetVgcutMesh() const {
    sprintf(buffer, "%s/evaluation/vgcut_mesh.ply", data_directory.c_str());
    return buffer;
  }

  std::string GetCollada() const {
    sprintf(buffer, "%s/evaluation/floorplan_detailed.dae", data_directory.c_str());
    return buffer;
  }
  std::string GetColladaWithCeiling() const {
    sprintf(buffer, "%s/evaluation/floorplan_detailed_ceil.dae", data_directory.c_str());
    return buffer;
  }

  std::string GetErrorReport(const std::string& prefix) const {
    sprintf(buffer, "%s/evaluation/error_%s.txt", data_directory.c_str(), prefix.c_str());
    return buffer;
  }

  std::string GetErrorHistogram(const std::string& prefix) const {
    sprintf(buffer, "%s/evaluation/error_histogram_%s.txt", data_directory.c_str(), prefix.c_str());
    return buffer;
  }
  
  
 private:
  const std::string data_directory;
  mutable char buffer[1024];
};

inline int GetNumPanoramas(const FileIO& file_io) {
  int panorama = 0;
  while (1) {
    const std::string filename = file_io.GetPanoramaImage(panorama);
    std::ifstream ifstr;
    ifstr.open(filename.c_str());
    if (!ifstr.is_open()) {
      ifstr.close();
      return panorama;
    }
    ifstr.close();
    ++panorama;
  }
}
 
}  // namespace structured_indoor_modeling
#endif  // FILE_IO_H__
