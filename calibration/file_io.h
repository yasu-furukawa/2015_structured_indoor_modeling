#ifndef FILE_IO_H__
#define FILE_IO_H__

namespace file_io {

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
    sprintf(buffer, "%s/ply/%03d.ply", data_directory.c_str(), panorama + 1);
    return buffer;
  }
  std::string GetLocalToGlobalTransformation(const int panorama) const {
    sprintf(buffer, "%s/transformations/%03d.txt", data_directory.c_str(), panorama + 1);
    return buffer;
  }
  std::string GetMeta(const int panorama) const {
    sprintf(buffer, "%s/data/%03d/meta.txt", data_directory.c_str(), panorama + 1);
    return buffer;
  }

  std::string GetPanoramaImage(const int panorama) const {
    sprintf(buffer, "%s/panorama/%03d.png", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetImageAlignmentCalibration(const int panorama) const {
    sprintf(buffer, "%s/calibration/%03d.calibration", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetPanoramaDepthAlignmentCalibration(const int panorama) const {
    sprintf(buffer, "%s/calibration/%03d.calibration2", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetPanoramaDepthAlignmentVisualization(const int panorama) const {
    sprintf(buffer, "%s/panorama/%03d.jpg", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetPanoramaToGlobalTransformation(const int panorama) const {
    sprintf(buffer, "%s/calibration/%03d.camera_to_global", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetDepthPanorama(const int panorama) const {
    sprintf(buffer, "%s/panorama/%03d.depth", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetDepthVisualization(const int panorama) const {
    sprintf(buffer, "%s/panorama/%03d_depth.png", data_directory.c_str(), panorama);
    return buffer;
  }
  std::string GetFloorplan() const {
    sprintf(buffer, "%s/floorplan.txt", data_directory.c_str());
    return buffer;
  }
  /*
  std::string GetFloorplan() const {
    sprintf(buffer, "%s/floorplan.dat", data_directory.c_str());
    return buffer;
  }
  */
  /*
  std::string GetRotationMat() const {
    sprintf(buffer, "%s/rotationmat.txt", data_directory.c_str());
    return buffer;
  }
  */
  std::string GetRoomThumbnail(const int room) const {
    sprintf(buffer, "%s/panorama/room_thumbnail%03d.png", data_directory.c_str(), room);
    return buffer;
  }
  
 private:
  const std::string data_directory;
  mutable char buffer[1024];
};
};
#endif  // FILE_IO_H__
