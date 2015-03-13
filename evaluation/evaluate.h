#include <vector>

namespace structured_indoor_modeling {

class FileIO;
class Panorama;
class PointCloud;

void ReadInputPointClouds(const FileIO& file_io,
                          std::vector<PointCloud>* input_point_clouds);
void ReadObjectPointClouds(const FileIO& file_io,
                           const int num_rooms,
                           std::vector<PointCloud>* object_point_clouds);
void ReadPanoramas(const FileIO& file_io,
                   std::vector<Panorama>* panoramas);

}  // namespace structured_indoor_modeling
  
