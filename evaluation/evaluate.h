#include <vector>

namespace structured_indoor_modeling {

class FileIO;
class PointCloud;

void ReadInputPointClouds(const FileIO& file_io,
                          std::vector<PointCloud>* input_point_clouds);
void ReadObjectPointClouds(const FileIO& file_io,
                           const int num_rooms,
                           std::vector<PointCloud>* object_point_clouds);

}  // namespace structured_indoor_modeling
  
