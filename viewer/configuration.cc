#include <iostream>
#include "configuration.h"
#include "../base/file_io.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

void ReadPanoramaConfiguration(const string& data_directory,
                               const int pano_index,
                               PanoramaConfiguration* panorama_configuration) {
  FileIO file_io(data_directory);;
  panorama_configuration->rgb_image_filename =
    file_io.GetPanoramaImage(pano_index);

  panorama_configuration->depth_image_filename =
    file_io.GetDepthPanorama(pano_index);

  const string buffer = file_io.GetPanoramaToGlobalTransformation(pano_index);
  {
    ifstream ifstr;
    ifstr.open(buffer.c_str());
    string stmp;
    ifstr >> stmp;
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x)
        ifstr >> panorama_configuration->local_to_global(y, x);
      ifstr >> panorama_configuration->center(y);
    }
    double dummy;
    for (int i = 0; i < 4; ++i)
      ifstr >> dummy;
    ifstr >> panorama_configuration->phi_range;
    ifstr.close();
    // panorama_configuration->rho_per_pixel = 0.004363323;
  }  

}

void ReadConfiguration(const string filename, Configuration* configuration) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  ifstr >> configuration->data_directory;

  int pano_start_id, pano_end_id;
  ifstr >> pano_start_id >> pano_end_id;

  for (int p = pano_start_id; p < pano_end_id; ++p) {
    PanoramaConfiguration panorama_configuration;
    ReadPanoramaConfiguration(configuration->data_directory, p, &panorama_configuration);
    configuration->panorama_configurations.push_back(panorama_configuration);
  }

  ifstr.close();
}

}  // namespace structured_indoor_modeling
  
