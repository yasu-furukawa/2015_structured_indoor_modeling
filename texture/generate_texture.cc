#include <fstream>
#include <iostream>

#include "generate_texture.h"
#include "../base/file_io.h"
#include "../base/panorama.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

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
  
void ReadPanoramas(const FileIO& file_io,
                   const int start_panorama,
                   const int end_panorama,
                   const int num_pyramid_levels,
                   vector<vector<Panorama> >* panoramas) {
  panoramas->resize(end_panorama - start_panorama);
  cout << "Reading panoramas" << flush;
  for (int p = start_panorama; p < end_panorama; ++p) {
    cout << '.' << flush;
    const int p_index = p - start_panorama;
    panoramas->at(p_index).resize(num_pyramid_levels);
    for (int level = 0; level < num_pyramid_levels; ++level) {
      panoramas->at(p_index)[level].Init(file_io, p);
      panoramas->at(p_index)[level].MakeOnlyBackgroundBlack();
      if (level != 0) {
        const int new_width  = panoramas->at(p_index)[level].Width()  / (0x01 << level);
        const int new_height = panoramas->at(p_index)[level].Height() / (0x01 << level);
        panoramas->at(p_index)[level].ResizeRGB(Vector2i(new_width, new_height));
      }
    }
  }
  cout << " done." << endl;
}

void ReadPanoramaToGlobals(const FileIO& file_io,
                           const int start_panorama,
                           const int end_panorama,
                           vector<Matrix4d>* panorama_to_globals) {
  panorama_to_globals->resize(end_panorama - start_panorama);
  for (int p = start_panorama; p < end_panorama; ++p) {
    const int p_index = p - start_panorama;
    const string filename = file_io.GetPanoramaToGlobalTransformation(p);
    ifstream ifstr;
    ifstr.open(filename.c_str());
    if (!ifstr.is_open()) {
      cerr << "Cannot open a file: " << filename << endl;
      exit (1);
    }
    string header;
    ifstr >> header;
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        ifstr >> (*panorama_to_globals)[p_index](y, x);
      }
    }
    ifstr.close();
  }
}

void ReadPointClouds(const FileIO& file_io,
                     const int start_panorama,
                     const int end_panorama,
                     std::vector<PointCloud>* point_clouds) {
  cout << "Reading pointclouds" << flush;
  point_clouds->resize(end_panorama - start_panorama);
  for (int p = start_panorama; p < end_panorama; ++p) {
    cout << '.' << flush;
    const int p_index = p - start_panorama;
    point_clouds->at(p_index).Init(file_io, p);
    point_clouds->at(p_index).ToGlobal(file_io, p);
  }
  cout << " done." << endl;
}  

void Invert(const vector<Matrix4d>& panorama_to_globals,
            vector<Matrix4d>* global_to_panoramas) {
  global_to_panoramas->resize(panorama_to_globals.size());
  for (int i = 0; i < panorama_to_globals.size(); ++i) {
    Matrix3d rotation = panorama_to_globals[i].block(0, 0, 3, 3);
    global_to_panoramas->at(i).block(0, 0, 3, 3) = rotation.transpose();
    global_to_panoramas->at(i).block(0, 3, 3, 1) =
      - rotation.transpose() * panorama_to_globals[i].block(0, 3, 3, 1);
    global_to_panoramas->at(i)(3, 0) = 0.0;
    global_to_panoramas->at(i)(3, 1) = 0.0;
    global_to_panoramas->at(i)(3, 2) = 0.0;
    global_to_panoramas->at(i)(3, 3) = 1.0;
  }
}

}  // namespace structured_indoor_modeling
  
