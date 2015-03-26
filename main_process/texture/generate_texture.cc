#include <fstream>
#include <iostream>

#include "generate_texture.h"
#include "../../base/file_io.h"
#include "../../base/panorama.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

void ReadPanoramaToGlobals(const FileIO& file_io,
                           vector<Matrix4d>* panorama_to_globals) {
  const int num_panoramas = GetNumPanoramas(file_io);
  
  panorama_to_globals->resize(num_panoramas);
  for (int p = 0; p < num_panoramas; ++p) {
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
        ifstr >> (*panorama_to_globals)[p](y, x);
      }
    }
    ifstr.close();
  }
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
  
