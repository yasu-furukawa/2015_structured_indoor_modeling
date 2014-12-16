#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "../calibration/file_io.h"
#include "../floorplan/floorplan.h"

DEFINE_int32(num_panoramas, 1, "Number of panorama images.");
DEFINE_int32(num_pyramid_levels, 3, "Num pyramid levels.");

using namespace std;
using namespace Eigen;

void ReadPanoramas(const file_io::FileIO& file_io,
                   const int num_panoramas,
                   const int num_pyramid_levels,
                   vector<vector<cv::Mat> >* panoramas) {
  panoramas->resize(num_pyramid_levels);

  const int kFirstLevel = 0;
  panoramas->at(kFirstLevel).resize(num_panoramas);

  for (int p = 0; p < num_panoramas; ++p) {
    (*panoramas)[kFirstLevel][p] = cv::imread(file_io.GetPanoramaImage(p), 1);
    if ((*panoramas)[kFirstLevel][p].empty()) {
      cerr << "Image does not exist." << endl;
      exit (1);      
    }
  }
  for (int level = 1; level < num_pyramid_levels; ++level) {
    panoramas->at(level).resize(num_panoramas);
    for (int p = 0; p < num_panoramas; ++p) {
      pyrDown((*panoramas)[level - 1][p], (*panoramas)[level][p],
              Size((*panoramas)[level - 1][p].cols / 2,
                   (*panoramas)[level - 1][p].rows / 2));
    }
  }
}

void ReadPanoramaToGlobals(const file_io::FileIO& file_io,
                           const int num_panoramas,
                           vector<Matrix4d>* panorama_to_globals) {
  panorama_to_globals->resize(num_panoramas);
  for (int p = 0; p < num_panoramas; ++p) {
    const string filename = file_io.GetPanoramaToGlobalTransformation(p);
    ifstream ifstr;
    ifstr.open(filename.c_str());
    string header;
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
  global_to_panoramas.resize(panorama_to_globals.size());
  for (int i = 0; i < panorama_to_globals.size(); ++i) {
    Matrix3d rotation = panorama_to_globals[i].block(0, 0, 3, 3);
    global_to_panoramas->at(i).block(0, 0, 3, 3) = rotation.transpose();
    global_to_panoramas->at(i).block(0, 3, 3, 1) =
      - rotation.transpose() * panorama_to_globals[i].block(0, 3, 3, 1);
    global_to_panoramas->at(i)(3, 0) = 0.0;
    global_to_panoramas->at(i)(3, 1) = 0.0;
    global_to_panoramas->at(i)(3, 2) = 0.0;
    global_to_panoramas->at(i)(3, 3) = 1.0;
    
    // debug.
    Matrix4d a = global_to_panoramas->at(i) * panorama_to_globals[i];
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x)
        cout << a(y, x) << ' ';
      cout << endl;
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Read data from the directory.
  file_io::FileIO file_io(argv[1]);
  vector<vector<cv::Mat> > panoramas;
  {
    ReadPanoramas(file_io,
                  FLAGS_num_panoramas,
                  FLAGS_num_pyramid_levels,
                  &panoramas);
  }
  vector<Matrix4d> panorama_to_globals;
  {
    ReadPanoramaToGlobals(file_io, FLAGS_num_panorams, &panorama_to_globals);
   }
  Floorplan floorplan;
  {
    const string filename = file_io.GetFloorplan();
    ifstream ifstr;
    ifstr.open(filename.c_str());
    ifstr >> floorplan;
    ifstr.close();
  }
  vector<Matrix4d> global_to_panoramas;
  {
    Invert(panorama_to_globals, &global_to_panoramas);
  }

  // For each wall rectangle, floor, and ceiling,
  // 1. grab texture
  // 2. stitch
  
  
  
  
  
}
