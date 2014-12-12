#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "../floorplan/file_io.h"

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

  void ReadPanoramaToGlobals(const file_io:
  std::string GetPanoramaToGlobalTransformation(const int panorama) const {  

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
    
  file_io::FileIO file_io(argv[1]);

  vector<cv::Mat> panoramas;
  vector<Matrix4d> panorama_to_globals;
  {
    ReadPanoramas(file_io,
                  FLAGS_num_panoramas,
                  FLAGS_num_pyramid_levels,
                  &panoramas);
    
    ReadPanoramaToGlobals(file_io, FLAGS_num_panorams, &panorama_to_globals);
  }

}
