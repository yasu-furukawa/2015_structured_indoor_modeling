#include <cstdlib>
#include <iostream>
#include "stitch_panorama.h"

using std::cerr;
using std::endl;
using pre_process::Input;
using pre_process::StitchPanorama;

int main(int argc, char* argv[]) {
  if (argc < 4) {
    cerr << argv[0] << " directory width height" << endl;
    return 1;
  }
  
  StitchPanorama stitch_panorama;
  Input input;
  input.directory = argv[1];
  input.out_width = atoi(argv[2]);
  input.out_height = atoi(argv[3]);
  
  if (!stitch_panorama.Stitch(input)) {
    cerr << "Failed." << endl;
    return 1;
  }
  
  return 0;
}
