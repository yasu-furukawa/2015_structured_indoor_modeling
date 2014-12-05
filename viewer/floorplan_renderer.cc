#include "floorplan_renderer.h"

#include <iostream>
#include <fstream>

#include "../calibration/file_io.h"

using namesapce std;

FloorplanRenderer::FloorplanRenderer() {
}

FloorplanRenderer::~FloorplanRenderer() {
}

void FloorplanRenderer::Init(const std::string data_directory) {
  file_io::FileIO file_io(data_directory);
  {
    ifstream ifstr;
    ifstr.open(file_io.GetFloorplan().c_str());
    ifstr >> floorplan;
    ifstr.close();
  }
  {
    ifstream ifstr;
    ifstr.open(file_io.GetRotationMat().c_str());

    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x) {
        ifstr >> rotation(y, x);
        cout << rotation(y, x) << ' ';
      }
      cout << endl;
    }
    ifstr.close();
  }
}



void FloorplanRenderer::Render(const FloorplanStyle& style) {
  for (const auto& component : floorplan.components) {
    const Shape& outer_shape = component.outer_shape;
    for (

  }


}
