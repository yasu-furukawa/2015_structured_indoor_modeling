#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

namespace structured_indoor_modeling {

// Object detection is defined by the panorama image id and the
// bounding box inside it. Bounding box coordinates (c0, c1, c2, c3)
// are given by the ratio (u,v) whose values range from 0 to 1. A
// bounding box may cross the right and left borders of an
// image. Therefore, c1.u could be smaller than c0.u.
//
// To avoid redundancy, the bounding box coordinates are given by 4
// numbers. The left and the right "u" values. The top and the bottom
// "v" values.
//
//        Panorama image
// (0,0)                    (1.0)
//   +------------------------+
//   |                        |
//   |                        |
//   |      c0    c1          |
//   |       +----+           |
//   |       |    |           |
//   |       +----+           |
//   |      c3    c2          |
//   +------------------------+
// (0,1)                    (1,1)
//
// 
//
struct Detection {
  Detection() {
    panorama = -1;

    score = 0.0;

    room = -1;
    object = -1;
  }

  //----------------------------------------------------------------------
  // Input of the pipeline (or output of the RCNN).
  //----------------------------------------------------------------------
  int panorama;
  // Left and right u values. us[0] could be smaller than us[1].
  Eigen::Vector2d us;
  // Top and bottom v values. vs[0] is always saller than vs[1].
  Eigen::Vector2d vs;

  // Output of the object detector. Name and the detection score.
  std::string name;
  double score;

  // Associated icon information.
  int room;
  int object;
  // Bounding box in the floorplan coordinate.
  Eigen::Vector2d ranges[3];
};

std::istream& operator >>(std::istream& istr, Detection& detection);
std::ostream& operator <<(std::ostream& ostr, const Detection& detection);
std::istream& operator >>(std::istream& istr, std::vector<Detection>& detections);
std::ostream& operator <<(std::ostream& ostr, const std::vector<Detection>& detections);

}  // namespace structured_indoor_modeling
