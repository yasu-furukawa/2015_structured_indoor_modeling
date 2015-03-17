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
  int panorama;
  // Left and right u values. us[0] could be smaller than us[1].
  Eigen::Vector2d us;
  // Top and bottom v values. vs[0] is always saller than vs[1].
  Eigen::Vector2d us;

  // Output of the object detector. Name and the detection score.
  string name;
  double score;
};

istream& operator >>(istream& istr, Detection& detection);
ostream& operator <<(ostream& ostr, const Detection& detection);
istream& operator >>(istream& istr, vector<Detection>& detections);
ostream& operator <<(ostream& ostr, const vector<Detection>& detections);

}  // namespace structured_indoor_modeling
