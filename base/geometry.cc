#include "geometry.h"

using namespace std;

namespace structured_indoor_modeling {

istream& operator>>(std::istream& istr, Triangle& triangle) {
  for (int i = 0; i < 3; ++i) {
    istr >> triangle.indices[i];
  }
  istr >> triangle.image_index;
  for (int i = 0; i < 3; ++i) {
    istr >> triangle.uvs[i][0] >> triangle.uvs[i][1];
  }
  return istr;
}

ostream& operator<<(std::ostream& ostr, const Triangle& triangle) {
  for (int i = 0; i < 3; ++i) {
    ostr << triangle.indices[i] << ' ';
  }
  ostr << triangle.image_index << ' ';
  for (int i = 0; i < 3; ++i) {
    ostr << triangle.uvs[i][0] << ' ' << triangle.uvs[i][1] << ' ';
  }
  ostr << endl;
  return ostr;
}
   
}  // namespace structured_indoor_modeling

