#include "line.h"

using namespace std;
using namespace ImageProcess;

std::istream& operator>>(std::istream& istr, ImageProcess::Cline& line) {
  std::string header;
  istr >> header;
  istr >> line.m_abc >> line.m_endPoints[0] >> line.m_endPoints[1];
  return istr;
};

std::ostream& operator<<(std::ostream& ostr, const ImageProcess::Cline& line) {
  ostr << "LINE" << std::endl
       << line.m_abc << ' '
       << line.m_endPoints[0] << ' '
       << line.m_endPoints[1] << std::endl;
  return ostr;
};

Vec3f Cline::grad2dir(const Vec3f& grad) {
  return Vec3f(grad[1], -grad[0], 0.0f);
}
