#ifndef IMAGEPROCESS_LINE_H
#define IMAGEPROCESS_LINE_H

#include <vector>
#include <numeric/vec3.h>
#include <string>
#include <iostream>

namespace ImageProcess {
  
class Cline {
 public:
  Cline(void) { m_tmp = 0; };
  virtual ~Cline() {};
  
  // ax + by + c = 0.
  Vec3f m_abc;

  // Two end points
  Vec3f m_endPoints[2];

  // Temporary
  float m_tmp;
  float m_tmp2;
  int m_itmp;

  static Vec3f grad2dir(const Vec3f& grad);
 protected:
}; 
};

std::istream& operator>>(std::istream& istr,
                         ImageProcess::Cline& line);
std::ostream& operator<<(std::ostream& ostr,
                         const ImageProcess::Cline& line);

#endif //IMAGEPROCESS_LINE_H
