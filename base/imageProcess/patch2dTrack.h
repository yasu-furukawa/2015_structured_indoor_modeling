#ifndef IMAGEPROCESS_PATCH2DTRACK_H
#define IMAGEPROCESS_PATCH2DTRACK_H

#include <vector>
#include <string>
#include "../numeric/vec2.h"
#include "../numeric/mat2.h"
#include "patch2d.h"

namespace ImageProcess {

class Cpatch2dTrack {
 public:
  Cpatch2dTrack(void);
  virtual ~Cpatch2dTrack();

  void resize(const int fnum);

  // Get 2d global coordinate from a local one
  inline Vec2f getGlobal(const int frame, const Vec2f& local) const;
  // Get 2d local coordinate from a global one
  inline Vec2f getLocal(const int frame, const Vec2f& global) const;

  std::vector<Cpatch2d> m_patch2ds;
  
  int m_fnum;
  
 protected:

};

std::istream& operator>>(std::istream& istr, Cpatch2dTrack& rhs);
std::ostream& operator<<(std::ostream& ostr, Cpatch2dTrack& rhs); 
 
Vec2f Cpatch2dTrack::getGlobal(const int frame, const Vec2f& local) const {
  return m_patch2ds[frame].getGlobal(local);
};

Vec2f Cpatch2dTrack::getLocal(const int frame, const Vec2f& global) const {
  return m_patch2ds[frame].getLocal(global);
};
 
};

#endif // IMAGEPROCESS_PATCH2DTRACK_H
