#ifndef IMAGEPROCESS_PATCH2D_H
#define IMAGEPROCESS_PATCH2D_H

#include "../numeric/vec3.h"
#include "../numeric/mat2.h"
#include <iostream>

namespace ImageProcess {

class Cpatch2d {
 public:
  Cpatch2d(void);
  virtual ~Cpatch2d();
  
  // Get 2d global coordinate from a local one
  inline Vec2f getGlobal(const Vec2f& local) const;
  // Get 2d local coordinate from a global one
  inline Vec2f getLocal(const Vec2f& global) const;
  
  // center
  Vec2f m_center;
  // x and y axes
  Vec2f m_xaxis;
  Vec2f m_yaxis;

  // score
  // 2.0 means failure
  float m_score;

  // id
  int m_id;
  // number of neighbors
  int m_num;
  // speed
  float m_speed;

  inline int isTracked(void) const{
    if (m_score == 2.0)
      return 0;
    else
      return 1;
  };

  inline float det(void) const{
    return m_xaxis[0] * m_yaxis[1] - m_xaxis[1] * m_yaxis[0];
  };
  
  // 2-fold ambiguity is obtained by flipping the sign of the third
  // elements of r0 and r1, respectively.
  void lift(float& z, Vec3f& r0, Vec3f& r1) const;
  // A is multiplied from the left hand side  
  void lift(const Mat2f& A, float& z, Vec3f& r0, Vec3f& r1) const;
 protected:
  static void lift(const Vec2f& xaxis, const Vec2f& yaxis,
		   float& z, Vec3f& r0, Vec3f& r1);
};

std::istream& operator>>(std::istream& istr, Cpatch2d& rhs);
std::ostream& operator<<(std::ostream& ostr, Cpatch2d& rhs); 
 
Vec2f Cpatch2d::getGlobal(const Vec2f& local) const {
  return local[0] * m_xaxis + local[1] * m_yaxis + m_center;
};

Vec2f Cpatch2d::getLocal(const Vec2f& global) const {
  const Vec2f ans = global - m_center;
  Mat2f mat(m_xaxis[0], m_yaxis[0], m_xaxis[1], m_yaxis[1]);
  Mat2f imat;
  invert(imat, mat);
  return imat * ans;
};
 
};

#endif // IMAGEPROCESS_PATCH2D_H

