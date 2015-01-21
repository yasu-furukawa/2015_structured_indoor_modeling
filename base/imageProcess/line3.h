#ifndef IMAGEPROCESS_LINE3_H
#define IMAGEPROCESS_LINE3_H

#include <boost/shared_ptr.hpp>
#include <image/photo.h>
#include <numeric/vec4.h>
#include "line.h"

namespace ImageProcess {

class Cline3;
typedef boost::shared_ptr<Cline3> Pline3;

class Cline3 {
 public:
  Cline3(void);
  virtual ~Cline3();

  inline Vec4f getCoord(const float x) const{
    return m_origin + x * m_direction;
  };

  static Vec4f unproject(const Vec3f& lhs, const Cline& rhs,
                         const Image::Cphoto& photo0,
                         const Image::Cphoto& photo1);
  
  static float getPos(const Vec3f& iorg, const Vec3f& idir, const Vec3f& icoord);

  static int isClose(const Cline3& lhs, const Cline3& rhs,
                     const float threshold);

  //----------------------------------------------------------------------
  void init(const std::vector<Vec4f>& coords,
            const std::vector<int>& ids);
  
  // Compute a projection of a coordinate onto a line (ignore ranges)
  Vec4f snap(const Vec4f& coord) const;
  
  // Set m_vimages
  void setVimages(void);

  // Set m_oranges
  void setOranges(void);

  //----------------------------------------------------------------------
  // For octree
  int checkBox(const Vec3f& mmin, const Vec3f& mmax) const;
  float error(const Vec4f& coord) const;
  
  // Any point on a line is represented by m_origin + x * m_direction
  // Origin
  Vec4f m_origin;
  // Direction
  Vec4f m_direction;

  // Final range
  std::vector<Vec2f> m_range;

  // For each image, inside ranges, and associated line ids
  std::vector<std::vector<Vec3f> > m_ranges;

  // A set of visible images (should be consistent with m_ranges)
  std::vector<int> m_vimages;
  
  // Intersection between m_range and m_ranges
  // (startrange, lastrange, edgeid)
  std::vector<std::vector<Vec3f> > m_oranges;
  
  // Average color on both sides
  Vec3f m_color0;
  Vec3f m_color1;
  
  // How good the 3d line is (integration of visible images + range)
  float m_score;
  
 protected:
};
};

std::istream& operator>>(std::istream& istr,
                         ImageProcess::Cline3& line);
std::ostream& operator<<(std::ostream& ostr,
                         const ImageProcess::Cline3& line);

#endif // IMAGEPROCESS_LINE3_H
