#ifndef IMAGEPROCESS_LINEEXTRACTER_H
#define IMAGEPROCESS_LINEEXTRACTER_H

#include <image/image.h>
#include "line.h"
#include <vector>

namespace ImageProcess {
class ClineExtracter {
 public:
  ClineExtracter(void);
  virtual ~ClineExtracter();

  void extract(Image::Cimage& eImage, const int level, const float minLen);

  void write(const std::string& file) const;
  
  std::vector<Cline> m_lines;
  
 protected:
  void BDF(void);
  void smooth(void);
  void nonMaxSuppress(void);

  void extractLines(Image::Cimage& eImage);
  void extractLine(Image::Cimage& eImage, Cline& line);

  void getInOut(Image::Cimage& eImage, const Cline& line,
                std::vector<int>& inout) const;

  void fillIn(const int scanwhich, std::vector<int>& inout) const;
  
  void removeEdge(Image::Cimage& eImage, const Cline& line);
  
  int theta2cell(const float theta) const;
  float cell2theta(const int cell) const;
  int rho2cell(const float rho) const;
  float cell2rho(const int cell) const;
  
  // Votes
  std::vector<std::vector<float> > m_votes;
  // temporary
  std::vector<std::vector<float> > m_tmp;

  // Removed pixel map
  std::vector<std::vector<std::vector<Vec3f> > > m_removed;
  
  // Units for the voting space
  float m_thetaUnit;
  float m_rhoUnit;
  // Width and height of the voting space (theta, rho)
  int m_thetaWidth;
  int m_rhoHeight;
  int m_rhoOffset;
  
  // level
  int m_level;
  // Width and height of the image
  int m_width;
  int m_height;
  Vec2f m_center;

  // Thresholds
  // Gap allowed in a line segment
  int m_gap;
  // Minimum length of a line segment allowed
  float m_minLen;



  //void squareFilter(Image::Cimage& eImage);
  void boxFilter(Image::Cimage& eImage);
  void gaussFilter(Image::Cimage& eImage);
};
};
#endif // IMAGEPROCESS_LINEEXTRACTER_H
