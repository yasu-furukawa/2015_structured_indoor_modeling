#ifndef IMAGEPROCESS_LINEORGANIZER_H
#define IMAGEPROCESS_LINEORGANIZER_H

#include <vector>
#include <string>
#include <image/photoSet.h>
#include <imageProcess/line.h>

namespace ImageProcess {
class ClineOrganizer {
 public:
  ClineOrganizer(const Image::CphotoSet& photoSet);
  virtual ~ClineOrganizer();

  void init(const int num, const std::string prefix,
            const int level);

  std::vector<std::vector<std::vector<unsigned char> > > m_limages;
  std::vector<std::vector<ImageProcess::Cline> > m_lines;
  
 protected:
  int m_num;
  int m_level;
  std::string m_prefix;
  const Image::CphotoSet& m_photoSet;

};
};
#endif // IMAGEPROCESS_LINEORGANIZER_H
