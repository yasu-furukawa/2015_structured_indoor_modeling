#include "lineOrganizer.h"
#include <fstream>
#include <iostream>

using namespace Image;
using namespace ImageProcess;
using namespace std;

ClineOrganizer::ClineOrganizer(const Image::CphotoSet& photoSet)
  : m_photoSet(photoSet) {
}

ClineOrganizer::~ClineOrganizer() {
}

void ClineOrganizer::init(const int num, const std::string prefix,
                          const int level) {
  m_num = num;
  m_prefix = prefix;
  m_level = level;

  m_lines.resize(m_num);
  for (int image = 0; image < m_num; ++image) {
    char buffer[1024];
    sprintf(buffer, "%slines/%04d.line", m_prefix.c_str(), image);
    
    ifstream ifstr;
    ifstr.open(buffer);
    if (!ifstr.is_open()) {
      cerr << "No line file: " << buffer << endl;
      continue;
    }

    string header;
    int num;
    ifstr >> header >> num;
    m_lines[image].resize(num);
    for (int i = 0; i < num; ++i)
      ifstr >> m_lines[image][i];
      
    ifstr.close();
  }

  const float scale = 0x0001 << m_level;
  // Set limages
  m_limages.resize(m_num);
  for (int image = 0; image < m_num; ++image) {
    const int width = m_photoSet.getWidth(image, m_level);
    const int height = m_photoSet.getHeight(image, m_level);
    m_limages[image].resize(height);
    for (int y = 0; y < height; ++y) {
      m_limages[image][y].resize(width);
      for (int x = 0; x < width; ++x)
        m_limages[image][y][x] = 0;
    }

    for (int i = 0; i < (int)m_lines[image].size(); ++i) {
      const Vec3f p0 = m_lines[image][i].m_endPoints[0] / scale;
      const Vec3f p1 = m_lines[image][i].m_endPoints[1] / scale;
      Vec3f ray = p1 - p0;
      const float len = norm(ray);
      const int samplenum = max(2, (int)ceil(len));
      const float unit = len / samplenum;
      ray = ray * (unit / len);
      Vec3f pos = p0;
      for (int j = 0; j <= samplenum; ++j) {
        const int ix = (int)floor(pos[0] + 0.5f);
        const int iy = (int)floor(pos[1] + 0.5f);
        if (ix < 1 || width - 1 <= ix || iy < 1 || height - 1 <= iy)
          continue;

        m_limages[image][iy][ix] = 1;
        /*
        m_limages[image][iy][ix - 1] = 1;
        m_limages[image][iy][ix + 1] = 1;
        m_limages[image][iy - 1][ix] = 1;
        m_limages[image][iy + 1][ix] = 1;
        */
        pos += ray;
      }
    }

    /*
    ofstream ofstr;
    char buffer[1024];
    sprintf(buffer, "l%04d.pgm", image);
    ofstr.open(buffer);
    ofstr << "P2" << endl
          << width << ' ' << height << endl
          << 255 << endl;
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        if (m_limages[image][y][x])
          ofstr << 255 << ' ';
        else
          ofstr << 0 << ' ';
    ofstr.close();
    */
  }
}
