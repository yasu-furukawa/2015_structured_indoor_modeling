#include "lineExtracter.h"
#include <fstream>

using namespace std;
using namespace Image;
using namespace ImageProcess;

ClineExtracter::ClineExtracter(void) {
}

ClineExtracter::~ClineExtracter() {
}

int ClineExtracter::theta2cell(const float theta) const{
  return (int)floor(theta / m_thetaUnit + 0.5f);
}

float ClineExtracter::cell2theta(const int cell) const{
  return cell * m_thetaUnit;
}

int ClineExtracter::rho2cell(const float rho) const{
  return (int)floor(rho / m_rhoUnit + 0.5f) + m_rhoOffset;
}

float ClineExtracter::cell2rho(const int cell) const{
  return (cell - m_rhoOffset) * m_rhoUnit;
}

// Input is edge detected image
void ClineExtracter::extract(Image::Cimage& eImage, const int level,
                             const float minLen) {
  //----------------------------------------------------------------------
  // Initialization
  m_level = level;
  m_width = eImage.getWidth(m_level);
  m_height = eImage.getHeight(m_level);
  m_center = Vec2f(m_width / 2.0f, m_height / 2.0f);
  m_minLen = minLen;

  //----------------------------------------------------------------------
  // Thresholds
  //m_gap = 2;
  m_gap = 3;
  
  // One degree each
  m_rhoUnit = 1.0f;
  m_rhoHeight = (int)floor(sqrt(m_width * m_width + m_height * m_height)
                           / m_rhoUnit + 0.5f);
  m_rhoOffset = m_rhoHeight / 2;

  // Make this close to m_rhoHeight;
  m_thetaWidth = max(360, m_rhoHeight);
  m_thetaUnit = M_PI / m_thetaWidth;
  
  //m_thetaUnit = 0.3f * M_PI / 180.0f;
  //m_thetaWidth = (int)ceil(M_PI / m_thetaUnit);
  
  m_votes.resize(m_rhoHeight);
  m_tmp.resize(m_rhoHeight);
  m_removed.resize(m_rhoHeight);
  for (int y = 0; y < m_rhoHeight; ++y) {
    m_votes[y].resize(m_thetaWidth);
    m_tmp[y].resize(m_thetaWidth);
    m_removed[y].resize(m_thetaWidth);
    for (int x = 0; x < m_thetaWidth; ++x)
      m_votes[y][x] = 0.0f;
  }

  //cerr << "gauss..." << flush;
  //boxFilter(eImage);
  //gaussFilter(eImage);
  //squareFilter(eImage);

  cerr << "voting..." << flush;  
  //----------------------------------------------------------------------
  // Vote
  //----------------------------------------------------------------------
  for (int y = 0; y < m_height; ++y) {
    for (int x = 0; x < m_width; ++x) {
      if (eImage.getEdge(x, y, m_level) == 0)
        continue;
      
      // cast votes
      for (int t = 0; t < m_thetaWidth; ++t) {
        const float theta = cell2theta(t);
        const float rho =
          cos(theta) * (x - m_center[0]) + sin(theta) * (y - m_center[1]);
        //const int r = rho2cell(rho);
        const float rf = rho / m_rhoUnit + m_rhoOffset;
        const int r0 = (int)floor(rf);
        const int r1 = r0 + 1;
        if (0 <= r0 && r1 < m_rhoHeight) {
          m_votes[r0][t] += r1 - rf;
          m_votes[r1][t] += rf - r0;
        }
      }
    }
  }

  /*
  vector<unsigned char> image;
  image.resize(m_thetaWidth * m_rhoHeight * 3);
  int count = 0;
  for (int y = 0; y < m_rhoHeight; ++y)
    for (int x = 0; x < m_thetaWidth; ++x) {
      image[count++] = 
        (unsigned char)min(255, max(0, (int)floor(m_votes[y][x] * 3)));
      image[count++] =
        (unsigned char)min(255, max(0, (int)floor(m_votes[y][x] * 3)));
      image[count++] =
        (unsigned char)min(255, max(0, (int)floor(m_votes[y][x] * 3)));
    }
  
  Image::Cimage::writeJpegImage("hough.jpg", image, m_thetaWidth, m_rhoHeight);
  */

  cerr << "BDF..." << flush;
  BDF();

  cerr << "Smooth..." << flush;    
  smooth();
  
  cerr << "NMS..." << flush;
  nonMaxSuppress();

  cerr << "Extraction..." << flush;
  extractLines(eImage);
  cerr << "Done" << endl;
}

void ClineExtracter::gaussFilter(Image::Cimage& eImage) {
  vector<float> fimage;
  fimage.resize(m_width * m_height);
  int count = 0;
  for (int y = 0; y < m_height; ++y)
    for (int x = 0; x < m_width; ++x) {
      if (eImage.getEdge(x, y, m_level))
        fimage[count++] = 1;
      else
        fimage[count++] = 0;
    }

  const int margin = 6;
  const int size = 2 * margin + 1;
  const float sigma = margin / 2.0f;
  const float sigma2 = 2.0f * sigma * sigma;
  vector<float> filter;
  filter.resize(size);
  for (int i = -margin; i <= margin; ++i)
    filter[i + margin] = exp(- i * i / sigma2);

  vector<float> vftmp;
  vftmp.resize((int)fimage.size());
  // hozirontal
  count = 0;
  for (int y = 0; y < m_height; ++y) {
    for (int x = 0; x < m_width; ++x) {
      vftmp[count] = 0.0f;
      float denom = 0.0f;
      for (int i = -margin; i <= margin; ++i) {
        const int xtmp = x + i;
        if (xtmp < 0 || m_width <= xtmp)
          continue;
        vftmp[count] += filter[i + margin] * fimage[y * m_width + xtmp];
        denom += filter[i + margin];
      }
      vftmp[count] /= denom;
      count++;
    }
  }

  vftmp.swap(fimage);
  // vertical
  count = 0;
  for (int y = 0; y < m_height; ++y) {
    for (int x = 0; x < m_width; ++x) {
      vftmp[count] = 0.0f;
      float denom = 0.0f;
      for (int j = -margin; j <= margin; ++j) {
        const int ytmp = y + j;
        if (ytmp < 0 || m_height <= ytmp)
          continue;
        vftmp[count] += filter[j + margin] * fimage[ytmp * m_width + x];
        denom += filter[j + margin];
      }
      vftmp[count] /= denom;
      count++;
    }
  }

  //----------------------------------------------------------------------
  const float threshold = 0.2;;//0.5
  vector<unsigned char>& edgeImage = eImage.getEdge(m_level);
  for (int i = 0; i < (int)vftmp.size(); ++i) {
    if (edgeImage[i] && vftmp[i] < threshold)
      edgeImage[i] = 1;
    else
      edgeImage[i] = 0;
  }

  /*
  ofstream ofstr;
  ofstr.open("test.pgm");
  ofstr << "P2" << endl
        << m_width << ' ' << m_height << endl
        << 255 << endl;
  for (int i = 0; i < (int)edgeImage.size(); ++i) {
    ofstr << 255 * edgeImage[i] << ' ';
  }
  ofstr.close();
  */
}

void ClineExtracter::boxFilter(Image::Cimage& eImage) {
  vector<unsigned char>& edgeImage = eImage.getEdge(m_level);
  vector<int> image0, image1;

  image0.resize((int)edgeImage.size());
  image1.resize((int)edgeImage.size());
  for (int i = 0; i < (int)image0.size(); ++i) {
    image0[i] = (int)edgeImage[i];
    image1[i] = 0;
  }
  
  const int margin = 6;
  //hozizontal
  for (int y = 0; y < m_height; ++y) {
    for (int x = margin; x < m_width - margin; ++x) {
      const int index = y * m_width + x;
      for (int i = -margin; i <= margin; ++i) {
        const int xtmp = x + i;
        if (image0[y * m_width + xtmp])
          image1[index]++;
      }
    }
  }
  
  //vetical
  for (int i = 0; i < (int)image0.size(); ++i)
    image0[i] = 0;

  for (int y = margin; y < m_height - margin; ++y) {
    for (int x = margin; x < m_width - margin; ++x) {
      const int index = y * m_width + x;
      for (int j = -margin; j <= margin; ++j) {
        const int ytmp = y + j;
        image0[index] += image1[ytmp * m_width + x];
      }
    }
  }

  // Threshold
  const int threshold = (2 * margin + 1) * 2;
  for (int y = margin; y < m_height - margin; ++y)
    for (int x = margin; x < m_width - margin; ++x) {
      const int index = y * m_width + x;
      if (edgeImage[index] && image0[index] < threshold)
        image0[index] = 1;
      else
        image0[index] = 0;
    }
  for (int i = 0; i < (int)image0.size(); ++i)
    edgeImage[i] = (unsigned char)image0[i];



  ofstream ofstr;
  ofstr.open("test.pgm");
  ofstr << "P2" << endl
        << m_width << ' ' << m_height << endl
        << 255 << endl;
  for (int i = 0; i < (int)edgeImage.size(); ++i) {
    ofstr << 255 * edgeImage[i] << ' ';
  }
  ofstr.close();
}

void ClineExtracter::write(const std::string& file) const{
  ofstream ofstr;
  ofstr.open(file.c_str());
  ofstr << "Lines " << (int)m_lines.size() << endl;
  for (int i = 0; i < (int)m_lines.size(); ++i)
    ofstr << m_lines[i] << endl;
  ofstr.close();
  /*
  for (int i = 0; i < (int)m_lines.size(); ++i)
    cout << m_lines[i].m_endPoints[0][0] << ' '
         << m_lines[i].m_endPoints[0][1] << ' '
         << m_lines[i].m_endPoints[1][0] << ' '
         << m_lines[i].m_endPoints[1][1] << endl;
  */
}

void ClineExtracter::BDF(void) {
  //----------------------------------------------------------------------
  // Butterfly filter
  for (int y = 0; y < m_rhoHeight; ++y)
    for (int x = 0; x < m_thetaWidth; ++x)
      m_tmp[y][x] = 0.0f;

  for (int y = 3; y < m_rhoHeight - 3; ++y) {
    for (int x = 0; x < m_thetaWidth; ++x) {
      float background = 0.0f;
      background += (m_votes[y-3][x] + m_votes[y-2][x] +
                     m_votes[y+2][x] + m_votes[y+3][x]) / 4.0f;

      const int nx = (x == m_thetaWidth - 1) ? 0 : x + 1;
      const int px = (x == 0) ? m_thetaWidth - 1 : x - 1;
      
      m_tmp[y][x] =
        (m_votes[y][x] +
         m_votes[y][nx] + m_votes[y+1][nx] + m_votes[y-1][nx] +
         m_votes[y][px] + m_votes[y+1][px] + m_votes[y-1][px]) / 7.0f -
        background;
    }
  }
  m_votes.swap(m_tmp);
}

void ClineExtracter::smooth(void) {
  //----------------------------------------------------------------------
  // Blur and non-maximum supression
  //----------------------------------------------------------------------  
  const int vmargin = 3;
  const int vsize = 2 * vmargin + 1;
  const float vsigma = vmargin / 2.0f;
  const float vsigma2 = 2.0f * vsigma * vsigma;

  const int hmargin = max(2, (int)ceil(0.5 * M_PI / 180.0f / m_thetaUnit));
  const int hsize = 2 * hmargin + 1;
  const float hsigma = hmargin / 2.0f;
  const float hsigma2 = 2.0f * hsigma * hsigma;

  vector<float> vfilter;
  vfilter.resize(vsize);
  for (int i = 0; i < vsize; ++i)
    vfilter[i] = exp(- (i - vmargin) * (i - vmargin) / vsigma2);
  vector<float> hfilter;
  hfilter.resize(hsize);
  for (int i = 0; i < hsize; ++i)
    hfilter[i] = exp(- (i - hmargin) * (i - hmargin) / hsigma2);
  
  for (int y = 0; y < m_rhoHeight; ++y)
    for (int x = 0; x < m_thetaWidth; ++x)
      m_tmp[y][x] = 0.0f;

  // vertical
  for (int y = vmargin; y < m_rhoHeight - vmargin; ++y)
    for (int x = 0; x < m_thetaWidth; ++x) {
      m_tmp[y][x] = 0.0f;
      float denom = 0.0f;
      for (int j = -vmargin; j <= vmargin; ++j) {
        const int ytmp = y + j;
        m_tmp[y][x] += vfilter[j + vmargin] * m_votes[ytmp][x];
        denom += vfilter[j + vmargin];
      }
      m_tmp[y][x] /= denom;
    }
  m_votes.swap(m_tmp);
  // horizontal
  for (int y = vmargin; y < m_rhoHeight - vmargin; ++y)
    for (int x = 0; x < m_thetaWidth; ++x) {
      m_tmp[y][x] = 0.0f;
      float denom = 0.0f;
      for (int i = -hmargin; i <= hmargin; ++i) {
        int xtmp = x + i;
        if (xtmp < 0)
          xtmp += m_thetaWidth;
        else if (m_thetaWidth <= xtmp)
          xtmp -= m_thetaWidth;
        m_tmp[y][x] += hfilter[i + hmargin] * m_votes[y][xtmp];
        denom += hfilter[i + hmargin];
      }
      m_tmp[y][x] /= denom;
    }
  m_votes.swap(m_tmp);
}

void ClineExtracter::nonMaxSuppress(void) {             
  // Conservative threshold just to remove obviously bad ones
  const float threshold = m_minLen / 100.0f;
  
  for (int y = 0; y < m_rhoHeight; ++y)
    for (int x = 0; x < m_thetaWidth; ++x) {
      if (m_votes[y][x] < threshold)
        m_votes[y][x] = 0.0f;
    }
  
  for (int y = 0; y < m_rhoHeight; ++y)
    for (int x = 0; x < m_thetaWidth; ++x)
      m_tmp[y][x] = 0.0f;
  
  for (int y = 1; y < m_rhoHeight - 1; ++y) {
    for (int x = 0; x < m_thetaWidth; ++x) {
      if (m_votes[y][x] == 0.0f)
        continue;
      const int nx = (x == m_thetaWidth - 1) ? 0 : x + 1;
      const int px = (x == 0) ? m_thetaWidth - 1 : x - 1;

      if (m_votes[y][x] < m_votes[y][px]   || m_votes[y][x] < m_votes[y][nx] ||
          m_votes[y][x] < m_votes[y+1][px] || m_votes[y][x] < m_votes[y+1][x] ||
          m_votes[y][x] < m_votes[y+1][nx] || m_votes[y][x] < m_votes[y-1][px] ||
          m_votes[y][x] < m_votes[y-1][x]  || m_votes[y][x] < m_votes[y-1][nx])
        continue;
      
      m_tmp[y][x] = m_votes[y][x];
    }
  }
  m_votes.swap(m_tmp);
}

void ClineExtracter::extractLines(Image::Cimage& eImage) {
  vector<Vec3f> candidates;
  for (int y = 0; y < m_rhoHeight; ++y) {
    for (int x = 0; x < m_thetaWidth; ++x) {
      if (m_votes[y][x] == 0.0f)
        continue;
      
      const float theta = cell2theta(x);
      const float rho = cell2rho(y);
      
      candidates.push_back(Vec3f(-m_votes[y][x], theta, rho));
    }
  }

  // Sort candidates
  sort(candidates.begin(), candidates.end(), Svec3cmp<float>());

  vector<Vec3f>::iterator begin = candidates.begin();
  while (begin != candidates.end()) {
    const float theta = (*begin)[1];
    const float rho = (*begin)[2];
    Cline line;
    line.m_abc[0] = cos(theta);
    line.m_abc[1] = sin(theta);
    line.m_abc[2] = - rho - cos(theta) * m_center[0] - sin(theta) * m_center[1];
    
    // Extract relative pixels, hacky part
    extractLine(eImage, line);
    begin++;
  }
}

void ClineExtracter::removeEdge(Image::Cimage& eImage, const Cline& line) {
  Vec3f ray = line.m_endPoints[1] - line.m_endPoints[0];
  const int sampleNum = (int)ceil(norm(ray));
  ray /= sampleNum;

  vector<unsigned char>& edgeImage = eImage.getEdge(m_level);
  
  Vec3f pos = line.m_endPoints[0];

  const int margin = 1;
  for (int i = 0; i <= sampleNum; ++i) {
    const int x = (int)floor(pos[0] + 0.5f);
    const int y = (int)floor(pos[1] + 0.5f);
    
    for (int j = -margin; j <= margin; ++j) {
      const int ytmp = y + j;
      if (ytmp < 0 || m_height <= ytmp)
        continue;
      for (int i = -margin; i <= margin; ++i) {
        const int xtmp = x + i;
        if (xtmp < 0 || m_width <= xtmp)
          continue;
        edgeImage[ytmp * m_width + xtmp] = (unsigned char)0;
        if (m_removed[ytmp][xtmp].empty())
          m_removed[ytmp][xtmp].push_back(Vec3f(line.m_abc[0],
                                                line.m_abc[1],
                                                0.0f));
      }
    }
    pos += ray;
    /*
    const int x0 = (int)floor(pos[0]);
    const int x1 = x0 + 1;
    const int y0 = (int)floor(pos[1]);
    const int y1 = y0 + 1;

    pos += ray;
    if (x0 < 0 || m_width <= x1 || y0 < 0 || m_height <= y1)
      continue;

    edgeImage[y0 * m_width + x0] = (unsigned char)0;
    edgeImage[y0 * m_width + x1] = (unsigned char)0;
    edgeImage[y1 * m_width + x0] = (unsigned char)0;
    edgeImage[y1 * m_width + x1] = (unsigned char)0;
    */
  }
}

void ClineExtracter::fillIn(const int scanwhich, std::vector<int>& inout) const {
  const int sizes[2] = {m_width, m_height};

  vector<int> lefts, rights;
  lefts.resize((int)inout.size());
  rights.resize((int)inout.size());
  for (int i = 0; i < (int)inout.size(); ++i)
    lefts[i] = rights[i] = 0;

  const int threshold = m_gap * 2;
  int seq = 0;
  for (int i = 0; i < (int)inout.size(); ++i) {
    if (inout[i] == 0)
      seq++;
    else
      seq = 0;
    if (threshold <= seq)
      lefts[i] = 1;
  }
  seq = 0;
  for (int i = (int)inout.size() - 1; 0 <= i; --i) {
    if (inout[i] == 0)
      seq++;
    else
      seq = 0;
    if (threshold <= seq)
      rights[i] = 1;
  }
  
  // Fill in gaps
  for (int i = m_gap; i < sizes[scanwhich] - m_gap; ++i) {
    if (inout[i] == 0)
      continue;

    int left = 0;    int right = 0;
    for (int j = -m_gap; j <= -1; ++j)
      if (lefts[i + j]) {
        left = 1;        break;
      }
    for (int j = 1; j <= m_gap; ++j)
      if (rights[i + j] == 0) {
        right = 1;        break;
      }

    if (left && right)
      inout[i] = 0;
  }
}
  
void ClineExtracter::getInOut(Image::Cimage& eImage, const Cline& line,
                              std::vector<int>& inout) const{
  const int scanwhich = (fabs(line.m_abc[0]) < fabs(line.m_abc[1])) ? 0 : 1;
  const int sizes[2] = {m_width, m_height};
  
  // Scan
  inout.resize(sizes[scanwhich]);
  for (int pivot = 0; pivot < sizes[scanwhich]; ++pivot) {
    inout[pivot] = 1;
    
    const float pos =
      - (pivot * line.m_abc[scanwhich] + line.m_abc[2]) /
      line.m_abc[1 - scanwhich];
    const int ipos = (int)floor(pos + 0.5f);
    const int ppos = ipos - 2;
    const int npos = ipos + 2;
    if (ppos < 0 || sizes[1 - scanwhich] <= npos)
      continue;

    if (scanwhich == 0) {
      for (int i = ppos; i <= npos; ++i)
        if (eImage.getEdge(pivot, i, m_level) ||
            (!m_removed[i][pivot].empty() &&
             fabs(m_removed[i][pivot][0] * line.m_abc) < 0.5)) {
          inout[pivot] = 0;          break;
        }
    }
    else {
      for (int i = ppos; i <= npos; ++i)
        if (eImage.getEdge(i, pivot, m_level) ||
            (!m_removed[pivot][i].empty() &&
             fabs(m_removed[pivot][i][0] * line.m_abc) < 0.5)) {
          inout[pivot] = 0;          break;
        }
    }
  }

  fillIn(scanwhich, inout);
  
  inout[0] = inout[sizes[scanwhich] - 1] = 1;
}

void ClineExtracter::extractLine(Image::Cimage& eImage, Cline& line) {
  /*
  {
    Cline newline;
    newline.m_abc = line.m_abc;
    if (fabs(newline.m_abc[0]) < fabs(newline.m_abc[1])) {
      int x0 = 0;      int x1 = m_width - 1;
      float y0 = - (newline.m_abc[0] * x0 + newline.m_abc[2]) / newline.m_abc[1];
      float y1 = - (newline.m_abc[0] * x1 + newline.m_abc[2]) / newline.m_abc[1];
      newline.m_endPoints[0] = Vec3f(x0, y0, 1.0f);
      newline.m_endPoints[1] = Vec3f(x1, y1, 1.0f);
    }
    else {
      int y0 = 0;      int y1 = m_height - 1;
      float x0 = - (newline.m_abc[1] * y0 + newline.m_abc[2]) / newline.m_abc[0];
      float x1 = - (newline.m_abc[1] * y1 + newline.m_abc[2]) / newline.m_abc[0];
      newline.m_endPoints[0] = Vec3f(x0, y0, 1.0f);
      newline.m_endPoints[1] = Vec3f(x1, y1, 1.0f);
    }
    m_lines.push_back(newline);
    return;
  }
  */
  
  // Which axis to scan
  const int scanwhich = (fabs(line.m_abc[0]) < fabs(line.m_abc[1])) ? 0 : 1;
  const int sizes[2] = {m_width, m_height};
  vector<int> inout;
  getInOut(eImage, line, inout);
  
  // ??? more robust
  /*
  vector<int> pinout, ninout;
  Cline pline, nline;
  pline.m_abc[0] = line.m_abc[0];  pline.m_abc[1] = line.m_abc[1];
  pline.m_abc[2] = line.m_abc[2] - 3.0f;
  nline.m_abc[0] = line.m_abc[0];  nline.m_abc[1] = line.m_abc[1];
  nline.m_abc[2] = line.m_abc[2] + 3.0f;
  getInOut(eImage, pline, pinout);
  getInOut(eImage, nline, ninout);
  for (int i = 0; i < (int)inout.size(); ++i) {
    if (pinout[i] == 0 && ninout[i] == 0)
      inout[i] = 1;
  }
  */
    
  // Determine end points
  const float scale = max(fabs(line.m_abc[0]), fabs(line.m_abc[1]));
  int index = 0;
  int pindex = -1;
  for (index = 0; index < sizes[scanwhich]; ++index) {
    if (inout[index] == 0) {
      if (pindex == -1)
        pindex = index;
    }
    else {
      // End of a line
      const int score = index - pindex;
      if (pindex != -1 && m_minLen * scale <= score) {
          Cline newline;
          newline.m_abc = line.m_abc;

          const float pos0 =
            - (pindex * line.m_abc[scanwhich] + line.m_abc[2]) /
            line.m_abc[1 - scanwhich];
          const float pos1 =
            - ((index - 1) * line.m_abc[scanwhich] + line.m_abc[2]) /
            line.m_abc[1 - scanwhich];
          
          if (scanwhich == 0) {
            newline.m_endPoints[0] = Vec3f(pindex, pos0, 1.0f);
            newline.m_endPoints[1] = Vec3f(index - 1, pos1, 1.0f);
          }
          else {
            newline.m_endPoints[0] = Vec3f(pos0, pindex, 1.0f);
            newline.m_endPoints[1] = Vec3f(pos1, index - 1, 1.0f);
          }
          
          m_lines.push_back(newline);
          removeEdge(eImage, newline);
      }
      pindex = -1;
    }
  }
}
  
