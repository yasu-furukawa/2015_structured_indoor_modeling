#include <string>
#include "line3.h"
#include <numeric/mat4.h>
#include <numeric/mylapack.h>

extern "C" {
#include <clapack/f2c.h>
#include <clapack/clapack.h>
};

using namespace std;
using namespace ImageProcess;

Cline3::Cline3(void) {
};

Cline3::~Cline3() {
};

Vec4f Cline3::unproject(const Vec3f& lhs, const Cline& rhs,
                        const Image::Cphoto& photo0,
                        const Image::Cphoto& photo1) {
  const vector<Vec4f>& proj0 = photo0.m_projection[0];
  const vector<Vec4f>& proj1 = photo1.m_projection[0];

  vector<float> A;  A.resize(16);
  vector<float> b;  b.resize(4);
  
  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x)
      A[x * 4 + y] = proj0[y][x];
    A[12 + y] = -lhs[y];
    b[y] = -proj0[y][3];
  }

  Vec4f abcp;
  for (int x = 0; x < 4; ++x)
    abcp[x] = rhs.m_abc[0] * proj1[0][x] + rhs.m_abc[1] * proj1[1][x] +
      rhs.m_abc[2] * proj1[2][x];
  
  for (int x = 0; x < 3; ++x)
    A[4 * x + 3] = abcp[x];
  b[3] = -abcp[3];

  // A (x y z lambda) = b
  Cmylapack::lls(A, b, 4, 4);
  return Vec4f(b[0], b[1], b[2], 1.0f);
}

void Cline3::setVimages(void) {
  m_vimages.clear();
  for (int i = 0; i < (int)m_ranges.size(); ++i)
    if (!m_ranges[i].empty())
      m_vimages.push_back(i);
}

void Cline3::setOranges(void) {
  m_oranges.clear();
  m_oranges.resize((int)m_ranges.size());
  for (int image = 0; image < (int)m_oranges.size(); ++image) {
    for (int i = 0; i < (int)m_ranges[image].size(); ++i) {
      for (int j = 0; j < (int)m_range.size(); ++j) {
        Vec2f intersect;
        if (overlap(Vec2f(m_ranges[image][i][0], m_ranges[image][i][1]),
                    m_range[j], intersect)) {
          m_oranges[image].push_back(Vec3f(intersect,
                                           m_ranges[image][i][2]));
        }
      }
    }
  }
}

float Cline3::getPos(const Vec3f& iorg, const Vec3f& idir,
                     const Vec3f& icoord) {
  const float f0 = idir[0] - icoord[0] * idir[2];
  const float f1 = idir[1] - icoord[1] * idir[2];

  if (f0 == 0.0f || f1 == 0.0f)
    return NAN;
  
  const float a0 = (icoord[0] * iorg[2] - iorg[0]) / f0;
  const float a1 = (icoord[1] * iorg[2] - iorg[1]) / f1;
  
  return (a0 + a1) / 2.0f;
}

Vec4f Cline3::snap(const Vec4f& coord) const{
  return m_origin + ((coord - m_origin) * m_direction) * m_direction;
}

int Cline3::isClose(const Cline3& lhs, const Cline3& rhs,
                    const float threshold) {
  const Vec4f lproj = lhs.snap(rhs.m_origin);
  const Vec4f rproj = rhs.snap(lhs.m_origin);

  if (norm(lproj - rhs.m_origin) < threshold &&
      norm(rproj - lhs.m_origin) < threshold)
    return 1;
  else
    return 0;
}

std::istream& operator>>(std::istream& istr,
                         ImageProcess::Cline3& line) {
  string header;
  istr >> header >> line.m_origin >> line.m_direction;
  int rnum;
  istr >> rnum;
  line.m_range.resize(rnum);
  for (int i = 0; i < rnum; ++i)
    istr >> line.m_range[i];
  int num;
  istr >> num;
  line.m_ranges.resize(num);
  for (int i = 0; i < num; ++i) {
    int itmp;    istr >> itmp;
    line.m_ranges[i].resize(itmp);
    for (int j = 0; j < itmp; ++j)
      istr >> line.m_ranges[i][j];
  }
  istr >> line.m_color0 >> line.m_color1
       >> line.m_score;
  return istr;
}

std::ostream& operator<<(std::ostream& ostr,
                         const ImageProcess::Cline3& line) {
  ostr << "LINE3" << endl
       << line.m_origin << "  " << line.m_direction << endl;
  ostr << (int)line.m_range.size() << endl;
  for (int i = 0; i < (int)line.m_range.size(); ++i)
    ostr << line.m_range[i] << endl;
  ostr << (int)line.m_ranges.size() << endl;
  for (int i = 0; i < (int)line.m_ranges.size(); ++i) {
    ostr << (int)line.m_ranges[i].size() << endl;
    for (int j = 0; j < (int)line.m_ranges[i].size(); ++j) {
      ostr << line.m_ranges[i][j] << endl;
    }
  }
  ostr << endl;
  ostr << line.m_color0 << ' ' << line.m_color1 << endl
       << line.m_score << endl;
  return ostr;
}

void Cline3::init(const std::vector<Vec4f>& coords,
                  const std::vector<int>& ids) {
  if (coords.empty()) {
    cerr << "impossible in init" << endl;    exit (1);
  }
  
  m_origin = Vec4f();
  for (int i = 0; i < (int)coords.size(); ++i)
    m_origin += coords[i];
  m_origin /= (int)coords.size();

  vector<vector<float> > A;
  A.resize((int)coords.size());
  for (int i = 0; i < (int)coords.size(); ++i) {
    A[i].resize(3);
    for (int j = 0; j < 3; ++j)
      A[i][j] = coords[i][j] - m_origin[j];
  }

  vector<vector<float> > U, VT;
  vector<float> S;

  Cmylapack::svd(A, U, VT, S);

  m_direction[0] = VT[0][0];
  m_direction[1] = VT[0][1];
  m_direction[2] = VT[0][2];
  m_direction[3] = 0.0f;
  unitize(m_direction);

  m_vimages = ids;

  m_range.resize(1);
  m_range[0][0] = INT_MAX/2;
  m_range[0][1] = -INT_MAX/2;
  for (int i = 0; i < (int)coords.size(); ++i) {
    const float ftmp = (coords[i] - m_origin) * m_direction;
    m_range[0][0] = min(m_range[0][0], ftmp);
    m_range[0][1] = max(m_range[0][1], ftmp);
  }
}

//----------------------------------------------------------------------
// For octree
int Cline3::checkBox(const Vec3f& mmin, const Vec3f& mmax) const {
  cerr << "not implemented Cline3::checkBox" << endl;
  exit (1);

  return 1;
}

float Cline3::error(const Vec4f& coord) const {
  cerr << "not implemented Cline3::error" << endl;
  exit (1);
  
  return 0.0f;
}


/*
    Mat4f A;
  A[0] = right0 - m_origin;  A[1] = right1 - m_origin;
  A[2] = left0 - m_origin;   A[3] = left1 - m_origin;
  
  // Set direction
  {
    char jobu = 'N';    char jobvt = 'S';
    integer M = 4;
    integer N = 3;
    
    float C[M * N];
    int count = 0;
    for (int x = 0; x < N; ++x)
      for (int y = 0; y < M; ++y)
        C[count++] = A[y][x];
    integer lda = M;
    float S[M];
    float U[1];
    integer LDU = 1;
    float VT[N * N];
    integer LDVT = N;
    integer lwork = 5 * max(N, M);
    float work[lwork];
    integer info;
    
    sgesvd_(&jobu, &jobvt, &M, &N, C, &lda, S, U, &LDU,
            VT, &LDVT, work, &lwork, &info);

    cout << info << ' ' << work[0] << ' ' << work[1] << endl;
    
    m_direction = Vec4f(VT[0], VT[3], VT[6], 0.0f);
    unitize(m_direction);
  }

*/
