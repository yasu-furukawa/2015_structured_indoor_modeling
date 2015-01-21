#include "patch2d.h"
#include <cmath>

using namespace ImageProcess;
using namespace std;

Cpatch2d::Cpatch2d(void) {
  m_center = Vec2f();
  m_xaxis = Vec2f(1.0f, 0.0f);
  m_yaxis = Vec2f(0.0f, 1.0f);
  m_score = 2.0f;

  m_id = -1;
  m_num = -1;
}

Cpatch2d::~Cpatch2d() {
}

void Cpatch2d::lift(float& z, Vec3f& r0, Vec3f& r1) const{
  lift(m_xaxis, m_yaxis, z, r0, r1);
}

// A is multiplied from the left hand side
void Cpatch2d::lift(const Mat2f& A, float& z,
		    Vec3f& r0, Vec3f& r1) const {
  lift(A * m_xaxis, A * m_yaxis, z, r0, r1);
}

void Cpatch2d::lift(const Vec2f& xaxis, const Vec2f& yaxis,
		    float& z, Vec3f& r0, Vec3f& r1) {
  // Frobeneous norm
  const float AF2 = norm2(xaxis) + norm2(yaxis);
  const float AF4 = AF2 * AF2;
  const float det = xaxis[0] * yaxis[1] - xaxis[1] * yaxis[0];
  const float det2 = det * det;

  // b_4ac is guaranteed to be non-negative.
  const float b_4ac = max(0.0f, AF4 - 4 * det2);
  z = sqrt(max(0.0f, (AF2 - sqrt(b_4ac)) / (2 * det2)));

  r0[0] = xaxis[0] * z;  r0[1] = xaxis[1] * z;
  r0[2] = sqrt(max(0.0f, 1.0f - (r0[0] * r0[0] + r0[1] * r0[1])));

  r1[0] = yaxis[0] * z;  r1[1] = yaxis[1] * z;
  r1[2] = sqrt(max(0.0f, 1.0f - (r1[0] * r1[0] + r1[1] * r1[1])));

  // Make sure that the dot product between r0 and r1 is 0.
  //????????
  if (r0[0] * r1[0] + r0[1] * r1[1] > 0.0)
    //r0[2] = -r0[2];
    r1[2] = -r1[2];
  
}

std::istream& ImageProcess::operator>>(std::istream& istr, Cpatch2d& rhs) {
  string header;
  istr >> header;
  if (header != "PATCH2D") {
    cerr << "Header is not PATCH2D: " << header << endl;
    exit (1);
  }
  istr >> rhs.m_center >> rhs.m_xaxis >> rhs.m_yaxis >> rhs.m_score;
  return istr;
}

std::ostream& ImageProcess::operator<<(std::ostream& ostr, Cpatch2d& rhs) {
  ostr << "PATCH2D" << endl
       << rhs.m_center << endl
       << rhs.m_xaxis << ' ' << rhs.m_yaxis << ' ' << rhs.m_score << endl;
  return ostr;  
}
