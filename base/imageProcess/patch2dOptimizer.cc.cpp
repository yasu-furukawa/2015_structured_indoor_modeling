#include <vector>
#include <iostream>
#include "patch2dOptimizer.h"
#include "../image/photo.h"

extern "C" {
#include <acc/wnconj.h>
#include <acc2/wnconj.h>  
#include <acc3/wnconj.h>  
#include <acc4/wnconj.h>
    
#include <acc/wnconjd.h>
#include <acc2/wnconjd.h>  
#include <acc3/wnconjd.h>  
#include <acc4/wnconjd.h>
};

#include <minpack/cminpak.h>
#include <minpack/dpmpar.h>

using namespace std;
using namespace Image;
using namespace ImageProcess;

Cpatch2dOptimizer* Cpatch2dOptimizer::m_one = NULL;

Cpatch2dOptimizer::Cpatch2dOptimizer(void) {
  m_CPU = 4;

  m_ppatches0T.resize(m_CPU);
  m_ppatches1T.resize(m_CPU);
  m_patches1OrgT.resize(m_CPU);

  m_pimages0T.resize(m_CPU);
  m_pimages1T.resize(m_CPU);
  
  m_halfsizesT.resize(m_CPU);
  m_levelsT.resize(m_CPU);

  m_ssdsT.resize(m_CPU);

  m_one = this;

  // ????? critical parameter
  m_maxMoves[0] = 2.0f;  m_maxMoves[1] = 2.0f;
  m_maxMoves[2] = 1.0f;  m_maxMoves[3] = 1.0f;
  m_maxMoves[4] = 1.0f;  m_maxMoves[5] = 1.0f;

  // Initialize stats
  m_stats.resize(5);
  for (int i = 0; i < 5; ++i)
    m_stats[i] = 0;

  for (int id = 0; id < m_CPU; ++id)
    m_ssdsT[id] = 0;
}

Cpatch2dOptimizer::~Cpatch2dOptimizer() {
}

//----------------------------------------------------------------------
// 6 param optimization
// optimize, patch2d's location and shapes
//----------------------------------------------------------------------
double Cpatch2dOptimizer::optimize(const Cpatch2d& lhs, Cpatch2d& rhs,
				   const Image::Cimage& lhsimage,
				   const Image::Cimage& rhsimage,
				   const int halfsize, const int level,
				   const int id) {
  if (isInvalidAxes(lhs.m_xaxis, lhs.m_yaxis) ||
      isInvalidAxes(rhs.m_xaxis, rhs.m_yaxis)) {
    cerr << "Invalid axes at inputs." << endl;
    return 2.0f;
  }
  
  m_ppatches0T[id] = &lhs;      m_ppatches1T[id] = &rhs;
  m_pimages0T[id] = &lhsimage;  m_pimages1T[id] = &rhsimage;
  m_halfsizesT[id] = halfsize;  m_levelsT[id] = level;

  m_patches1OrgT[id] = rhs;

  int code;  double pval_min;  double vect[6];  int len = 6;
  const int max_iterations = 20;
  for (int i = 0; i < 6; ++i)
    vect[i] = 0.0;

  // Position, xaxis, yaxis
  double initial_coord_x0s[6];
  initial_coord_x0s[0] = 1.0f;  initial_coord_x0s[1] = 1.0f;
  initial_coord_x0s[2] = 0.5f;  initial_coord_x0s[3] = 0.5f;
  initial_coord_x0s[4] = 0.5f;  initial_coord_x0s[5] = 0.5f;
  
  if (id == 0)
    wn_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			     len, eval0, max_iterations);
  else if (id == 1)
    wn2_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, eval1, max_iterations);
  else if (id == 2)
    wn3_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, eval2, max_iterations);
  else if (id == 3)
    wn4_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, eval3, max_iterations);
  else {
    cerr << "No way in refinePatch" << endl;
    exit (1);
  }

  double score;
  if (m_ssdsT[id] == 0)
    score = pval_min;
  else {
    m_ssdsT[id] = 0;
    score = eval(vect, id);
    m_ssdsT[id] = 1;
  }    
  
  if (score != 2.0f && (code == WN_SUCCESS || code == WN_SUBOPTIMAL)) {
    if (isTooMuchMove(vect, 1, id) == 0) {
      encode(vect, id);
      m_stats[0]++;
      return score;
    }
    else {
      m_stats[1]++;
      *m_ppatches1T[id] = m_patches1OrgT[id];
      return 2.0f;
    }
  }
  else {
    m_stats[2]++;
    *m_ppatches1T[id] = m_patches1OrgT[id];
    return 2.0f;
  }
}

double Cpatch2dOptimizer::eval0(double* xs) {
  return m_one->eval(xs, 0);
}

double Cpatch2dOptimizer::eval1(double* xs) {
  return m_one->eval(xs, 1);
}

double Cpatch2dOptimizer::eval2(double* xs) {
  return m_one->eval(xs, 2);
}

double Cpatch2dOptimizer::eval3(double* xs) {
  return m_one->eval(xs, 3);
}

void Cpatch2dOptimizer::encode(const double* const vect, const int id) {
  const int level = m_levelsT[id];
  const float scale = (float)(0x0001 << level);    
  
  m_ppatches1T[id]->m_center[0] = m_patches1OrgT[id].m_center[0]
    + vect[0] * scale;
  m_ppatches1T[id]->m_center[1] = m_patches1OrgT[id].m_center[1]
    + vect[1] * scale;
  m_ppatches1T[id]->m_xaxis[0] = m_patches1OrgT[id].m_xaxis[0] + vect[2];
  m_ppatches1T[id]->m_xaxis[1] = m_patches1OrgT[id].m_xaxis[1] + vect[3];
  m_ppatches1T[id]->m_yaxis[0] = m_patches1OrgT[id].m_yaxis[0] + vect[4];
  m_ppatches1T[id]->m_yaxis[1] = m_patches1OrgT[id].m_yaxis[1] + vect[5];
}

double Cpatch2dOptimizer::eval(const double* const vect, const int id) {
  if (isTooMuchMove(vect, 0, id))
    return 2.0f;
  
  encode(vect, id);

  if (isInvalidAxes(m_ppatches1T[id]->m_xaxis, m_ppatches1T[id]->m_yaxis))
    return 2.0f;

  const int level = m_levelsT[id];
  const int halfsize = m_halfsizesT[id];

  const float scale = (float)(0x0001 << level);  
  const Vec2f center0 = m_ppatches0T[id]->m_center / scale;
  const Vec2f center1 = m_ppatches1T[id]->m_center / scale;
  const Vec2f& xaxis0 = m_ppatches0T[id]->m_xaxis;
  const Vec2f& yaxis0 = m_ppatches0T[id]->m_yaxis;
  const Vec2f& xaxis1 = m_ppatches1T[id]->m_xaxis;
  const Vec2f& yaxis1 = m_ppatches1T[id]->m_yaxis;

  if (isOutside(center0, xaxis0, yaxis0, halfsize,
		m_pimages0T[id]->getWidth(level), m_pimages0T[id]->getHeight(level)) ||
      isOutside(center1, xaxis1, yaxis1, halfsize,
		m_pimages1T[id]->getWidth(level), m_pimages1T[id]->getHeight(level)))
    return 2.0;

  const int size = (2 * halfsize + 1) * (2 * halfsize + 1);
  vector<Vec3f> lhscolors, rhscolors;
  lhscolors.resize(size);       rhscolors.resize(size);
  int count = -1;
  for (int y = -halfsize; y <= halfsize; ++y) {
    Vec2f lpos = center0 + y * yaxis0 - halfsize * xaxis0;
    Vec2f rpos = center1 + y * yaxis1 - halfsize * xaxis1;
    for (int x = -halfsize; x <= halfsize; ++x) {
      count++;

      lhscolors[count] = m_pimages0T[id]->getColor(lpos[0], lpos[1], level);
      rhscolors[count] = m_pimages1T[id]->getColor(rpos[0], rpos[1], level);
      lpos += xaxis0;
      rpos += xaxis1;
    }
  }

  if (m_ssdsT[id] == 0) {
    Cphoto::normalize(lhscolors);
    Cphoto::normalize(rhscolors);
    return Cphoto::idot(lhscolors, rhscolors);
  }
  else
    return Cphoto::ssd(lhscolors, rhscolors);    
}

int Cpatch2dOptimizer::isTooMuchMove(const double* const vect, const int final, const int id) const {
  const float scale = (final ? 1.0f : 1.5) * (0x0001 << m_levelsT[id]);
  
  if (scale * m_maxMoves[0] < fabs(vect[0]) || scale * m_maxMoves[1] < fabs(vect[1]) ||
      scale * m_maxMoves[2] < fabs(vect[2]) || scale * m_maxMoves[3] < fabs(vect[3]) ||
      scale * m_maxMoves[4] < fabs(vect[4]) || scale * m_maxMoves[5] < fabs(vect[5]))
    return 1;
  else
    return 0;
}

//----------------------------------------------------------------------
// Position optimization
//----------------------------------------------------------------------
// optimize, patch2d's location and shapes
double Cpatch2dOptimizer::optimizePos(const Cpatch2d& lhs, Cpatch2d& rhs,
				      const Image::Cimage& lhsimage, const Image::Cimage& rhsimage,
				      const int halfsize, const int level,
				      const int id) {
  if (isInvalidAxes(lhs.m_xaxis, lhs.m_yaxis) ||
      isInvalidAxes(rhs.m_xaxis, rhs.m_yaxis)) {
    cerr << "Invalid axes at inputs." << endl;
    return 2.0f;
  }
  
  m_ppatches0T[id] = &lhs;      m_ppatches1T[id] = &rhs;
  m_pimages0T[id] = &lhsimage;  m_pimages1T[id] = &rhsimage;
  m_halfsizesT[id] = halfsize;  m_levelsT[id] = level;

  m_patches1OrgT[id] = rhs;

  int code;  double pval_min;  double vect[2];  int len = 2;
  const int max_iterations = 8;

  // Position, xaxis, yaxis
  double initial_coord_x0s[2];
  initial_coord_x0s[0] = 1.0f;  initial_coord_x0s[1] = 1.0f;
  
  for (int i = 0; i < 2; ++i)
    vect[i] = 0.0;
  
  if (id == 0)
    wn_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			     len, evalPos0, max_iterations);
  else if (id == 1)
    wn2_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, evalPos1, max_iterations);
  else if (id == 2)
    wn3_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, evalPos2, max_iterations);
  else if (id == 3)
    wn4_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, evalPos3, max_iterations);
  else {
    cerr << "No way in refinePatch" << endl;
    exit (1);
  }

  double score;
  if (m_ssdsT[id] == 0)
    score = pval_min;
  else {
    m_ssdsT[id] = 0;
    score = evalPos(vect, id);
    m_ssdsT[id] = 1;
  }    
  
  if (score != 2.0f && (code == WN_SUCCESS || code == WN_SUBOPTIMAL)) {
    if (isTooMuchMovePos(vect, 1, id) == 0) {
      encodePos(vect, id);
      m_stats[0]++;
      return score;
    }
    else {
      m_stats[1]++;
      *m_ppatches1T[id] = m_patches1OrgT[id];
      return 2.0f;
    }
  }
  else {
    m_stats[2]++;
    *m_ppatches1T[id] = m_patches1OrgT[id];
    return 2.0f;
  }
}

double Cpatch2dOptimizer::evalPos0(double* xs) {
  return m_one->evalPos(xs, 0);
}

double Cpatch2dOptimizer::evalPos1(double* xs) {
  return m_one->evalPos(xs, 1);
}

double Cpatch2dOptimizer::evalPos2(double* xs) {
  return m_one->evalPos(xs, 2);
}

double Cpatch2dOptimizer::evalPos3(double* xs) {
  return m_one->evalPos(xs, 3);
}

void Cpatch2dOptimizer::encodePos(const double* const vect, const int id) {
  const int level = m_levelsT[id];
  const float scale = (float)(0x0001 << level);    
  
  m_ppatches1T[id]->m_center[0] = m_patches1OrgT[id].m_center[0]
    + vect[0] * scale;
  m_ppatches1T[id]->m_center[1] = m_patches1OrgT[id].m_center[1]
    + vect[1] * scale;
}

double Cpatch2dOptimizer::evalPos(const double* const vect, const int id) {
  if (isTooMuchMovePos(vect, 0, id))
    return 2.0f;
  
  encodePos(vect, id);

  const int level = m_levelsT[id];
  const int halfsize = m_halfsizesT[id];
  
  const float scale = (float)(0x0001 << level);  
  const Vec2f center0 = m_ppatches0T[id]->m_center / scale;
  const Vec2f center1 = m_ppatches1T[id]->m_center / scale;
  const Vec2f& xaxis0 = m_ppatches0T[id]->m_xaxis;
  const Vec2f& yaxis0 = m_ppatches0T[id]->m_yaxis;
  const Vec2f& xaxis1 = m_ppatches1T[id]->m_xaxis;
  const Vec2f& yaxis1 = m_ppatches1T[id]->m_yaxis;

  if (isOutside(center0, xaxis0, yaxis0, halfsize,
		m_pimages0T[id]->getWidth(level), m_pimages0T[id]->getHeight(level)) ||
      isOutside(center1, xaxis1, yaxis1, halfsize,
		m_pimages1T[id]->getWidth(level), m_pimages1T[id]->getHeight(level)))
    return 2.0;

  const int size = (2 * halfsize + 1) * (2 * halfsize + 1);
  vector<Vec3f> lhscolors, rhscolors;
  lhscolors.resize(size);       rhscolors.resize(size);
  int count = -1;
  for (int y = -halfsize; y <= halfsize; ++y) {
    Vec2f lpos = center0 + y * yaxis0 - halfsize * xaxis0;
    Vec2f rpos = center1 + y * yaxis1 - halfsize * xaxis1;
    for (int x = -halfsize; x <= halfsize; ++x) {
      count++;
      lhscolors[count] = m_pimages0T[id]->getColor(lpos[0], lpos[1], level);
      rhscolors[count] = m_pimages1T[id]->getColor(rpos[0], rpos[1], level);
      lpos += xaxis0;
      rpos += xaxis1;
    }
  }

  if (m_ssdsT[id] == 0) {
    Cphoto::normalize(lhscolors);
    Cphoto::normalize(rhscolors);
    return Cphoto::idot(lhscolors, rhscolors);
  }
  else
    return Cphoto::ssd(lhscolors, rhscolors);    
}

int Cpatch2dOptimizer::isTooMuchMovePos(const double* const vect, const int final, const int id) const {
  const float scale = (final ? 1.0f : 1.5) * (0x0001 << m_levelsT[id]);
  
  if (scale * m_maxMoves[0] < fabs(vect[0]) || scale * m_maxMoves[1] < fabs(vect[1]))
    return 1;
  else
    return 0;
}

//----------------------------------------------------------------------
// 4 param optimization optimize axes.
//----------------------------------------------------------------------
double Cpatch2dOptimizer::optimizeAxes(const Cpatch2d& lhs, Cpatch2d& rhs,
				       const Image::Cimage& lhsimage,
				       const Image::Cimage& rhsimage,
				       const int halfsize, const int level,
				       const int id) {
  if (isInvalidAxes(lhs.m_xaxis, lhs.m_yaxis) ||
      isInvalidAxes(rhs.m_xaxis, rhs.m_yaxis)) {
    cerr << "Invalid axes at inputs." << endl;
    return 2.0f;
  }
  
  m_ppatches0T[id] = &lhs;      m_ppatches1T[id] = &rhs;
  m_pimages0T[id] = &lhsimage;  m_pimages1T[id] = &rhsimage;
  m_halfsizesT[id] = halfsize;  m_levelsT[id] = level;

  m_patches1OrgT[id] = rhs;

  int code;  double pval_min;  double vect[4];  int len = 4;
  const int max_iterations = 15;

  // Position, xaxis, yaxis
  double initial_coord_x0s[4];
  initial_coord_x0s[0] = 0.5f;  initial_coord_x0s[1] = 0.5f;
  initial_coord_x0s[2] = 0.5f;  initial_coord_x0s[2] = 0.5f;

  for (int i = 0; i < 4; ++i)
    vect[i] = 0.0;
  
  if (id == 0)
    wn_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			     len, evalAxes0, max_iterations);
  else if (id == 1)
    wn2_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, evalAxes1, max_iterations);
  else if (id == 2)
    wn3_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, evalAxes2, max_iterations);
  else if (id == 3)
    wn4_conj_direction_method(&code, &pval_min, vect, initial_coord_x0s,
			      len, evalAxes3, max_iterations);
  else {
    cerr << "No way in refinePatch" << endl;
    exit (1);
  }

  double score;
  if (m_ssdsT[id] == 0)
    score = pval_min;
  else {
    m_ssdsT[id] = 0;
    score = evalAxes(vect, id);
    m_ssdsT[id] = 1;
  }    

  if (score != 2.0f && (code == WN_SUCCESS || code == WN_SUBOPTIMAL)) {
    if (isTooMuchMoveAxes(vect, 1, id) == 0) {
      m_stats[0]++;
      encodeAxes(vect, id);
      return score;
    }
    else {
      m_stats[1]++;
      *m_ppatches1T[id] = m_patches1OrgT[id];
      return 2.0f;
    }
  }
  else {
    m_stats[2]++;
    *m_ppatches1T[id] = m_patches1OrgT[id];
    return 2.0f;
  }
}

double Cpatch2dOptimizer::evalAxes0(double* xs) {
  return m_one->evalAxes(xs, 0);
}

double Cpatch2dOptimizer::evalAxes1(double* xs) {
  return m_one->evalAxes(xs, 1);
}

double Cpatch2dOptimizer::evalAxes2(double* xs) {
  return m_one->evalAxes(xs, 2);
}

double Cpatch2dOptimizer::evalAxes3(double* xs) {
  return m_one->evalAxes(xs, 3);
}

void Cpatch2dOptimizer::encodeAxes(const double* const vect, const int id) {
  m_ppatches1T[id]->m_xaxis[0] = m_patches1OrgT[id].m_xaxis[0] + vect[0];
  m_ppatches1T[id]->m_xaxis[1] = m_patches1OrgT[id].m_xaxis[1] + vect[1];
  m_ppatches1T[id]->m_yaxis[0] = m_patches1OrgT[id].m_yaxis[0] + vect[2];
  m_ppatches1T[id]->m_yaxis[1] = m_patches1OrgT[id].m_yaxis[1] + vect[3];
}

double Cpatch2dOptimizer::evalAxes(const double* const vect, const int id) {
  if (isTooMuchMoveAxes(vect, 0, id))
    return 2.0f;
  
  encodeAxes(vect, id);

  if (isInvalidAxes(m_ppatches1T[id]->m_xaxis, m_ppatches1T[id]->m_yaxis))
    return 2.0f;

  const int level = m_levelsT[id];
  const int halfsize = m_halfsizesT[id];

  const float scale = (float)(0x0001 << level);  
  const Vec2f center0 = m_ppatches0T[id]->m_center / scale;
  const Vec2f center1 = m_ppatches1T[id]->m_center / scale;
  const Vec2f& xaxis0 = m_ppatches0T[id]->m_xaxis;
  const Vec2f& yaxis0 = m_ppatches0T[id]->m_yaxis;
  const Vec2f& xaxis1 = m_ppatches1T[id]->m_xaxis;
  const Vec2f& yaxis1 = m_ppatches1T[id]->m_yaxis;

  if (isOutside(center0, xaxis0, yaxis0, halfsize,
		m_pimages0T[id]->getWidth(level), m_pimages0T[id]->getHeight(level)) ||
      isOutside(center1, xaxis1, yaxis1, halfsize,
		m_pimages1T[id]->getWidth(level), m_pimages1T[id]->getHeight(level)))
    return 2.0;

  const int size = (2 * halfsize + 1) * (2 * halfsize + 1);
  vector<Vec3f> lhscolors, rhscolors;
  lhscolors.resize(size);       rhscolors.resize(size);
  int count = -1;
  for (int y = -halfsize; y <= halfsize; ++y) {
    Vec2f lpos = center0 + y * yaxis0 - halfsize * xaxis0;
    Vec2f rpos = center1 + y * yaxis1 - halfsize * xaxis1;
    for (int x = -halfsize; x <= halfsize; ++x) {
      count++;
      lhscolors[count] = m_pimages0T[id]->getColor(lpos[0], lpos[1], level);
      rhscolors[count] = m_pimages1T[id]->getColor(rpos[0], rpos[1], level);
      lpos += xaxis0;
      rpos += xaxis1;
    }
  }

  if (m_ssdsT[id] == 0) {
    Cphoto::normalize(lhscolors);
    Cphoto::normalize(rhscolors);
    return Cphoto::idot(lhscolors, rhscolors);
  }
  else
    return Cphoto::ssd(lhscolors, rhscolors);
}

int Cpatch2dOptimizer::isTooMuchMoveAxes(const double* const vect, const int final, const int id) const {
  const float scale = (final ? 1.0f : 1.5) * (0x0001 << m_levelsT[id]);
  
  if (scale * m_maxMoves[2] < fabs(vect[0]) || scale * m_maxMoves[3] < fabs(vect[1]) ||
      scale * m_maxMoves[4] < fabs(vect[2]) || scale * m_maxMoves[5] < fabs(vect[3]))
    return 1;
  else
    return 0;
}

//----------------------------------------------------------------------
int Cpatch2dOptimizer::isOutside(const Vec2f& center,
				 const Vec2f& xaxis, const Vec2f& yaxis,
				 const int halfsize,
				 const int width, const int height) const {
  // Check range for the first image
  const float xmargin = (fabs(xaxis[0]) + fabs(yaxis[0])) * halfsize;
  const float minx = center[0] - xmargin;
  const float maxx = center[0] + xmargin;
    
  const float ymargin = (fabs(xaxis[1]) + fabs(yaxis[1])) * halfsize;
  const float miny = center[1] - ymargin;
  const float maxy = center[1] + ymargin;

  // We need additional 1 to use bicubic interpolation
  const int additional = 1;
  
  if (minx < additional || width - 1 - additional <= maxx ||
      miny < additional || height - 1 - additional <= maxy)
    return 1;
  else
    return 0;
}  

int Cpatch2dOptimizer::isInvalidAxes(const Vec2f& xaxis,
				     const Vec2f& yaxis) {
  // Thresholds
  const float minlen = 0.2;  const float maxlen = 5.0;
  const float mindet = 0.1;
  
  const float xlen = xaxis.norm();
  const float ylen = yaxis.norm();

  if (xlen < minlen || maxlen < xlen ||
      ylen < minlen || maxlen < ylen)
    return 1;

  const float det = xaxis[0] * yaxis[1] - xaxis[1] * yaxis[0];

  if (det < mindet)
    return 1;
  
  return 0;
  
  /*
  if (xaxis[0] * yaxis[1] - xaxis[1] * yaxis[0] > 0.0)
    return 0;
  else
    return 1;
  */
}

void Cpatch2dOptimizer::showStats(void) const {
  cerr << m_stats[0] << ' ' << m_stats[1] << ' ' << m_stats[2]
       << "  success too-much-move optim-fail" << endl;
}

void Cpatch2dOptimizer::initStats(void) {
  for (int i = 0; i < (int)m_stats.size(); ++i)
    m_stats[i] = 0;
}

//----------------------------------------------------------------------
void Cpatch2dOptimizer::optimize(Cpatch2dTrack& patch2dTrack,
				 std::vector<Image::Cimage>& images,
				 const int firstFrame, const int lastFrame, const int refFrame,
				 const int halfsize, const int level) {
  if (firstFrame <= refFrame && refFrame < lastFrame) {
    cerr << "refFrame must not be in [firstFrame lastFrame): "
	 << firstFrame << ' ' << lastFrame << ' ' << refFrame << endl;
    exit (1);
  }
  
  m_one = this;
  m_ppatch2dTrack = &patch2dTrack;
  m_pimages = &images;
  m_firstFrame = firstFrame;
  m_lastFrame = lastFrame;
  m_refFrame = refFrame;
  m_halfsize = halfsize;
  m_level = level;
  
  // Fix only the reference frame and optimize other m_center, m_xaxis, m_yaxis
  const int n = (lastFrame - firstFrame) * 6;
  const int m = (lastFrame - firstFrame + 1) * (lastFrame - firstFrame) / 2;

  m_orgx.resize(n);
  int count = 0;
  for (int f = firstFrame; f < lastFrame; ++f) {
    m_orgx[count++] = patch2dTrack.m_patch2ds[f].m_center[0];
    m_orgx[count++] = patch2dTrack.m_patch2ds[f].m_center[1];
    m_orgx[count++] = patch2dTrack.m_patch2ds[f].m_xaxis[0];
    m_orgx[count++] = patch2dTrack.m_patch2ds[f].m_xaxis[1];
    m_orgx[count++] = patch2dTrack.m_patch2ds[f].m_yaxis[0];
    m_orgx[count++] = patch2dTrack.m_patch2ds[f].m_yaxis[1];
  }
  
  double x[n];  double fvec[m];
  int msk[n];   int info, nfev;
  // ?????
  int maxfev = 1000 * n;
  const double tol = sqrt(dpmpar[0]);
		       
  for (int i = 0; i < n; ++i)
    x[i] = 0.0;
  
  // when showing initial error
  int iflag;
  func(m, n, x, fvec, &iflag, NULL);
  double fnorm = enorm(m, fvec);
  cerr << "Error " << fnorm << " -> " << flush;

  for (int i = 0; i < m; ++i)
    fvec[i] = 0.0;
  for (int i = 0; i < n; ++i)
    msk[i] = 1;

  //----------------------------------------------------------------------
  // Compute energy functions first
  lmdif0(sfunc, m, n, x, msk, fvec, tol, &info, &nfev, maxfev, 0.25);

  //cerr << endl
  //<< "----------------------------------------------------------------------" << endl
  //<< "Minimization Done " << endl;
  fnorm = enorm(m, fvec);
  cerr << fnorm << "   Exit Param: " << info << endl;
  
  /*
c         info = 0  improper input parameters.
c
c         info = 1  both actual and predicted relative reductions
c                   in the sum of squares are at most ftol.
c
c         info = 2  relative error between two consecutive iterates
c                   is at most xtol.
c
c         info = 3  conditions for info = 1 and info = 2 both hold.
c
c         info = 4  the cosine of the angle between fvec and any
c                   column of the jacobian is at most gtol in
c                   absolute value.
c
c         info = 5  number of calls to fcn has reached or
c                   exceeded maxfev.
c
c         info = 6  ftol is too small. no further reduction in
c                   the sum of squares is possible.
c
c         info = 7  xtol is too small. no further improvement in
c                   the approximate solution x is possible.
c
c         info = 8  gtol is too small. fvec is orthogonal to the
c                   columns of the jacobian to machine precision.
   */
  setFromX(x);
}

void Cpatch2dOptimizer::setValid(const double* const x,
				 std::vector<unsigned char>& valid) {
  const int scale = 0x0001 << m_level;
  valid.resize(m_lastFrame - m_firstFrame);
  
  int count = 0;
  for (int f = m_firstFrame; f < m_lastFrame; ++f) {
    valid[f - m_firstFrame] = 1;
    
    // Check the amount of movements
    if (m_maxMoves[0] < fabs(x[count++]) || m_maxMoves[1] < fabs(x[count++]) ||
	m_maxMoves[2] < fabs(x[count++]) || m_maxMoves[3] < fabs(x[count++]) ||
	m_maxMoves[4] < fabs(x[count++]) || m_maxMoves[5] < fabs(x[count++])) {
      valid[f - m_firstFrame] = 0;
      continue;
    }

    // Check the axes
    if (isInvalidAxes(m_ppatch2dTrack->m_patch2ds[f].m_xaxis,
		      m_ppatch2dTrack->m_patch2ds[f].m_yaxis)) {
      valid[f - m_firstFrame] = 0;
      continue;
    }

    // Check outside
    const Vec2f center = m_ppatch2dTrack->m_patch2ds[f].m_center / scale;
    const Vec2f& xaxis = m_ppatch2dTrack->m_patch2ds[f].m_xaxis;
    const Vec2f& yaxis = m_ppatch2dTrack->m_patch2ds[f].m_yaxis;

    if (isOutside(center, xaxis, yaxis, m_halfsize,
		  (*m_pimages)[f].getWidth(m_level),
		  (*m_pimages)[f].getHeight(m_level))) {
      valid[f - m_firstFrame] = 0;
      continue;
    }
  }

  // for the reference frame
  // Check outside
  const Vec2f center = m_ppatch2dTrack->m_patch2ds[m_refFrame].m_center / scale;
  const Vec2f& xaxis = m_ppatch2dTrack->m_patch2ds[m_refFrame].m_xaxis;
  const Vec2f& yaxis = m_ppatch2dTrack->m_patch2ds[m_refFrame].m_yaxis;
  
  if (isOutside(center, xaxis, yaxis, m_halfsize,
		(*m_pimages)[m_refFrame].getWidth(m_level),
		(*m_pimages)[m_refFrame].getHeight(m_level)))
    valid.push_back(0);
  else
    valid.push_back(1);
}

void Cpatch2dOptimizer::sfunc(int m, int n, double* x, double* fvec, int* iflag,
			      void* arg) {
  m_one->func(m, n, x, fvec, iflag, arg);
}

void Cpatch2dOptimizer::func(int m, int n, double* x, double* fvec, int* iflag,
			     void* arg) {
  setFromX(x);

  // check if each frame is valid or invalid
  vector<unsigned char> valid;
  setValid(x, valid);
  
  vector<vector<Vec3f> > colors;
  colors.resize(m_lastFrame - m_firstFrame + 1);
  vector<int> frameids;
  for (int f = m_firstFrame; f < m_lastFrame; ++f)
    frameids.push_back(f);
  frameids.push_back(m_refFrame);

  const int size = (2 * m_halfsize + 1) * (2 * m_halfsize + 1);
  const float scale = 0x0001 << m_level;
  for (int i = 0; i < (int)frameids.size(); ++i) {
    if (valid[i] == 0)
      continue;
    
    const int f = frameids[i];
    const Vec2f center = m_ppatch2dTrack->m_patch2ds[f].m_center / scale;
    const Vec2f& xaxis = m_ppatch2dTrack->m_patch2ds[f].m_xaxis;
    const Vec2f& yaxis = m_ppatch2dTrack->m_patch2ds[f].m_yaxis;
    
    colors[i].resize(size);
    int count = -1;
    for (int y = -m_halfsize; y <= m_halfsize; ++y) {
      Vec2f pos = center + y * yaxis - m_halfsize * xaxis;
      for (int x = -m_halfsize; x <= m_halfsize; ++x) {
	count++;
	colors[i][count] =
	  (*m_pimages)[f].getColor(pos[0], pos[1], m_level);
      }
    }
    
    Cphoto::normalize(colors[i]);
  }
  
  int count = 0;
  for (int i = 0; i < (int)frameids.size(); ++i) {
    for (int j = i+1; j < (int)frameids.size(); ++j) {
      if (valid[i] == 0 || valid[j] == 0)
	fvec[count] = 2.0;
      else
	fvec[count] = Cphoto::idot(colors[i], colors[j]);
      count++;
    }
  }
}

void Cpatch2dOptimizer::setFromX(const double* const x) {
  const int scale = 0x0001 << m_level;
  int count = 0;
  for (int f = m_firstFrame; f < m_lastFrame; ++f) {
    m_ppatch2dTrack->m_patch2ds[f].m_center[0] = m_orgx[count] + x[count] * scale;
    count++;
    m_ppatch2dTrack->m_patch2ds[f].m_center[1] = m_orgx[count] + x[count] * scale;
    count++;
    m_ppatch2dTrack->m_patch2ds[f].m_xaxis[0] = m_orgx[count] + x[count];
    count++;
    m_ppatch2dTrack->m_patch2ds[f].m_xaxis[1] = m_orgx[count] + x[count];
    count++;
    m_ppatch2dTrack->m_patch2ds[f].m_yaxis[0] = m_orgx[count] + x[count];
    count++;
    m_ppatch2dTrack->m_patch2ds[f].m_yaxis[1] = m_orgx[count] + x[count];
    count++;
  }
}

