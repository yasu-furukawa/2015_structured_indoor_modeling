#ifndef IMAGEPROCESS_PATCH2DOPTIMIZER_H
#define IMAGEPROCESS_PATCH2DOPTIMIZER_H

#include "../image/image.h"
#include "patch2d.h"
#include "patch2dTrack.h"

namespace ImageProcess {
class Cpatch2dOptimizer {  
 public:
  Cpatch2dOptimizer(void);
  virtual ~Cpatch2dOptimizer();

  void optimize(Cpatch2dTrack& patch2dTrack,
		std::vector<Image::Cimage>& images,
		const int firstFrame, const int lastFrame, const int refFrame,
		const int halfsize, const int level);
  
  double optimize(const Cpatch2d& lhs, Cpatch2d& rhs,
		  const Image::Cimage& lhsimage, const Image::Cimage& rhsimage,
		  const int halfsize, const int level = 0,
		  const int id = 0);
  
  double optimizePos(const Cpatch2d& lhs, Cpatch2d& rhs,
		     const Image::Cimage& lhsimage, const Image::Cimage& rhsimage,
		     const int halfsize, const int level = 0,
		     const int id = 0);

  double optimizeAxes(const Cpatch2d& lhs, Cpatch2d& rhs,
		      const Image::Cimage& lhsimage, const Image::Cimage& rhsimage,
		      const int halfsize, const int level = 0,
		      const int id = 0);

  void showStats(void) const;
  void initStats(void);
  
  // number of cpus
  int m_CPU;

 protected:
  //----------------------------------------------------------------------
  // 6 parameter optimization
  //----------------------------------------------------------------------
  // Evaluate a correlation score given 6 parameters
  double eval(const double* const vect, const int id);
  static double eval0(double* xs);
  static double eval1(double* xs);
  static double eval2(double* xs);
  static double eval3(double* xs);

  void encode(const double* const vect, const int id);
  
  // Check if optimizer moves too much or not
  int isTooMuchMove(const double* const vect, const int final, const int id) const;

  //----------------------------------------------------------------------
  // Position (2 params) optimization
  //----------------------------------------------------------------------
  double evalPos(const double* const vect, const int id);
  static double evalPos0(double* xs);
  static double evalPos1(double* xs);
  static double evalPos2(double* xs);
  static double evalPos3(double* xs);

  void encodePos(const double* const vect, const int id);
  
  // Check if optimizer moves too much or not
  int isTooMuchMovePos(const double* const vect, const int final, const int id) const;

  //----------------------------------------------------------------------
  // Axes (4 params) optimization
  //----------------------------------------------------------------------
  double evalAxes(const double* const vect, const int id);
  static double evalAxes0(double* xs);
  static double evalAxes1(double* xs);
  static double evalAxes2(double* xs);
  static double evalAxes3(double* xs);

  void encodeAxes(const double* const vect, const int id);
  
  // Check if optimizer moves too much or not
  int isTooMuchMoveAxes(const double* const vect, const int final, const int id) const;

  //----------------------------------------------------------------------
  int isOutside(const Vec2f& center,
		const Vec2f& xaxis, const Vec2f& yaxis,
		const int halfsize,
		const int width, const int height) const;

  static int isInvalidAxes(const Vec2f& xaxis, const Vec2f& yaxis);
  
  static Cpatch2dOptimizer* m_one;
  
  // Left patch, right patch
  std::vector<const Cpatch2d*> m_ppatches0T;
  std::vector<Cpatch2d*> m_ppatches1T;
  // original right patch
  std::vector<Cpatch2d> m_patches1OrgT;
  
  // Left image, right image
  std::vector<const Image::Cimage*> m_pimages0T;
  std::vector<const Image::Cimage*> m_pimages1T;

  // which consistency function. (ncc, ssd)
  std::vector<int> m_ssdsT;
  
  // halfsize
  std::vector<int> m_halfsizesT;
  // level
  std::vector<int> m_levelsT;

  // How much movement is allowed for each parameter
  float m_maxMoves[6];
  
  //----------------------------------------------------------------------
  // Statistics
  //----------------------------------------------------------------------
  // 0: success
  // 1: too much move
  // 2: optim fail
  std::vector<int> m_stats;

  //----------------------------------------------------------------------
  // For final optimization
  ImageProcess::Cpatch2dTrack* m_ppatch2dTrack;
  std::vector<Image::Cimage>* m_pimages;
  int m_firstFrame;
  int m_lastFrame;
  int m_refFrame;
  int m_halfsize;
  int m_level;
  std::vector<double> m_orgx;

  void setValid(const double* const x, std::vector<unsigned char>& valid);
  static void sfunc(int m, int n, double* x, double* fvec, int* iflag, void* arg);
  void func(int m, int n, double* x, double* fvec, int* iflag, void* arg);
  void setFromX(const double* const x);
  void setToX(double* const x) const;
};
};

#endif // IMAGEPROCESS_PATCH2DOPTIMIZER_H
