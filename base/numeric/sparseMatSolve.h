#ifndef NUMERIC_SPARSEMATSOLVE_H
#define NUMERIC_SPARSEMATSOLVE_H

#include <cmath>

template <class T>
void wJacobi(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
	     const int iter, const T omega);

template <class T>
void GaussSeidel(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
		 const int iter);

template <class T>
void Richardson(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
		const int iter, const T alpha);

// without preconditioner
template <class T>
void GMRES(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
	   int m, const T rtol, const int maxit,
	   int& iter, std::vector<T>& res);

// with preconditioner
template <class T>
void GMRESILU(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
	      int m, const T rtol, const int maxit,
	      int& iter, std::vector<T>& res);

// solve LD^{-1}U x = b
template <class T>
void solveLDU(const CsparseMat<T>& LDU,
	      Cvec<T>& x,
	      const Cvec<T>& b);

template <class T>
void CG(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
	const T rtol, const int maxit, int& iter,
	std::vector<T>& res);

template <class T>
void extractD(const CsparseMat<T>& A, CsparseMat<T>& D);

template <class T>
void extractE(const CsparseMat<T>& A, CsparseMat<T>& E);

template <class T>
void extractF(const CsparseMat<T>& A, CsparseMat<T>& F);

template <class T>
void res(const CsparseMat<T>& A, const Cvec<T>& x, const Cvec<T>& b, Cvec<T>& r);
template <class T>
T normres(const CsparseMat<T>& A, const Cvec<T>& x, const Cvec<T>& b);
template <class T>
void solveLT(const CsparseMat<T>& A, Cvec<T>& x, const Cvec<T>& b);
template <class T>
void solveUT(const CsparseMat<T>& A, Cvec<T>& x, const Cvec<T>& b);
template <class T>
void multD(const CsparseMat<T>& A, Cvec<T>& x, const Cvec<T>& b);

template <class T>
void solveLT(const std::vector<std::vector<T> >& A, Cvec<T>& x, const Cvec<T>& b);
template <class T>
void solveUT(const std::vector<std::vector<T> >& A, Cvec<T>& x, const Cvec<T>& b);

// Preconditioning
// Imcomplete LU factorization (0)
// A =? L D U
template <class T>
void ILU0(const CsparseMat<T>& A, CsparseMat<T>& LDU);

//======================================================================
//
// Implementation
//
//======================================================================

// for dense matrix
template <class T>
void solveLT(const std::vector<std::vector<T> >& A, Cvec<T>& x, const Cvec<T>& b) {
  if (A.empty()) {
    std::cerr << "Empty matrix A in solveLT" << std::endl;    exit (1);
  }
  
  const int n = (int)A[0].size();
  x.resize(n);

  for (int r = 0; r < n; ++r) {
    double ans = 0.0;
    for (int c = 0; c < r; ++c)
      ans += A[r][c] * x[c];
    
    if (A[r][r] == 0.0) {
      std::cerr << "Diag is zero in solveLT" << std::endl;      exit (1);
    }
    
    x[r] = (b[r] - ans) / A[r][r];
  }
};

// for dense matrix
template <class T>
void solveUT(const std::vector<std::vector<T> >& A, Cvec<T>& x, const Cvec<T>& b) {
  if (A.empty()) {
    std::cerr << "Empty matrix A in solveUT" << std::endl;    exit (1);
  }
  
  const int n = (int)A[0].size();
  x.resize(n);

  for (int r = n-1; 0 <= r; --r) {
    double ans = 0.0;
    for (int c = r+1; c < n; ++c)
      ans += A[r][c] * x[c];

    if (A[r][r] == 0.0) {
      std::cerr << "Diag is zero in solveUT" << std::endl;      exit (1);
    }
    x[r] = (b[r] - ans) / A[r][r];
  }
};

// Extract diagonal matrix from a general matrix A
template <class T>
void extractD(const CsparseMat<T>& A, CsparseMat<T>& D) {
  const int row = A.getRow();
  const int col = A.getCol();
  std::vector<T> val;
  std::vector<int> colind, rowind;

  rowind.push_back(0);
  for (int r = 0; r < row; ++r){
    const int begin = A.m_rowind[r];
    const int end = A.m_rowind[r+1];
    
    for (int i = begin; i < end; ++i) {
      const int ctmp = A.m_colind[i];
      if (ctmp < r)
	continue;
      else if (ctmp == r) {
	val.push_back(A.m_val[i]);
	colind.push_back(ctmp);
	break;
      }
    }
    rowind.push_back((int)colind.size());
  }

  D.init(row, col, rowind[row+1], &val[0], &colind[0], &rowind[0]);
};

// Extract strict lower
template <class T>
void extractE(const CsparseMat<T>& A, CsparseMat<T>& E) {
  const int row = A.getRow();
  const int col = A.getCol();
  std::vector<T> val;
  std::vector<int> colind, rowind;

  rowind.push_back(0);
  for (int r = 0; r < row; ++r){
    const int begin = A.m_rowind[r];
    const int end = A.m_rowind[r+1];
    
    for (int i = begin; i < end; ++i) {
      const int ctmp = A.m_colind[i];
      if (ctmp < r) {
	val.push_back(-A.m_val[i]);
	colind.push_back(ctmp);
      }
      else
	break;
    }
    rowind.push_back((int)colind.size());
  }

  E.init(row, col, rowind[row+1], &val[0], &colind[0], &rowind[0]);
};

// Extract strict upper
template <class T>
void extractF(const CsparseMat<T>& A, CsparseMat<T>& F) {
  const int row = A.getRow();
  const int col = A.getCol();
  std::vector<T> val;
  std::vector<int> colind, rowind;

  rowind.push_back(0);
  for (int r = 0; r < row; ++r){
    const int begin = A.m_rowind[r];
    const int end = A.m_rowind[r+1];
    
    for (int i = begin; i < end; ++i) {
      const int ctmp = A.m_colind[i];
      if (ctmp <= r)
	continue;
      else {
	val.push_back(-A.m_val[i]);
	colind.push_back(ctmp);
      }
    }
    rowind.push_back((int)colind.size());
  }
  
  F.init(row, col, rowind[row+1], &val[0], &colind[0], &rowind[0]);
};

// compute residual
template <class T>
void res(const CsparseMat<T>& A, const Cvec<T>& x, const Cvec<T>& b, Cvec<T>& r) {
  r = b - A * x;
};

// compute norm of the residual
template <class T>
T normres(const CsparseMat<T>& A, const Cvec<T>& x, const Cvec<T>& b) {
  Cvec<T> r;
  res(A, x, b, r);
  return r.norm();
};

// Solving lower triangular system. A does not have to be lower
// triangular, we just use lower triangular part of A.
template <class T>
void solveLT(const CsparseMat<T>& A, Cvec<T>& x, const Cvec<T>& b) {
  const int col = A.getCol();

  for (int r = 0; r < col; ++r) {
    const int begin = A.m_rowind[r];
    const int end = A.m_rowind[r+1];

    T ans = 0.0;
    T diag = 0.0;
    
    for (int j = begin; j < end; ++j) {
      const int col = A.m_colind[j];
      if (col < r) {
	ans += A.m_val[j] * x[col];
	continue;
      }

      if (col == r)
	diag = A.m_val[j];
      
      break;
    }

    if (diag == 0.0) {
      std::cerr << "Diag is zero: " << r << std::endl;      exit (1);
    }
    x[r] = (b[r] - ans) / diag;
  }
};

// Solving upper triangular system. A does not have to be upper
// triangular, we just use upper triangular part of A.
template <class T>
void solveUT(const CsparseMat<T>& A, Cvec<T>& x, const Cvec<T>& b) {
  const int col = A.getCol();

  for (int r = col-1; 0 <= r; --r) {
    int begin = A.m_rowind[r];
    const int end = A.m_rowind[r+1];
    
    T ans = 0.0;
    T diag = 0.0;
    
    for (int j = end - 1; begin <= j; --j) {
      const int col = A.m_colind[j];
      if (r < col) {
	ans += A.m_val[j] * x[col];
	continue;
      }

      if (col == r)
	diag = A.m_val[j];

      break;
    }

    x[r] = (b[r] - ans) / diag;
  }  
};

// compute 
template <class T>
void multD(const CsparseMat<T>& A, Cvec<T>& x, const Cvec<T>& b) {
  for (int r = 0; r < A.getRow(); ++r) {
    const T numer = A.get(r, r);
    x[r] = b[r] * numer;
  }
};

// just follow the algorithm in the textbook.
template <class T>
void wJacobi(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
	     const int iter, const T omega) {
  std::cerr << "prep" << std::flush;
  CsparseMat<T> D;
  extractD(A, D);

  //----------------------------------------------------------------------
  // construct matrices that are used in a linear system
  CsparseMat<T> DwE;
  extractE(A, DwE);

  DwE = D - (omega * DwE);

  CsparseMat<T> wFD;
  extractF(A, wFD);
  wFD = omega * wFD + (1.0 - omega) * D;
  
  const Cvec<T> wb = omega * b;

  // solve a linear system of lower triangular matrix
  Cvec<T> xtmp = x;
  std::cerr << "solve" << std::flush;
  for (int t = 0; t < iter; ++t) {
    xtmp = wFD * x + wb;
    solveLT(DwE, x, xtmp);
  }
  std::cerr << "done" << std::endl;
};

template <class T>
void GaussSeidel(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
		 const int iter) {
  const int n = A.getCol();
  const int m = A.getRow();
  if (n != m) {
    std::cerr << "n != m in GaussSeidel: " << m << ' ' << n << std::endl;
    exit (1);
  }

  // just follow the algorithm in the textbook.
  for (int t = 0; t < iter; ++t) {
    for (int r = 0; r < n; ++r) {
      const int begin = A.m_rowind[r];
      const int end = A.m_rowind[r+1];
      
      T ans = 0.0;
      T diag = 0.0;

      for (int j = begin; j < end; ++j) {
	const int col = A.m_colind[j];
	if (col != r)
	  ans += A.m_val[j] * x[col];
	else
	  diag = A.m_val[j];
      }
      
      x[r] = (b[r] - ans) / diag;
    }
  }
};

// very simple implementation.
template <class T>
void Richardson(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
		const int iter, const T alpha) {
  Cvec<T> r;
  for (int t = 0; t < iter; ++t) {
    r = b - A * x;
    x += alpha * r;
  }
};

template <class T>
void GMRESSub(Cvec<T>& x, const int m, std::vector<T>& res,
	      std::vector<std::vector<T> >& hessen,
	      const std::vector<Cvec<T> >& vs, const T beta) {  
  // solve linear system by givens rotation
  // use givens rotation to make it upper triangular system and solve.
  Cvec<T> be;    be.resize(m+1);
  be[0] = beta;
  for (int i = 1; i < m+1; ++i)
    be[i] = 0.0;

  // premultiply givens rotation matrices
  for (int r = 0; r < m; ++r) {
    const T denom = sqrt(hessen[r][r] * hessen[r][r] +
			      hessen[r+1][r] * hessen[r+1][r]);
    if (denom == 0.0) {
      std::cerr << "zero devisor in givens rotation: " << r << ' ' << m << std::endl;
      exit (1);
    }
    const T ci = hessen[r][r] / denom;
    const T si = hessen[r+1][r] / denom;

    // multiple from left side
    // i th and i+1 th row
    std::vector<T> ith, ith1;
    ith.resize(m);      ith1.resize(m);
    for (int c = r; c < m; ++c) {
      ith[c] = ci * hessen[r][c] + si * hessen[r+1][c];
      ith1[c] = -si * hessen[r][c] + ci * hessen[r+1][c];
    }
    for (int c = r; c < m; ++c) {
      hessen[r][c] = ith[c];	hessen[r+1][c] = ith1[c];
    }

    // update be
    const T bei = ci * be[r] + si * be[r+1];
    const T bei1 = -si * be[r] + ci * be[r+1];
    be[r] = bei;
    be[r+1] = bei1;
    
    res.push_back(fabs(bei1));
  }

  // now solve upper triangular system
  hessen.resize(m);
  be.resize(m);
  
  Cvec<T> ym;
  solveUT(hessen, ym, be);
  
  for (int i = 0; i < ym.size(); ++i)
    x += vs[i] * ym[i];
};

template <class T>
void GMRES(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
	   int m, const T rtol, const int maxit,
	   int& iter, std::vector<T>& res) {
  // Hessenberg
  // Since this is a dense matrix, I do not use CsparseMat<T> for it.
  // Accordingly, I also implemented matrix multiplications and
  // solveUT for dense matrices.
  const T normb = b.norm();
  
  std::vector<std::vector<T> > hessen;
  std::vector<Cvec<T> > vs;
  while ((int)res.size() < maxit) {
    // initialization
    hessen.resize(m+1);
    for (int y = 0; y < m+1; ++y) {
      hessen[y].resize(m);
      for (int x = 0; x < m; ++x)
	hessen[y][x] = 0.0;
    }
    vs.clear();
    
    const Cvec<T> r = b - A * x;
    const T beta = r.norm();
    if (beta == 0.0)
      break;
    Cvec<T> v = r / beta;

    int finish = 0;
    for (int j = 0; j < m; ++j) {
      vs.push_back(v);
      
      Cvec<T> w = A * v;
      for (int i = 0; i <= j; ++i) {
	const T hij = w * vs[i];
	w -= hij * vs[i];
	hessen[i][j] = hij;
      }

      const T hj1j = w.norm();
      hessen[j+1][j] = hj1j;
      if (hj1j == 0.0) {
	m = j + 1;
	hessen.resize(m + 1);
	for (int y = 0; y < m+1; ++y)
	  hessen[y].resize(m);
	finish = 1;
	break;
      }

      if (hj1j == 0.0)
	break;
      v = w / hj1j;
    }

    // solve linear system and update x
    GMRESSub(x, m, res, hessen, vs, beta);

    // exceptional case
    if (finish)
      break;

    if (res.empty() || res[(int)res.size() - 1] / normb < rtol)
      break;
  }

  for (iter = 0; iter < (int)res.size(); ++iter)
    if (res[iter] / normb < rtol) {
      iter++;
      break;
    }
};

template <class T>
void CG(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
	const T rtol, const int maxit, int& iter,
	std::vector<T>& res) {
  Cvec<T> r = b - A * x;
  Cvec<T> p = r;
  Cvec<T> ap;

  const T normb = b.norm();
  const T threshold = rtol * normb;
  T oldrr = r * r;
  
  for (iter = 0; iter < maxit; ++iter) {
    //???? Outputting A norm
    Cvec<T> ans;
    ans.resize(x.size);
    for (int i = 0; i < x.size(); ++i)
      ans[i] = 1.0 / A.get(i, i);    
    ans -= x;
    std::cerr << sqrt((A * ans) * ans) << ' ';
    //????
    
    const T residual = sqrt(oldrr);
    res.push_back(residual);
    if (residual < threshold)
      break;
    
    ap = A * p;
    const T app = ap * p;
    if (app == 0.0 || oldrr == 0.0)
      break;
    
    const T alpha = oldrr / app;

    x += alpha * p;
    r -= alpha * ap;

    const T newrr = r * r;
    const T beta = newrr / oldrr;
    p = r + beta * p;
    oldrr = newrr;
  }


  //???? Outputting A norm  
  Cvec<T> ans;
  ans.resize(x.size());
  for (int i = 0; i < x.size(); ++i)
    ans[i] = 1.0 / A.get(i, i);    
  ans -= x;
  std::cerr << sqrt((A * ans) * ans) << std::endl;
  //????
};

//======================================================================
// Preconditioning related
//======================================================================
// with preconditioner
template <class T>
void GMRESILU(const CsparseMat<T>& A, const Cvec<T>& b, Cvec<T>& x,
	      int m, const T rtol, const int maxit,
	      int& iter, std::vector<T>& res) {
  CsparseMat<T> LDU;
  ILU0(A, LDU);

  // Hessenberg
  // Since this is a dense matrix, I do not use CsparseMat<T> for it.
  // Accordingly, I also implemented matrix multiplications and
  // solveUT for dense matrices.
  const T normb = b.norm();
  
  std::vector<std::vector<T> > hessen;
  std::vector<Cvec<T> > vs;
  while ((int)res.size() < maxit) {
    // initialization
    hessen.resize(m+1);
    for (int y = 0; y < m+1; ++y) {
      hessen[y].resize(m);
      for (int x = 0; x < m; ++x)
	hessen[y][x] = 0.0;
    }
    vs.clear();

    const Cvec<T> rtmp = b - A * x;
    Cvec<T> r;    r.resize((int)rtmp.size());
    solveLDU(LDU, r, rtmp);
    
    const T beta = r.norm();
    if (beta == 0.0)
      break;
    Cvec<T> v = r / beta;

    int finish = 0;
    for (int j = 0; j < m; ++j) {
      vs.push_back(v);
      
      const Cvec<T> wtmp = A * v;
      Cvec<T> w;      w.resize((int)wtmp.size());
      solveLDU(LDU, w, wtmp);
      
      for (int i = 0; i <= j; ++i) {
	const T hij = w * vs[i];
	w -= hij * vs[i];
	hessen[i][j] = hij;
      }

      const T hj1j = w.norm();
      hessen[j+1][j] = hj1j;
      if (hj1j == 0.0) {
	m = j + 1;
	hessen.resize(m + 1);
	for (int y = 0; y < m+1; ++y)
	  hessen[y].resize(m);
	finish = 1;
	break;
      }

      if (hj1j == 0.0)
	break;
      v = w / hj1j;
    }

    // solve linear system and update x
    GMRESSub(x, m, res, hessen, vs, beta);

    // exceptional case
    if (finish)
      break;

    if (res.empty() || res[(int)res.size() - 1] / normb < rtol)
      break;
  }

  for (iter = 0; iter < (int)res.size(); ++iter)
    if (res[iter] / normb < rtol) {
      iter++;
      break;
    }
};

// solve LD^{-1}U x = b
// x = U^{-1} D L^{-1} b
template <class T>
void solveLDU(const CsparseMat<T>& LDU, Cvec<T>& x,
	      const Cvec<T>& b) {
  // solve L x = b
  solveLT(LDU, x, b);
  // solve D^{-1} y = x
  Cvec<T> y;
  y.resize((int)x.size());
  multD(LDU, y, x);
  // solve U x = y
  solveUT(LDU, x, y);
};

template <class T>
void ILU0(const CsparseMat<T>& A, CsparseMat<T>& LDU) {
  LDU = A;

  const int n = LDU.getRow();
  if (n != LDU.getCol()) {
    std::cerr << "Not a square matrix: " << LDU.getRow() << ' '
	      << LDU.getCol() << std::endl;
    exit (1);
  }
    
  for (int i = 1; i < n; ++i) {
    const int begin = LDU.m_rowind[i];
    const int end = LDU.m_rowind[i+1];

    for (int k = begin; k < end; ++k) {
      const int ktmp = LDU.m_colind[k];
      if (i <= ktmp)
	break;
      LDU.m_val[k] /= LDU.get(ktmp, ktmp);
      
      for (int j = k+1; k < end; ++k) {
	const int jtmp = LDU.m_colind[j];
	LDU.m_val[j] -= LDU.m_val[k] * LDU.get(ktmp, jtmp);
      }
    }
  }
};

#endif // SPARSEMATSOLVE_H
