#ifndef NUMERIC_SPARSEMAT_H
#define NUMERIC_SPARSEMAT_H

#include <iostream>
#include <vector>
#include <algorithm>
#include "vec.h"
#include "mat.h"

template <class T> class CcooMat;
template <class T> class CdenseMat;
//======================================================================
// CSR format matrix class
//======================================================================

template <class T>
class CsparseMat: public Cmat {
 public:
  CsparseMat(void);
  CsparseMat(const int row, const int col);
  CsparseMat(const CsparseMat& rhs);  
  // special way of initializing CsparseMat
  CsparseMat(const int row, const int col, const int nnz, const T* val,
	     const int* colind, const int* rowind);

  CsparseMat(const std::vector<std::vector<T> >& data);
  CsparseMat(const CdenseMat<T>& data);
  
  virtual ~CsparseMat();
  
  void init(const int row, const int col, const int nnz, const T* val,
	    const int* colind, const int* rowind);

  void init(const std::vector<std::vector<T> >& data);
  
  void dealloc(void);

  int getNNZ(void);

  //----------------------------------------------------------------------
  // get and set functions
  //----------------------------------------------------------------------
  T get(const int i, const int j) const;
  
  //----------------------------------------------------------------------
  // print out matrix in a full format (you'll see lots of zeros)
  //----------------------------------------------------------------------
  void print(std::ostream& ostr) const;
    
  //----------------------------------------------------------------------
  // member variables
  //----------------------------------------------------------------------
  std::vector<T> m_val;
  std::vector<int> m_colind;
  std::vector<int> m_rowind;

  //======================================================================
  // linear algebra
  // copying function
  CsparseMat& operator=(const CsparseMat& rhs);

  CsparseMat& operator+=(const CsparseMat& rhs);
  CsparseMat operator+(const CsparseMat& rhs) const;
  
  CsparseMat& operator-=(const CsparseMat& rhs);
  CsparseMat operator-(const CsparseMat& rhs) const;
  
  CsparseMat& operator*=(const T a);
  CsparseMat operator*(const T a) const;
  
  CsparseMat& operator/=(const T a);
  CsparseMat operator/(const T a) const;
  
  // mat vec
  Cvec<T> operator*(const Cvec<T>& vec) const;
  // mat mat
  CsparseMat& operator*=(const CsparseMat& rhs);
  CsparseMat operator*(const CsparseMat& rhs) const;

  //======================================================================
  // Friend funcitons
  template <class T2>
    friend CsparseMat<T2> operator*(const T2 a, const CsparseMat<T2>& lhs);
  // Extract diagonal
  template <class T2>
    void extractD(const CsparseMat<T2>& A, CsparseMat<T2>& D);
  // Extract strict lower
  template <class T2>
    void extractE(const CsparseMat<T2>& A, CsparseMat<T2>& E);
  // Extract strict upper
  template <class T2>
    void extractF(const CsparseMat<T2>& A, CsparseMat<T2>& F);
  // Solving lower triangular system. A does not have to be lower
  // triangular, we just use lower triangular part of A.
  template <class T2>
    void solveLT(const CsparseMat<T2>& A, Cvec<T2>& x, const Cvec<T2>& b);
  // Solving upper triangular system. A does not have to be upper
  // triangular, we just use upper triangular part of A.
  template <class T2>
    void solveUT(const CsparseMat<T2>& A, Cvec<T2>& x, const Cvec<T2>& b);
  // File conversion
  template <class T2>
    friend void coo2csr(const CcooMat<T2>& coomat, CsparseMat<T2>& csrmat);
};


//======================================================================
// Mat base class
//======================================================================
template <class T>
CsparseMat<T>::CsparseMat(void): Cmat() {
  m_rowind.push_back(0);
};

template <class T>
CsparseMat<T>::CsparseMat(const int row, const int col): Cmat(row, col) {
  for (int r = 0; r < m_row + 1; ++r)
    m_rowind.push_back(0);
};

template <class T>
CsparseMat<T>::CsparseMat(const CsparseMat<T>& rhs) {
  m_row = rhs.m_row;
  m_col = rhs.m_col;

  m_val = rhs.m_val;
  m_colind = rhs.m_colind;
  m_rowind = rhs.m_rowind;
};

template <class T>
CsparseMat<T>::CsparseMat(const int row, const int col, const int nnz, const T* val,
			  const int* colind, const int* rowind) {
  m_row = row;
  m_col = col;

  m_val.resize(nnz);
  m_colind.resize(nnz);
  m_rowind.resize(m_row + 1);

  for (int i = 0; i < nnz; ++i) {
    m_val[i] = val[i];
    m_colind[i] = colind[i];
  }

  for (int r = 0; r < m_row + 1; ++r)
    m_rowind[r] = rowind[r];
};

template <class T>
CsparseMat<T>::~CsparseMat() {
};

template<class T>
int CsparseMat<T>::getNNZ(void) {
  if (m_rowind.empty())
    return 0;
  else
    return m_rowind[(int)m_rowind.size() - 1];
}

template <class T>
CsparseMat<T>::CsparseMat(const std::vector<std::vector<T> >& data) {
  if (data.empty()) {
    std::cerr << "Cannot initialize CsparseMat" << std::endl;    exit (1);
  }
  
  m_row = (int)data.size();
  m_col = (int)data[0].size();

  m_rowind.push_back(0);
  for (int y = 0; y < m_row; ++y) {
    for (int x = 0; x < m_col; ++x) {
      if (data[y][x] == 0.0)
	continue;

      m_val.push_back(data[y][x]);
      m_colind.push_back(x);
    }
    m_rowind.push_back((int)m_colind.size());
  }
};

template <class T>
CsparseMat<T>::CsparseMat(const CdenseMat<T>& data) {
  m_row = data.getRow();
  m_col = data.getCol();

  m_rowind.push_back(0);
  for (int y = 0; y < m_row; ++y) {
    for (int x = 0; x < m_col; ++x) {
      if (data[y][x] == 0.0)
	continue;

      m_val.push_back(data[y][x]);
      m_colind.push_back(x);
    }
    m_rowind.push_back((int)m_colind.size());
  }
};

template <class T>
void CsparseMat<T>::init(const int row, const int col, const int nnz, const T* val,
			 const int* colind, const int* rowind) {
  m_val.clear();
  m_colind.clear();
  m_rowind.clear();
  
  m_row = row;
  m_col = col;
  
  m_val.resize(nnz);
  m_colind.resize(nnz);
  m_rowind.resize(m_row + 1);

  for (int i = 0; i < nnz; ++i) {
    m_val[i] = val[i];
    m_colind[i] = colind[i];
  }

  for (int r = 0; r < m_row + 1; ++r)
    m_rowind[r] = rowind[r];
};

template <class T>
void CsparseMat<T>::init(const std::vector<std::vector<T> >& data) {
  if (data.empty()) {
    std::cerr << "Cannot initialize CsparseMat" << std::endl;
    exit (1);
  }

  m_val.clear();
  m_colind.clear();
  m_rowind.clear();
  
  m_row = (int)data.size();
  m_col = (int)data[0].size();

  m_rowind.push_back(0);
  for (int y = 0; y < m_row; ++y) {
    for (int x = 0; x < m_col; ++x) {
      if (data[y][x] == 0.0)
	continue;

      m_val.push_back(data[y][x]);
      m_colind.push_back(x);
    }
    m_rowind.push_back((int)m_colind.size());
  }
};

template <class T>
void CsparseMat<T>::dealloc(void) {
  m_row = 0;  m_col = 0;
  
  std::vector<T>().swap(m_val);
  std::vector<int>().swap(m_colind);
  std::vector<int>().swap(m_rowind);
};

// print out matrix in a full format (you'll see lots of zeros)
template <class T>
void CsparseMat<T>::print(std::ostream& ostr) const {
  for (int y = 0; y < m_row; ++y) {
    const int begin = m_rowind[y];
    const int end = m_rowind[y+1];

    T dltmp[m_col];
    for (int i = 0; i < m_col; ++i)
      dltmp[i] = 0.0;
    for (int i = begin; i < end; ++i)
      dltmp[m_colind[i]] = m_val[i];
    
    for (int i = 0; i < m_col; ++i)
      ostr << dltmp[i] << ' ';
    ostr << std::endl;
  }
};

//----------------------------------------------------------------------
// O(log(nz_in_a_row)) to get a value. Most efficient possible.
template <class T>
T CsparseMat<T>::get(const int i, const int j) const {
  // using stl. simple implementation
  std::vector<int>::const_iterator pos = lower_bound(m_colind.begin() + m_rowind[i],
						     m_colind.begin() + m_rowind[i+1],
						     j);
  if (pos == m_colind.begin() + m_rowind[i+1] || *pos != j)
    return 0.0;
  else
    return m_val[pos - m_colind.begin()];  
};

#include "sparseMatSolve.h"
#include "sparseMatAlgebra.h"

#endif // SPARSEMAT
