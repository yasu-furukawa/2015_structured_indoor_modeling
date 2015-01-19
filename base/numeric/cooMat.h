#ifndef NUMERIC_COOMAT_H
#define NUMERIC_COOMAT_H

#include <vector>
#include "mat.h"

template <class T>
class CcooMat: public Cmat {
 public:
  CcooMat(void);
  CcooMat(const int row, const int col);
  virtual ~CcooMat();

  void dealloc(void);

  //----------------------------------------------------------------------
  // get and set functions
  //----------------------------------------------------------------------
  T get(const int i, const int j) const;
  void set(const int i, const int j, const T rhs, const int nocheck = 0);

  //----------------------------------------------------------------------
  // member variables
  //----------------------------------------------------------------------
  std::vector<T> m_val;
  std::vector<int> m_rowind;
  std::vector<int> m_colind;
};

template <class T>
CcooMat<T>::CcooMat(void): Cmat() {
};

template <class T>
CcooMat<T>::CcooMat(const int row, const int col): Cmat(row, col) {
};

template <class T>
CcooMat<T>::~CcooMat() {
};

// deallocater
template <class T>
void CcooMat<T>::dealloc(void) {
  std::vector<T>().swap(m_val);
  std::vector<int>().swap(m_rowind);
  std::vector<int>().swap(m_colind);
  
  m_row = 0;  m_col = 0;
};

// set
template <class T>
T CcooMat<T>::get(const int i, const int j) const {
  for (int e = 0; e < m_val.size(); ++e) {
    if (m_rowind[e] == i && m_colind[e] == j)
      return m_val[e];
  }
  return 0.0;
};

template <class T>
void CcooMat<T>::set(const int i, const int j, const T rhs, const int nocheck) {
  // when setting 0
  const int nnz = (int)m_val.size();
  
  if (rhs == 0.0) {
    // if you find (i,j), remove it from the array
    for (int e = 0; e < nnz; ++e) {
      if (m_rowind[e] == i && m_colind[e] == j) {
	m_val[e] = m_val[nnz - 1];
	m_rowind[e] = m_rowind[nnz - 1];
	m_colind[e] = m_colind[nnz - 1];

	m_val.resize(nnz - 1);
	m_rowind.resize(nnz - 1);
	m_colind.resize(nnz - 1);

	return;
      }
    }
  }
  // when setting non-zero value
  else {
    if (nocheck == 0) {
      // when you find (i,j) overwrite the value
      for (int e = 0; e < nnz; ++e) {
	if (m_rowind[e] == i && m_colind[e] == j) {
	  m_val[e] = rhs;
	  return;
	}
      }
    }

    // if no (i,j), add it to the array
    m_val.push_back(rhs);
    m_rowind.push_back(i);
    m_colind.push_back(j);    
  }
};

template <class T>
void coo2csr(const CcooMat<T>& coomat, CsparseMat<T>& csrmat) {
  // free memory
  csrmat.dealloc();
    
  // copy element of the base matrix class
  const int row = coomat.m_row;
  const int col = coomat.m_col;
  const int nnz = (int)coomat.m_val.size();
  
  csrmat.m_row = row;  csrmat.m_col = col;

  // allocate memory
  csrmat.m_val.resize(nnz);
  csrmat.m_colind.resize(nnz);
  csrmat.m_rowind.resize(row + 1);

  // [table[0] table[1] ...] is equal to m_col
  std::vector<std::vector<int> > table;
  table.resize(row);
  for (int e = 0; e < nnz; ++e)
    table[coomat.m_rowind[e]].push_back(coomat.m_colind[e]);
  // sort each array
  for (int i = 0; i < (int)table.size(); ++i)
    sort(table[i].begin(), table[i].end());

  // set m_row
  csrmat.m_rowind[0] = 0;
  for (int i = 1; i < csrmat.m_row + 1; ++i)
    csrmat.m_rowind[i] = csrmat.m_rowind[i-1] + (int)table[i-1].size();
  
  // go through each element and set values
  for (int e = 0; e < nnz; ++e) {
    const int& row = coomat.m_rowind[e];
    const int offset = lower_bound(table[row].begin(), table[row].end(),
				   coomat.m_colind[e]) - table[row].begin();

    const int pos = csrmat.m_rowind[row] + offset;
    csrmat.m_val[pos] = coomat.m_val[e];
    csrmat.m_colind[pos] = coomat.m_colind[e];
  }
};

#endif // COOMAT_H
