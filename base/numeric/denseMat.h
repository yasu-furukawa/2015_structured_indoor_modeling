#ifndef NUMERIC_DENSEMAT_H
#define NUMERIC_DENSEMAT_H

#include <iostream>
#include <vector>
#include "vec.h"
#include "mat.h"
#include "sparseMat.h"

template <class T>
class CdenseMat: public Cmat {
 public:
  CdenseMat(void);
  CdenseMat(const int row, const int col);
  CdenseMat(const CdenseMat& rhs);
  CdenseMat(const int row, const int col, const std::vector<T>& val);
  CdenseMat(const std::vector<std::vector<T> >& val);
  CdenseMat(const CsparseMat<T>& rhs);

  virtual ~CdenseMat();
  // Just allocate.
  void resize(const int row, const int col);
  // Allocate and set all the values to zero
  void init(const int row, const int col);
  void init(const int row, const int col, const std::vector<T>& val);
  void init(const std::vector<std::vector<T> >& val);
  void init(const CsparseMat<T>& rhs);
  
  void dealloc(void);

  // Add row, column
  void addRow(void);
  void addCol(void);
  void addRow(const Cvec<T>& row);
  void addCol(const Cvec<T>& col);

  Cvec<T>& operator[](const int row);
  const Cvec<T>& operator[](const int row) const;

  //======================================================================
  // Linear algebra
  CdenseMat& operator=(const CdenseMat& rhs);
  
  CdenseMat& operator+=(const CdenseMat& rhs);
  CdenseMat operator+(const CdenseMat& rhs) const;

  CdenseMat& operator-=(const CdenseMat& rhs);
  CdenseMat operator-(const CdenseMat& rhs) const;

  CdenseMat& operator*=(const CdenseMat& rhs);
  CdenseMat operator*(const CdenseMat& rhs) const;

  CdenseMat& operator*=(const T a);
  CdenseMat operator*(const T a) const;

  Cvec<T> operator*(const Cvec<T>& rhs) const;

  CdenseMat& operator/=(const T a);
  CdenseMat operator/(const T a) const;
  
  void transpose(void);

 protected:
  void setRowColumn(void);
  
  std::vector<Cvec<T> > m_val;

  template <class T2>
    friend std::istream& operator>>(std::istream& istr, CdenseMat<T2>& lhs);
  template <class T2>
    friend std::ostream& operator<<(std::ostream& ostr, const CdenseMat<T2>& lhs);  
};

template <class T>
void CdenseMat<T>::setRowColumn(void) {
  m_row = (int)m_val.size();
  if (m_row == 0) {
    m_col = 0;
    return;
  }
  m_col = (int)m_val[0].size();
};

template <class T>
CdenseMat<T>::CdenseMat(void) : Cmat() {
};

template <class T>
CdenseMat<T>::CdenseMat(const int row, const int col) : Cmat(row, col) {
  resize(row, col);
};

template <class T>
CdenseMat<T>::CdenseMat(const CdenseMat& rhs) : Cmat(rhs.m_row, rhs.m_col) {
  m_val = rhs.m_val;
};

template <class T>
CdenseMat<T>::CdenseMat(const int row, const int col, const std::vector<T>& val) :
  Cmat(row, col) {
  init(row, col, val);
};

template <class T>
CdenseMat<T>::CdenseMat(const std::vector<std::vector<T> >& val) {
  init(val);
};

template <class T>
CdenseMat<T>::CdenseMat(const CsparseMat<T>& rhs) {
  init(rhs);
};

template <class T>
CdenseMat<T>::~CdenseMat() {
};

template <class T>
void CdenseMat<T>::resize(const int row, const int col) {
  m_row = row;
  m_col = col;
  m_val.resize(row);
  for (int y = 0; y < row; ++y)
    m_val[y].resize(col);
};

template <class T>
void CdenseMat<T>::init(const int row, const int col) {
  m_row = row;
  m_col = col;
  m_val.resize(row);
  for (int y = 0; y < row; ++y)
    m_val[y].init(col);
};

template <class T>
void CdenseMat<T>::init(const int row, const int col, const std::vector<T> &val) {
  resize(row, col);
  int count = 0;
  for (int y = 0; y < row; ++y)
    for (int x = 0; x < col; ++x)
      m_val[y][x] = val[count++];  
};

template <class T>
void CdenseMat<T>::init(const std::vector<std::vector<T> > &val) {
  m_row = (int)val.size();
  if (m_row == 0)
    m_col = 0;
  else
    m_col = (int)val[0].size();
  resize(m_row, m_col);
  for (int y = 0; y < m_row; ++y)
    for (int x = 0; x < m_col; ++x)
      m_val[y][x] = val[y][x];
};

template <class T>
void CdenseMat<T>::init(const CsparseMat<T>& rhs) {
  m_row = rhs.getRow();
  m_col = rhs.getCol();
  init(m_row, m_col);

  for (int r = 0; r < m_row; ++r) {
    const int rbegin = rhs.m_rowind[r];
    const int rend = rhs.m_rowind[r + 1];
    for (int i = rbegin; i < rend; ++i) {
      m_val[r][rhs.m_colind[i]] = rhs.m_val[i];
    }
  }
};

template <class T>
void CdenseMat<T>::dealloc(void) {
  m_row = 0;
  m_col = 0;
  std::vector<Cvec<T> >().swap(m_val);
};

template <class T>
Cvec<T>& CdenseMat<T>::operator[](const int row) {
  return m_val[row];
};

template <class T>
const Cvec<T>& CdenseMat<T>::operator[](const int row) const{
  return m_val[row];
};

//======================================================================
template <class T>
CdenseMat<T>& CdenseMat<T>::operator=(const CdenseMat<T>& rhs) {
  if (this == &rhs)
    return;

  m_row = rhs.m_row;
  m_col = rhs.m_col;
  m_val = rhs.m_val;
  return *this;
};

template <class T>
CdenseMat<T>& CdenseMat<T>::operator+=(const CdenseMat<T>& rhs) {
  if (m_row != rhs.m_row || m_col != rhs.m_col) {
    std::cerr << "Matrix size incompatible" << std::endl;    exit (1);
  }
  for (int y = 0; y < m_row; ++y)
    m_val[y] += rhs.m_val[y];
  return *this;
};

template <class T>
CdenseMat<T> CdenseMat<T>::operator+(const CdenseMat<T>& rhs) const {
  return CdenseMat<T>(*this) += rhs;
};

template <class T>
CdenseMat<T>& CdenseMat<T>::operator-=(const CdenseMat<T>& rhs) {
  if (m_row != rhs.m_row || m_col != rhs.m_col) {
    std::cerr << "Matrix size incompatible" << std::endl;    exit (1);
  }
  for (int y = 0; y < m_row; ++y)
    m_val[y] -= rhs.m_val[y];
  return *this;
};

template <class T>
CdenseMat<T> CdenseMat<T>::operator-(const CdenseMat<T>& rhs) const {
  return CdenseMat<T>(*this) -= rhs;
};

template <class T>
CdenseMat<T>& CdenseMat<T>::operator*=(const CdenseMat& rhs) {
  if (m_col != rhs.m_row) {
    std::cerr << "Matrix size incompatible" << std::endl;    exit (1);
  }
  CdenseMat<T> ans;
  ans.init(m_row, rhs.m_col);
  for (int y = 0; y < ans.m_row; ++y)
    for (int x = 0; x < ans.m_col; ++x)
      for (int i = 0; i < m_col; ++i)
	ans[y][x] += m_val[y][i] * rhs[i][x];
  m_row = ans.m_row;
  m_col = ans.m_col;
  m_val.swap(ans.m_val);
  return *this;
};

template <class T>
CdenseMat<T> CdenseMat<T>::operator*(const CdenseMat& rhs) const {
  return CdenseMat<T>(*this) *= rhs;
};

template <class T>
CdenseMat<T>& CdenseMat<T>::operator*=(const T a) {
  for (int y = 0; y < m_row; ++y)
    m_val[y] *= a;
  return *this;
};

template <class T>
CdenseMat<T> CdenseMat<T>::operator*(const T a) const {
  return CdenseMat<T>(*this) *= a;
};

template <class T>
Cvec<T> CdenseMat<T>::operator*(const Cvec<T>& rhs) const {
  if (m_col != rhs.size()) {
    std::cerr << "Incompatible size" << std::endl;    exit (1);
  }
  Cvec<T> ans;
  ans.init(m_row);
  for (int y = 0; y < m_row; ++y)
    ans[y] = m_val[y] * rhs;
  return ans;
};
  
template <class T>
CdenseMat<T>& CdenseMat<T>::operator/=(const T a) {
  for (int y = 0; y < m_row; ++y)
    m_val[y] /= a;
  return *this;
};

template <class T>
CdenseMat<T> CdenseMat<T>::operator/(const T a) const {
  return CdenseMat<T>(*this) /= a;
};
  
template <class T>
void CdenseMat<T>::transpose(void) {
  const int itmp = m_col;
  m_col = m_row;
  m_row = itmp;

  std::vector<Cvec<T> > newval;
  newval.resize(m_row);
  for (int y = 0; y < m_row; ++y) {
    newval[y].resize(m_col);
    for (int x = 0; x < m_col; ++x)
      newval[y][x] = m_val[x][y];
  }

  m_val.swap(newval);
};

template <class T>
std::istream& operator>>(std::istream& istr, CdenseMat<T>& lhs) {
  std::string header;
  istr >> header;
  if (header != "DENSEMAT") {
    std::cerr << "Invalid header: " << header << std::endl;
    exit (1);
  }
  istr >> lhs.m_row >> lhs.m_col;
  
  for (int y = 0; y < lhs.m_row; ++y)
    for (int x = 0; x < lhs.m_col; ++x)
      istr >> lhs.m_val[y][x];
  return istr;
};

template <class T>
std::ostream& operator<<(std::ostream& ostr, const CdenseMat<T>& lhs) {
  ostr << "DENSEMAT" << std::endl
       << lhs.m_row << ' ' << lhs.m_col << std::endl;
  for (int y = 0; y < lhs.m_row; ++y) {
    for (int x = 0; x < lhs.m_col; ++x)
      ostr << lhs.m_val[y][x] << ' ';
    ostr << std::endl;
  }
  return ostr;
};

template <class T>
void CdenseMat<T>::addRow(void) {
  m_row++;
  m_val.resize((int)m_val.size() + 1);
};

template <class T>
void CdenseMat<T>::addRow(const Cvec<T>& row) {
  if (getCol() != (int)row.size()) {
    std::cerr << "Invalid new row size: " << getRow() << ' ' << row.size() << std::endl;
    exit (1);    
  }
  m_row++;
  m_val.push_back(row);
};

template <class T>
void CdenseMat<T>::addCol(void) {
  m_col++;
  for (int y = 0; y < m_row; ++y)
    m_val[y].resize(m_col);
};

template <class T>
void CdenseMat<T>::addCol(const Cvec<T>& col) {
  m_col++;
  for (int y = 0; y < m_row; ++y)
    m_val[y].push_back(col[y]);
};

#endif // NUMERIC_DENSEMAT_H
