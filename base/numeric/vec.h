#ifndef NUMERIC_VEC_H
#define NUMERIC_VEC_H

#include <iostream>
#include <vector>

template<class T>
class Cvec {
 public:
  // constructors. elements are initialized to 0 if any
  Cvec(void);
  Cvec(const int n);
  Cvec(const Cvec& lhs);
  // destructor
  virtual ~Cvec();

  // copying function
  Cvec& operator=(const Cvec& lhs);

  void init(const int n);

  void unitize(void);

  T norm(void) const;
  // resize vector
  void resize(const int n);
  // free all the memory
  void dealloc(void);
  // Add one more element
  void push_back(const T& rhs);
  
  //----------------------------------------------------------------------
  // print out vector
  //----------------------------------------------------------------------
  void print(std::ostream& ostr) const;
  
  // set each element to 0
  void setzero(void);

  //======================================================================
  // algebra
  //======================================================================
  inline Cvec& operator+=(const Cvec& lhs);
  inline Cvec& operator-=(const Cvec& lhs);
  inline Cvec& operator*=(const T s);
  inline Cvec& operator/=(const T s);
  inline T operator*(const Cvec& rhs) const;

  /*
  template <class T2>
    friend Cvec<T2> operator+(const Cvec<T2>& lhs, const Cvec<T2>& rhs);
  template <class T2>
    friend Cvec<T2> operator-(const Cvec<T2>& lhs, const Cvec<T2>& rhs);
  template <class T2>
    friend Cvec<T2> operator*(const Cvec<T2>& lhs, const T2 s);
  template <class T2>
    friend Cvec<T2> operator*(const T2 s, const Cvec<T2>& lhs);
  template <class T2>
    friend Cvec<T2> operator/(const Cvec<T2>& lhs, const T2 s);
  */
  //======================================================================
  
  // accessing methods
  inline T& operator[](const int index);
  inline const T& operator[](const int index) const;
  inline int size(void) const;
  
  //----------------------------------------------------------------------
  // protected member variables
 protected:
  // size of the vector
  int m_n;
  // contents
  std::vector<T> m_val;

  template <class T2>
    friend std::istream& operator>>(std::istream& istr, Cvec<T2>& lhs);
  template <class T2>
    friend std::ostream& operator<<(std::ostream& ostr, const Cvec<T2>& lhs);
};

//======================================================================
// Implementation of inline functions
//======================================================================
template <class T>
inline T& Cvec<T>::operator[](const int index) {
  return m_val[index];
};

template <class T>
inline const T& Cvec<T>::operator[](const int index) const{
  return m_val[index];
};

template <class T>
inline int Cvec<T>::size(void) const {
  return m_n;
};

template <class T>
inline Cvec<T>& Cvec<T>::operator+=(const Cvec<T>& lhs) {
  for (int i = 0; i < m_n; ++i)
    m_val[i] += lhs.m_val[i];
  return *this;
};

template <class T>
inline Cvec<T>& Cvec<T>::operator-=(const Cvec<T>& lhs) {
  for (int i = 0; i < m_n; ++i)
    m_val[i] -= lhs.m_val[i];
  return *this;
};

template <class T>
inline Cvec<T>& Cvec<T>::operator*=(const T s) {
  for (int i = 0; i < m_n; ++i)
    m_val[i] *= s;
  return *this;
};

template <class T>
inline Cvec<T>& Cvec<T>::operator/=(const T s) {
  for (int i = 0; i < m_n; ++i)
    m_val[i] /= s;
  return *this;
};

template <class T>
inline T Cvec<T>::operator*(const Cvec<T>& rhs) const{
  T dtmp = 0.0;
  for (int i = 0; i < m_n; ++i)
    dtmp += m_val[i] * rhs.m_val[i];
  return dtmp;
};

template <class T>
inline Cvec<T> operator+(const Cvec<T>& lhs, const Cvec<T>& rhs) {
  return Cvec<T>(lhs) += rhs;
};

template <class T>
inline Cvec<T> operator-(const Cvec<T>& lhs, const Cvec<T>& rhs) {
  return Cvec<T>(lhs) -= rhs;
};

template <class T>
inline Cvec<T> operator*(const Cvec<T>& lhs, const T s) {
  return Cvec<T>(lhs) *= s;
};

template <class T>
inline Cvec<T> operator*(const T s, const Cvec<T>& lhs) {
  return Cvec<T>(lhs) *= s;
};

template <class T>
inline Cvec<T> operator/(const Cvec<T>& lhs, const T s) {
  return Cvec<T>(lhs) /= s;
};

template <class T>
void Cvec<T>::push_back(const T& rhs) {
  m_n++;
  m_val.push_back(rhs);
};

// basic constructor which does nothing
template <class T>
Cvec<T>::Cvec(void) {
  m_n = 0;
};

// constructor which allocates memory and initializes each element by 0
template <class T>
Cvec<T>::Cvec(const int n) {
  m_n = n;

  m_val.resize(m_n);
  setzero();
};

// copy constructor
template <class T>
Cvec<T>::Cvec(const Cvec<T>& lhs) {
  m_n = lhs.m_n;

  m_val.resize(m_n);
  
  for (int i = 0; i < lhs.m_n; ++i)
    m_val[i] = lhs.m_val[i];
};

template <class T>
Cvec<T>::~Cvec() {
};

template <class T>
Cvec<T>& Cvec<T>::operator=(const Cvec<T>& lhs) {
  if (this == &lhs)
    return *this;

  m_n = lhs.m_n;
  m_val.resize(m_n);
  
  for (int i = 0; i < m_n; ++i)
    m_val[i] = lhs.m_val[i];
  
  return *this;
};

template <class T>
void Cvec<T>::init(const int n) {
  m_n = n;

  m_val.resize(m_n);
  setzero();
};

template <class T>
void Cvec<T>::unitize(void) {
  const T ntmp = norm();
  if (ntmp != 0.0)
    *this /= ntmp;
};

template <class T>
T Cvec<T>::norm(void) const {
  T dtmp = 0.0;
  for (int i = 0; i < m_n; ++i)
    dtmp += m_val[i] * m_val[i];
  return sqrt(dtmp);
};

template <class T>
void Cvec<T>::dealloc(void) {
  m_n = 0;
  std::vector<T>().swap(m_val);
};

template <class T>
void Cvec<T>::print(std::ostream& ostr) const {
  for (int i = 0; i < m_n; ++i)
    ostr << m_val[i] << ' ';
  ostr << std::endl;
};

template <class T>
void Cvec<T>::resize(const int n) {
  m_n = n;
  m_val.resize(n);
};

template <class T>
void Cvec<T>::setzero(void) {
  for (int i = 0; i < m_n; ++i)
    m_val[i] = 0.0;
};

//----------------------------------------------------------------------
// I/O functions
template <class T>
std::istream& operator>>(std::istream& istr, Cvec<T>& lhs) {
  for (int i = 0; i < lhs.m_n; ++i)
    istr >> lhs[i];
  return istr;
};

template <class T>
std::ostream& operator<<(std::ostream& ostr, const Cvec<T>& lhs) {
  for (int i = 0; i < lhs.m_n; ++i)
    ostr << lhs[i] << ' ';
  ostr << std::endl;
  return ostr;
};

#endif // VEC_H
