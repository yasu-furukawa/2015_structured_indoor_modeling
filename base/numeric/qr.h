#ifndef NUMERIC_QR
#define NUMERIC_QR

#include <numeric/mat3.h>
#include <numeric/mat4.h>

extern "C" {
#include <clapack/f2c.h>
#include <clapack/clapack.h>
};

//======================================================================
// For 3x3
//======================================================================
// A = QL
template<class T>
void QL(const TMat3<T>& A, TMat3<T>& Q, TMat3<T>& L) {
  integer m = 3;      integer n = 3;
  integer lda = 3;    doublereal a[9];
  int count = 0;
  for (int x = 0; x < 3; ++x)
    for (int y = 0; y < 3; ++y)
      a[count++] = A[y][x];
  doublereal tau[3], work[3];
  integer info;      
  dgeql2_(&m, &n, a, &lda, tau, work, &info);

  // Q = H2 H1 H0
  TVec3<T> v0(1.0, 0.0, 0.0);
  TVec3<T> v1(a[3], 1.0, 0.0);
  TVec3<T> v2(a[6], a[7], 1.0);
  
  TMat3<T> H0 = TMat3<T>::I() - tau[0] * TMat3<T>::outer_product(v0);
  TMat3<T> H1 = TMat3<T>::I() - tau[1] * TMat3<T>::outer_product(v1);
  TMat3<T> H2 = TMat3<T>::I() - tau[2] * TMat3<T>::outer_product(v2);
  
  Q = H2 * H1 * H0;
  TMat3<T> QT = transpose(Q);
  L = QT * A;
};

// A = RQ
// A^T = Q^T R^T = Q^T L
template<class T>
void RQ(const TMat3<T>& A, TMat3<T>& R, TMat3<T>& Q) {
  TMat3<T> NA = transpose(A);
  QL(NA, Q, R);
  Q = transpose(Q);
  R = transpose(R);
};

// A = QR
template<class T>
void QR(const TMat3<T>& A, TMat3<T>& Q, TMat3<T>& R) {
  integer m = 3;      integer n = 3;
  integer lda = 3;    doublereal a[9];
  int count = 0;
  for (int x = 0; x < 3; ++x)
    for (int y = 0; y < 3; ++y)
      a[count++] = A[y][x];
  doublereal tau[3], work[3];
  integer info;      
  dgeqr2_(&m, &n, a, &lda, tau, work, &info);

  // Q = H0 H1 H2
  TVec3<T> v0(1.0, a[1], a[2]);
  TVec3<T> v1(0.0, 1.0, a[5]);
  TVec3<T> v2(0.0, 0.0, 1.0);
  
  TMat3<T> H0 = TMat3<T>::I() - tau[0] * TMat3<T>::outer_product(v0);
  TMat3<T> H1 = TMat3<T>::I() - tau[1] * TMat3<T>::outer_product(v1);
  TMat3<T> H2 = TMat3<T>::I() - tau[2] * TMat3<T>::outer_product(v2);
  
  Q = H0 * H1 * H2;
  TMat3<T> QT = transpose(Q);
  R = QT * A;
};

// A = LQ
// A^T = Q^T L^T = Q^T R
template<class T>
void LQ(const TMat3<T>& A, TMat3<T>& L, TMat3<T>& Q) {
  TMat3<T> NA = transpose(A);
  QR(NA, Q, L);
  Q = transpose(Q);
  L = transpose(L);
};

//======================================================================
// For 4x4
//======================================================================
// A = QL
template<class T>
void QL(const TMat4<T>& A, TMat4<T>& Q, TMat4<T>& L) {
  integer m = 4;      integer n = 4;
  integer lda = 4;    doublereal a[16];
  int count = 0;
  for (int x = 0; x < 4; ++x)
    for (int y = 0; y < 4; ++y)
      a[count++] = A[y][x];
  doublereal tau[4], work[4];
  integer info;      
  dgeql2_(&m, &n, a, &lda, tau, work, &info);

  // Q = H3 H2 H1 H0
  TVec4<T> v0(1.0, 0.0, 0.0, 0.0);
  TVec4<T> v1(a[4], 1.0, 0.0, 0.0);
  TVec4<T> v2(a[8], a[9], 1.0, 0.0);
  TVec4<T> v3(a[12], a[13], a[14], 1.0);
  
  TMat4<T> H0 = TMat4<T>::I() - tau[0] * TMat4<T>::outer_product(v0);
  TMat4<T> H1 = TMat4<T>::I() - tau[1] * TMat4<T>::outer_product(v1);
  TMat4<T> H2 = TMat4<T>::I() - tau[2] * TMat4<T>::outer_product(v2);
  TMat4<T> H3 = TMat4<T>::I() - tau[3] * TMat4<T>::outer_product(v3);
  
  Q = H3 * H2 * H1 * H0;
  TMat4<T> QT = transpose(Q);
  L = QT * A;
};

// A = RQ
// A^T = Q^T R^T = Q^T L
template<class T>
void RQ(const TMat4<T>& A, TMat4<T>& R, TMat4<T>& Q) {
  TMat4<T> NA = transpose(A);
  QL(NA, Q, R);
  Q = transpose(Q);
  R = transpose(R);
};

// A = QR
template<class T>
void QR(const TMat4<T>& A, TMat4<T>& Q, TMat4<T>& R) {
  integer m = 4;      integer n = 4;
  integer lda = 4;    doublereal a[16];
  int count = 0;
  for (int x = 0; x < 4; ++x)
    for (int y = 0; y < 4; ++y)
      a[count++] = A[y][x];
  doublereal tau[4], work[4];
  integer info;      
  dgeqr2_(&m, &n, a, &lda, tau, work, &info);

  // Q = H0 H1 H2 H3
  TVec4<T> v0(1.0, a[1], a[2], a[3]);
  TVec4<T> v1(0.0, 1.0, a[6], a[7]);
  TVec4<T> v2(0.0, 0.0, 1.0, a[11]);
  TVec4<T> v3(0.0, 0.0, 0.0, 1.0);
  
  TMat4<T> H0 = TMat4<T>::I() - tau[0] * TMat4<T>::outer_product(v0);
  TMat4<T> H1 = TMat4<T>::I() - tau[1] * TMat4<T>::outer_product(v1);
  TMat4<T> H2 = TMat4<T>::I() - tau[2] * TMat4<T>::outer_product(v2);
  TMat4<T> H3 = TMat4<T>::I() - tau[3] * TMat4<T>::outer_product(v3);
  
  Q = H0 * H1 * H2 * H3;
  TMat4<T> QT = transpose(Q);
  R = QT * A;
};

// A = LQ
// A^T = Q^T L^T = Q^T R
template<class T>
void LQ(const TMat4<T>& A, TMat4<T>& L, TMat4<T>& Q) {
  TMat4<T> NA = transpose(A);
  QR(NA, Q, L);
  Q = transpose(Q);
  L = transpose(L);
};

#endif
