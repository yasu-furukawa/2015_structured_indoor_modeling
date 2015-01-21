#ifndef EULER_H
#define EULER_H

#include <cmath>

class Ceuler {
 public:
  template<class T>
    inline static void getAngles(const T& x, const T& y, const T& z, T& theta, T& phi);
}; 

template<class T>
inline void Ceuler::getAngles(const T& x, const T& y, const T& z, T& theta, T& phi) {
  theta = acos(z);
  const float sintheta = sin(theta);
  if (sintheta == 0.0) {
    phi = 0.0;
    return;
  }

  const float cosphi = max(-1.0f, min(1.0f, x / sintheta));
  const float sinphi = max(-1.0f, min(1.0f, y / sintheta));
  phi = atan2(cosphi, sinphi);
  if (phi < 0.0)
    phi += 2 * M_PI;
};

#endif // EULER_H
