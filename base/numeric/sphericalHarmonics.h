#ifndef SPHERICALHARMONICS_H
#define SPHERICALHARMONICS_H

#include <cmath>
#include <iostream>
#include "euler.h"

class CsphericalHarmonics {
 public:
  static float gen(const int l, const int m,
		   const float x, const float y, const float z);
  
 protected:
};

float CsphericalHarmonics::gen(const int l, const int m,
			       const float x, const float y, const float z) {
  float theta, phi;
  Ceuler::getAngles(x, y, z, theta, phi);
  const float pi = M_PI;
  static float const root_pi = std::sqrt( pi );
	
  switch( l ) {
  case 0:
    {
      static float const a0 = 1.0 / ( 2.0 * root_pi );
      return a0;
    }
    break;
    
  case 1:
    {
      static float const a0 = std::sqrt( 3.0 ) / ( 2.0 * root_pi );
      
      float const ct = std::cos( theta );
      float const st = std::sin( theta );
      float const cp = std::cos( phi );
      float const sp = std::sin( phi );
      
      switch( m )
	{
	case -1:
	  return a0*sp*st;
	  
	case 0:
	  return a0*ct;
	  
	default: // case 1
	  return a0*cp*st;
	}
    }
    break;
    
  case 2:
    {
      static float const a0 = std::sqrt( 15.0 ) / ( 2.0 * root_pi );
      static float const a1 = std::sqrt( 5.0 ) / ( 4.0 * root_pi );
      static float const a2 = std::sqrt( 15.0 ) / ( 4.0 * root_pi );
      
      float const ct = std::cos( theta );
      float const st = std::sin( theta );
      float const cp = std::cos( phi );
      float const sp = std::sin( phi );
      
      switch( m )
	{
	case -2:
	  return a0*cp*sp*st*st;
	  
	case -1:
	  return a0*sp*ct*st;
	  
	case 0:
	  return a1*( 3.0*ct*ct - 1.0 );
	  
	case 1:
	  return a0*cp*ct*st;
	  
	default: // case 2
	  return a2*( cp*cp - sp*sp )*st*st;
	}
    }
    break;
    
  case 3:
    {
      static float const a0 = std::sqrt( 35.0 ) / ( 4.0 * std::sqrt( 2.0 * pi ) );
      static float const a1 = std::sqrt( 105.0 ) / ( 2.0 * root_pi );
      static float const a2 = std::sqrt( 21.0 ) / ( 4.0 * std::sqrt( 2.0 * pi ) );
      static float const a3 = std::sqrt( 7.0 ) / ( 4.0 * root_pi );
      static float const a4 = std::sqrt( 105.0 ) / ( 4.0 * root_pi );
      
      float const ct = std::cos( theta );
      float const st = std::sin( theta );
      float const cp = std::cos( phi );
      float const sp = std::sin( phi );
      
      switch( m )
	{
	case -3:
	  return a0*( 3.0*cp*cp - sp*sp )*sp*st*st*st;
	  
	case -2:
	  return a1*cp*sp*ct*st*st;
	  
	case -1:
	  return a2*( 5.0*ct*ct - 1.0 )*sp*st;
	  
	case 0:
	  return a3*( 5.0*ct*ct*ct - 3.0*ct );
	  
	case 1:
	  return a2*( 5.0*ct*ct - 1.0 )*cp*st;
	  
	case 2:
	  return a4*( cp*cp - sp*sp )*ct*st*st;
	  
	default: // case 3
	  return a0*( cp*cp - 3.0*sp*sp )*cp*st*st*st;
	}			
    }
    break;
    
  case 4:
    {
      static float const a0 = ( 3.0 * std::sqrt( 35.0 ) ) / ( 4.0 * root_pi );
      static float const a1 = ( 3.0 * std::sqrt( 35.0 ) ) / ( 4.0 * std::sqrt( 2.0 * pi ) );
      static float const a2 = ( 3.0 * std::sqrt( 5.0 ) ) / ( 4.0 * root_pi );
      static float const a3 = ( 3.0 * std::sqrt( 5.0 ) ) / ( 4.0 * std::sqrt( 2.0 * pi ) );
      static float const a4 = 3.0 / ( 16.0 * root_pi );
      static float const a5 = ( 3.0 * std::sqrt( 5.0 ) ) / ( 8.0 * root_pi );
      static float const a6 = ( 3.0 * std::sqrt( 35.0 ) ) / ( 16.0 * root_pi );
      
      float const ct = std::cos( theta );
      float const st = std::sin( theta );
      float const cp = std::cos( phi );
      float const sp = std::sin( phi );
      
      switch( m )
	{
	case -4:
	  return a0*( cp*cp - sp*sp )*cp*sp*st*st*st*st;
	  
	case -3:
	  return a1*( 3.0*cp*cp - sp*sp )*sp*ct*st*st*st;
	  
	case -2:
	  return a2*( 7.0*ct*ct - 1.0 )*cp*sp*st*st;
	  
	case -1:
	  return a3*( 7.0*ct*ct - 3.0 )*sp*ct*st;
	  
	case 0:
	  return a4*( 3.0 - 30.0*ct*ct + 35.0*ct*ct*ct*ct );
	  
	case 1:
	  return a3*( 7.0*ct*ct - 3.0 )*cp*ct*st;
	  
	case 2:
	  return a5*( 7.0*ct*ct - 1.0 )*( cp*cp - sp*sp )*st*st;
	  
	case 3:
	  return a1*( cp*cp - 3.0*sp*sp )*cp*ct*st*st*st;
	  
	default: // case 4
	  return a6*( sp*sp*sp*sp - 6.0*cp*cp*sp*sp + cp*cp*cp*cp )*st*st*st*st;
	}
    }
    break;
  default:
    return 0.0;
  }
};

/*
  float CsphericalHarmonics::gen(const int l, const int m,
			       const float x, const float y, const float z) {
  // Unscaled version
  if (l == 0)
    return 0.1f;
    //return 1.0f;
  else if (l == 1) {
    if (m == 0)
      return z;
    else if (m == 1)
      return x;
    else if (m == -1)
      return y;
    else {
      std::cerr << "Invalid l and m: " << l << ' ' << m << std::endl;
      exit (1);
    }
  }
  else if (l == 2) {
    if (m == 0)
      return 3 * z * z - 1.0;
    else if (m == 1)
      return x * z;
    else if (m == -1)
      return y * z;
    else if (m == 2)
      return x * x - y * y;
    else if (m == -2)
      return y * x;
    else {
      std::cerr << "Invalid l and m: " << l << ' ' << m << std::endl;
      exit (1);
    }
  }
  else if (l == 3) {
    if (m == 0)
      return 5 * z * z * z - 3 * z;
    else if (m == -1)
      return (5 * z * z - 1) * y;
    else if (m == 1)
      return (5 * z * z - 1) * x;
    else if (m == -2)
      return x * y * z;
    else if (m == 2)
      return (x * x - y * y) * z;
    else if (m == -3)
      return 3 * x * x * y - y * y * y;
    else if (m == 3)
      return x * x * x - 3 * x * y * y;
    else {
      std::cerr << "Invalid l and m: " << l << ' ' << m << std::endl;
      exit (1);
    }
  }
  else {
    std::cerr << "Invalid l: " << l << std::endl;
    exit (1);
  }
  
  return 0.0f;
};
*/

  // Care about scale
  /*
  if (l == 0)
    return 0.5 / sqrt(M_PI);
  else if (l == 1) {
    if (m == 0)
      return 0.5 * sqrt(3.0 / M_PI) * z;
    else if (m == 1)
      return 0.5 * sqrt(3.0 / M_PI) * x;
    else if (m == -1)
      return 0.5 * sqrt(3.0 / M_PI) * y;
    else {
      std::cerr << "Invalid l and m: " << l << ' ' << m << std::endl;
      exit (1);
    }
  }
  else if (l == 2) {
    if (m == 0)
      return 0.25 * sqrt(5.0 / M_PI) * (3 * z * z - 1.0);
    else if (m == 1)
      return 0.5 * sqrt(15.0 / M_PI) * x * z;
    else if (m == -1)
      return 0.5 * sqrt(15.0 / M_PI) * y * z;
    else if (m == -2)
      return 0.5 * sqrt(15.0 / M_PI) * y * x;
    else if (m == 2)
      return 0.5 * sqrt(15.0 / M_PI) * (x * x - y * y);
    else {
      std::cerr << "Invalid l and m: " << l << ' ' << m << std::endl;
      exit (1);
    }
  }
  else {
    std::cerr << "Invalid l: " << l << std::endl;
    exit (1);
  }
  */

#endif // SPHERICALHARMONICS_H
