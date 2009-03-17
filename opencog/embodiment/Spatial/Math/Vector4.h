#ifndef VECTOR4_H
#define VECTOR4_H

#include "Vector3.h"

namespace Spatial {
  namespace Math {

    class Vector4 {
    public:
      double x;
      double y;
      double z;
      double w;
      
      /**
       * Build a vector from four coords (x,y,z,w)
       * @param x
       * @param y
       * @param z
       * @param w
       */
      inline Vector4( double x, double y, double z, double w ) : x(x), y(y), z(z), w(w) {
      }
	
      /**
       * Simple constructor
       */
      inline Vector4( void ) : x(0), y(0), z(0), w(0) {
      }
	
      /**
       * Copy constructor
       * @param vector
       */
      inline Vector4( const Vector4& vector ) {
	*this = vector;
      }
	
      /**
       * Build a 4 dimensional vector from a 3 dimensional plus another coord
       * @param vector
       * @param w
       */
      inline Vector4( const Vector3& vector, double w ) : x(vector.x), y(vector.y), z(vector.z), w(w) {
      }
	
      /**
       * Compute the dot product of this and a given 4 dimensional vector
       * @param vec
       * @return
       */
      inline double dot( const Vector4& vec ) const {
        return x * vec.x + y * vec.y + z * vec.z + w * vec.w;
      }
    
      /**
       * Compute the dot product of this and a given 3 dimensional vector
       * @param vec
       * @return
       */
      inline double dot( const Vector3& vec ) const {
        return x * vec.x + y * vec.y + z * vec.z + w;
      }
      
    }; // Vector4

  }; // Math
}; // Spatial

#endif // VECTOR4_H
