#ifndef PLANE_H
#define PLANE_H

#include <string>

#include "util/exceptions.h"

#include "Vector3.h"
#include "Matrix4.h"

namespace Spatial {
  namespace Math {

    class Plane {
    public:
      /**
       * Classifies the sides of the plane
       */
      enum SIDE {
	NEGATIVE,
	POSITIVE,
	INTERSECT
      };
      
      /**
       * Simple constructor
       */
      Plane( void );

      /**
       * Copy constructor
       * @param plane
       */
      Plane( const Plane& plane );

      /**
       * Use a normal and a distance from origin to build a plane
       * @param normal
       * @param distance
       */
      Plane( const Vector3& normal, double distance );

      /**
       * Build a plane using three arbitrary points
       * @param pointA
       * @param pointB
       * @param pointC
       */
      Plane( const Vector3& pointA, const Vector3& pointB, const Vector3& pointC );

      /**
       * Set new normal and distance values to this plane
       * @param normal
       * @param distance
       */
      void set( const Vector3& normal, double distance );

      /**
       * Get a 4 dimension vector that represents this plane
       * @return
       */
      Vector4 getVector4( void );

      /**
       * Get the distance between this plane and a given point
       * @param point
       * @return
       */
      double getDistance( const Vector3& point );

      /**
       * Identifies the side of the plane a given point is positioned 
       * @param point
       * @return
       */
      SIDE getSide( const Vector3& point );

      /**
       * Apply a transformation matrix to this plane
       * @param transformation
       */
      void transformSelf( const Matrix4& transformation );

      /**
       * Get the intersection point between this plane and other two
       * @param plane2
       * @param plane3
       * @return
       */
      Vector3 getIntersectionPoint( const Plane& plane2, const Plane& plane3 ) throw (opencog::NotFoundException);

      /*
       *
       */
      bool operator==( const Plane& other ) const;

      /*
       *
       */
      std::string toString(void) const;

      inline virtual ~Plane( void ) { }
      
      Vector3 normal;
      double distanceFromOrigo;      
    }; // Plane

  }; // Math
}; // Spatial

#endif // PLANE_H
