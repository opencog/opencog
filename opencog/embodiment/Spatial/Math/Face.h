#ifndef FACE_H
#define FACE_H

#include "Vector3.h"
#include "Plane.h"

namespace Spatial {
  namespace Math {

    class Face {
    public:

      /**  
       * 0+---+1 
       *  |  /
       *  | /
       *  |/
       *  +2
       *  Clockwise: 0, 1, 2
       *  Counterclockwise: 0, 2, 1
       */
      enum POLYGON_DIRECTION {
	CLOCK_WISE,
	COUNTER_CLOCK_WISE
      };

      /**
       * The three points of the face
       * @param pointA
       * @param pointB
       * @param pointC
       */
      Face( const Vector3& pointA, const Vector3& pointB, const Vector3& pointC );

      /**
       * Return the face normal. A perpendicular vertex pointing outside the face
       * @return Face Normal
       */
      const Vector3 getNormal( void ) const;

      /**
       * Get the direction of the vertices
       * @return
       */
      POLYGON_DIRECTION getPolygonDirection( void ) const;

      /**
       * Build a plane using the three face points
       * @return the builded plane
       */
      Plane getPlane( void ) const;

      /**
       * Translate the face points using the given vector
       * @param vector translation vector
       * @return self object
       */
      Face& addSelf( const Vector3& vector );

      /*
       *
       */
      std::string toString( void ) const;

      Vector3 pointA;
      Vector3 pointB;
      Vector3 pointC;

    private:
      
      POLYGON_DIRECTION direction;
      
    }; // Face

  }; // Math
}; // Spatial

#endif // FACE_H
