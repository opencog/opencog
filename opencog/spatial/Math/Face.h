/*
 * opencog/spatial/Math/Face.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
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
