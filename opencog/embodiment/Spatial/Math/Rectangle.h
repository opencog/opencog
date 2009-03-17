#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "Vector3.h"
#include <LADSUtil/exceptions.h>

namespace Spatial {
  namespace Math {

    class Rectangle {
    public:

      Rectangle( const Rectangle& other );

      Rectangle( const Vector3& leftTopCorner, const Vector3& rightTopCorner, const Vector3& rightBottomCorner ) throw(LADSUtil::InvalidParamException);

      inline virtual ~Rectangle( void ) { };

      bool isInside( const Vector3& point );

      Rectangle& operator=( const Rectangle& o );

      bool operator==( const Rectangle& o ) const;


    private:
	Vector3 leftTopCorner;
	Vector3 rightTopCorner;
	Vector3 rightBottomCorner;      
	Vector3 leftBottomCorner;      
    }; // Rectangle

  }; // Math
}; // Spatial

#endif // RECTANGLE_H
