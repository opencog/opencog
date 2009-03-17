#ifndef LINE_H
#define LINE_H

#include "Vector3.h"

namespace Spatial {
  namespace Math {
    /**
     * Represents a line on a 3d environment
     */
    class Line {
    public:
      
      /**
       * Use two points to determine the line
       * @param pointA
       * @param pointB
       */    
      inline Line( const Vector3& pointA, const Vector3& pointB ) :
	pointA( pointA ), pointB( pointB ) { }
      

      inline Line( const Line& line ) : 
	pointA( line.pointA ), pointB( line.pointB ) { }

      inline virtual ~Line( void ) { }
      
      /**
       * Return a vector starting on A and pointing to B.
       * This vector was built using the segment points informed to create this line 
       * @return 
       */
      inline Vector3 vectorPointingB( ) {
	return this->pointB - this->pointA;
      }
      
      /**
       * Return a vector starting on B and pointing to A.
       * This vector was built using the segment points informed to create this line  
       * @return
       */
      inline Vector3 vectorPointingA( ) {
	return this->pointA - this->pointB;
      }
      
      /** 
       * 
       */
      virtual inline bool operator==( const Line& line ) const {
	return ( ( this->pointA == line.pointA && pointB == line.pointB ) || 
		 ( this->pointA == line.pointB && pointB == line.pointA ) );
      }

      virtual inline Line& operator=( const Line& line ) {
	this->pointA = line.pointA;
	this->pointB = line.pointB;
	return *this;
      }

      
      /*
       *
       */
      inline std::string toString( void ) const {
	return this->pointA.toString() + " " + this->pointB.toString( );
      }


      inline bool intersects( const Line& line, Vector3* intersectionPoint = NULL ) const {

	// a1*x + b1*y + c1 = 0 is line 1
	double a1 = this->pointB.y - this->pointA.y;
	double b1 = this->pointA.x - this->pointB.x;
	double c1 = this->pointB.x*this->pointA.y - this->pointA.x*this->pointB.y;
	
	// a2*x + b2*y + c2 = 0 is line 2
	double a2 = line.pointB.y - line.pointA.y;
	double b2 = line.pointA.x - line.pointB.x;
	double c2 = line.pointB.x*line.pointA.y - line.pointA.x*line.pointB.y;
	
	double denominator = a1*b2 - a2*b1;

	if ( denominator <= 0.000001 ) {
	  return false;
	} // if
	
	if ( intersectionPoint != NULL ) {
	  intersectionPoint->x = (b1*c2 - b2*c1)/denominator;
	  intersectionPoint->y = (a2*c1 - a1*c2)/denominator;
	} // if

	return true;
      }
      
      Vector3 pointA;
      Vector3 pointB;        
    };
    
  }; // Spatial
}; // Math

#endif // LINE_H
