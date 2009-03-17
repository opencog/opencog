#ifndef SQUAREFACE_H
#define SQUAREFACE_H

#include "Face.h"
#include <vector>

namespace Spatial {
  namespace Math {

    class SquareFace : public Face {
    public:

      Vector3 pointD;
	
	/**
	 * Build a face using four points
	 * 
	 * @param pointA
	 * @param pointB
	 * @param pointC
	 * @param pointD
	 */
	SquareFace( Vector3 pointA, Vector3 pointB, Vector3 pointC, Vector3 pointD ) : Face( pointA, pointB, pointC ), pointD( pointD )  {

	}
	
	/* (non-Javadoc)
	 * @see com.vettalabs.petaverse.math.Face#addSelf(com.vettalabs.petaverse.math.Vector3)
	 */
	Face& addSelf( const Vector3& vector ) {
	  Face::addSelf( vector );
	  this->pointD + vector;
	  return *this;
	}

	/**
	 * Get the edges of the face
	 *    A+-----+B
	 *    /     /
	 *   /     /
	 * D+-----+C
	 * 
	 * @return
	 */
	std::vector<LineSegment> getAllEdges( void ) {
	  std::vector<LineSegment> edges;
	  edges.push_back( LineSegment( pointA, pointB ) );
	  edges.push_back( LineSegment( pointB, pointC ) );
	  edges.push_back( LineSegment( pointC, pointD ) );
	  edges.push_back( LineSegment( pointD, pointA ) );
	  return edges;
	}
	
	/* (non-Javadoc)
	 * @see com.vettalabs.petaverse.math.Face#toString()
	 */
	std::string toString( void ) const {
	  return Face::toString( ) + "|" + pointD.toString();
	}


    }; // SquareFace

  }; // Math
}; // Spatial

#endif // SQUAREFACE_H
