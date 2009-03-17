#include "Face.h"
#include "Matrix3.h"

using namespace Spatial::Math;

Face::Face( const Vector3& pointA, const Vector3& pointB, const Vector3& pointC ) : 
  pointA(pointA), pointB(pointB), pointC(pointC) {

  double det = Matrix3(
		       pointA.x, pointA.y, pointA.z,
		       pointB.x, pointB.y, pointB.z,
		       pointC.x, pointC.y, pointC.z
		       ).determinant( );
  this->direction = ( det >= 0) ? Face::COUNTER_CLOCK_WISE : Face::CLOCK_WISE;
}
	
const Vector3 Face::getNormal( void ) const {
  return getPlane( ).normal;
}

Face::POLYGON_DIRECTION Face::getPolygonDirection( void ) const {
  return direction;
}
	
Plane Face::getPlane( ) const {
  Plane plane( pointA, pointB, pointC);
  plane.distanceFromOrigo = -plane.distanceFromOrigo;
  return plane;
}

Face& Face::addSelf( const Vector3& vector ) {
  this->pointA += vector;
  this->pointB += vector;
  this->pointC += vector;
  return *this;
}

std::string Face::toString( void ) const {
  return pointA.toString() + "|" + pointB.toString() + "|" +pointC.toString();
}

