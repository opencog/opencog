#include "Entity.h"
#include <sstream>
#include <cstring>
#include <boost/variant/get.hpp>

using namespace Spatial;

Entity::Entity( const EntityPtr& entity ) : id(entity->id), name(entity->name), dimension(entity->dimension), position(entity->position), orientation(entity->orientation), expansionRadius(entity->expansionRadius), boundingBox(this) {
  this->properties.resize( Entity::NUMBER_OF_PROPERTIES );
}

Entity::Entity( long id, const std::string& name, const Math::Vector3& position, const Math::Dimension3& dimension, const Math::Quaternion& orientation, double radius ) : id(id), name(name), dimension(dimension), position(position), orientation(orientation), expansionRadius( radius ), boundingBox(this) {}

Math::Vector3 Entity::getDirection( void ) const {
  return orientation.rotate( Math::Vector3::X_UNIT );
}

double Entity::distanceTo( const EntityPtr& entity ) const {
		
  const Math::BoundingBox& bb1 = getBoundingBox( );
  const Math::BoundingBox& bb2 = entity->getBoundingBox( );
  
  unsigned int i;
  unsigned int j;

  // check for intersection. if it happens, distance is zero
  const std::vector<Math::Vector3>& corners1 = bb1.getAllCorners( );
  for ( i = 0; i < corners1.size(); ++i ) {
    if ( bb2.isInside(corners1[i]) ) {
      return 0.0;
    } // if
  } // for
  
  // check for intersection. if it happens, distance is zero
  const std::vector<Math::Vector3>& corners2 = bb2.getAllCorners( );
  for ( i = 0; i < corners2.size(); ++i ) {
    if ( bb1.isInside(corners2[i]) ) {
      return 0.0;
    } // if
  } // for

  const std::vector<Math::LineSegment>& edges1 = bb1.getAllEdges( );
  const std::vector<Math::LineSegment>& edges2 = bb2.getAllEdges( );

  double shortestDistance = std::numeric_limits<double>::max( );
  for ( i = 0; i < edges1.size(); ++i ) {
    for ( j = 0; j < edges2.size(); ++j ) {
      double candidateDistance = edges1[i].distanceTo(edges2[j]);
      if ( candidateDistance < shortestDistance ) {
	shortestDistance = candidateDistance;
      } // if
    } // for
  } // for

  return shortestDistance;  
}

void Entity::setProperty( Entity::PROPERTY property, PropertyValueType value ) {
  this->properties.insert( PropertyHashMap::value_type( property, value ) );
}



bool Entity::getBooleanProperty( Entity::PROPERTY property ) const {
  PropertyHashMap::const_iterator it = this->properties.find( property );
  const bool* result = NULL;
  if ( it == this->properties.end( ) || ( result = boost::get<bool>( &it->second ) ) == NULL ) {
    return false;
  } // if
  return *result;
}

std::string Entity::getStringProperty( PROPERTY property ) const {
  PropertyHashMap::const_iterator it = this->properties.find( property );
  const std::string* result = NULL;
  if ( it == this->properties.end( ) && ( result = boost::get<std::string>( &it->second ) ) == NULL ) {
    return "";
  } // if
  return *result;
}

double Entity::getDoubleProperty( PROPERTY property ) const {
  PropertyHashMap::const_iterator it = this->properties.find( property );
  const double* result = NULL;
  if ( it == this->properties.end( ) && ( result = boost::get<double>( &it->second ) ) == NULL ) {
    return 0.0;
  } // if
  return *result;
}

int Entity::getIntProperty( PROPERTY property ) const {
  PropertyHashMap::const_iterator it = this->properties.find( property );
  const int* result = NULL;
  if ( it == this->properties.end( ) && ( result = boost::get<int>( &it->second ) ) == NULL ) {
    return 0;
  } // if
  return *result;
}

bool Entity::operator==( const EntityPtr& entity ) const {
  return ( entity->getPosition() == getPosition() && 
	   entity->getOrientation() == getOrientation() && 
	   entity->dimension == getDimension( ) );
}

bool Entity::intersects( const EntityPtr& other ) const {
  return ( distanceTo( other ) == 0 );
}

std::string Entity::toString( ) const {
  std::stringstream description;
  
  description << "id[";
  description << this->id;
  description << "] name[";
  description << this->name;  
  description << "] position[";
  description << this->position.toString( );
  description << "] dim[";
  description << this->getDimension( ).toString( );
  description << "] orient[";
  description << this->orientation.toString( );  
  description << "] radius[";
  description << this->expansionRadius;
  description << "]";

  return description.str();
}
