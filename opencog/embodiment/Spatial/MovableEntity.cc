#include "MovableEntity.h"

using namespace Spatial;

void MovableEntity::rotate( const Math::Quaternion& rotation ) {
  this->needUpdate = true;
  this->orientation *= rotation;
}

void MovableEntity::setPosition( const Math::Vector3& position) {
  if ( this->position == position ) {
    return;
  } // if
  this->needUpdate = true;
  this->position = position;
}

void MovableEntity::setOrientation(const Math::Quaternion& orientation) {
  if ( this->orientation == orientation ) {
    return;
  } // if
  this->needUpdate = true;
  this->orientation = orientation;
}
