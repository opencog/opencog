/*
 * opencog/spatial/Entity3D.cc
 *
 * Copyright (C) 2002-2011 OpenCog Foundation
 * All Rights Reserved
 * Author(s): Shujing Ke
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

#include "Entity3D.h"

using namespace opencog;
using namespace opencog::spatial;


Entity3D::Entity3D(BlockVector _centerPosition, int _width, int _lenght, int _height, double yaw, std::string _entityName, std::string _entityClass, bool _is_obstacle)
{
    mCenterPosition = _centerPosition;
    mName = _entityName;
    mEntityClass = _entityClass;
    mYaw = yaw;
    mOrientation = math::Quaternion( math::Vector3::Z_UNIT, yaw);
    mBoundingBox.size_x = _width;
    mBoundingBox.size_y = _lenght;
    mBoundingBox.size_z = _height;
    mBoundingBox.nearLeftBottomConer.x = _centerPosition.x - _width / 2;
    mBoundingBox.nearLeftBottomConer.y = _centerPosition.y - _lenght / 2;
    mBoundingBox.nearLeftBottomConer.z = _centerPosition.z - _height / 2;
    is_obstacle =  _is_obstacle;
}

Entity3D* Entity3D::clone()
{
    Entity3D* clonedEntity = new Entity3D(mCenterPosition,mBoundingBox.size_x,mBoundingBox.size_y,mBoundingBox.size_z,mYaw,mName,mEntityClass,is_obstacle);
    return clonedEntity;
}

Entity3D::~Entity3D()
{

}

BlockVector Entity3D::getDirection() const
{
    math::Vector3 mathVec = mOrientation.rotate( math::Vector3::X_UNIT );
    int x = 1, y = 1;
    if (mathVec.x < 0)
        x = -1;
    if (mathVec.y < 0)
        y = -1;

    // calculate the tan of the angle
    double tan = mathVec.y / mathVec.x;
    if (tan < 0)
        tan *= -1.0f;

    // For the sake of simplification, we only have 8 kinds of direction:
    // (x = 1, y = 0):    -pai/8 <= a < pai/8
    // (x = 1, y = 1):     pai/8 <= a < pai*3/8
    // (x = 0, y = 1):   pai*3/8 <= a < pai*5/8
    // (x = -1, y = 1):  pai*5/8 <= a < pai*7/8
    // (x = -1, y = 0):  pai*7/8 <= a < -pai*7/8
    // (x = -1, y = -1):-pai*7/8 <= a < -pai*5/8
    // (x = 0, y = -1): -pai*5/8 <= a < -pai*3/8
    // (x = 1, y = -1): -pai*5/8 <= a < -pai/8

    // tan(pai/8) = 0.41414356f
    // tan(pai*3/8) = 2.41421356f

    if (tan < 0.41414356f) // x >> y, so |x| -> 1, y -> 0
        y = 0;
    else if (tan > 2.41421356f) // x <<y , so x -> 0, |y| -> 1
        x = 0;
    // else x approximate to y, so |x| -> 1, |y| -> 1

    return BlockVector(x,y,0);

}

void Entity3D::updateNonBlockEntitySpaceInfo(BlockVector _centerPosition, int _width, int _lenght, int _height,double yaw, bool _is_obstacle)
{
    mCenterPosition = _centerPosition;
    mYaw = yaw;
    mOrientation = math::Quaternion( math::Vector3::Z_UNIT, yaw);
    mBoundingBox.size_x = _width;
    mBoundingBox.size_y = _lenght;
    mBoundingBox.size_z = _height;
    mBoundingBox.nearLeftBottomConer.x = _centerPosition.x - _width / 2;
    mBoundingBox.nearLeftBottomConer.y = _centerPosition.y - _lenght / 2;
    mBoundingBox.nearLeftBottomConer.z = _centerPosition.z - _height / 2;
    is_obstacle =  _is_obstacle;
}

void Entity3D::updateNonBlockEntityLocation(BlockVector _centerPosition)
{
    mCenterPosition = _centerPosition;
    mBoundingBox.nearLeftBottomConer.x = _centerPosition.x - mBoundingBox.size_x / 2;
    mBoundingBox.nearLeftBottomConer.y = _centerPosition.y - mBoundingBox.size_y / 2;
    mBoundingBox.nearLeftBottomConer.z = _centerPosition.z - mBoundingBox.size_z / 2;
}
