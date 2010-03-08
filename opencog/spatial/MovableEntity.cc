/*
 * opencog/spatial/MovableEntity.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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

#include <opencog/spatial/MovableEntity.h>

using namespace opencog;
using namespace opencog::spatial;

void MovableEntity::rotate( const math::Quaternion& rotation )
{
    this->needUpdate = true;
    this->orientation *= rotation;
}

void MovableEntity::setPosition( const math::Vector3& position)
{
    if ( this->position == position ) {
        return;
    } // if
    this->needUpdate = true;
    this->position = position;
}

void MovableEntity::setOrientation(const math::Quaternion& orientation)
{
    if ( this->orientation == orientation ) {
        return;
    } // if
    this->needUpdate = true;
    this->orientation = orientation;
}
