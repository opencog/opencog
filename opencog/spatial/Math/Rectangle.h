/*
 * opencog/spatial/Math/Rectangle.h
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
#ifndef _SPATIAL_MATH_RECTANGLE_H_
#define _SPATIAL_MATH_RECTANGLE_H_

#include "Vector3.h"
#include <opencog/util/exceptions.h>

namespace Spatial
{
namespace Math {

class Rectangle
{
public:

    Rectangle( const Rectangle& other );

    Rectangle( const Vector3& leftTopCorner, const Vector3& rightTopCorner, const Vector3& rightBottomCorner ) throw(opencog::InvalidParamException);

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

} // Math
} // Spatial

#endif // _SPATIAL_MATH_RECTANGLE_H_
