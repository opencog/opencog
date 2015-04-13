/*
 * opencog/spatial/math/mathCommon.cc
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

#include <opencog/spatial/math/Vector3.h>
#include <opencog/spatial/math/LineSegment.h>

using namespace opencog;
using namespace opencog::spatial::math;

const Vector3 Vector3::ZERO;
const Vector3 Vector3::X_UNIT( 1, 0, 0 );
const Vector3 Vector3::Y_UNIT( 0, 1, 0 );
const Vector3 Vector3::Z_UNIT( 0, 0, 1 );
const Vector3 Vector3::NEG_X_UNIT( -1, 0, 0 );
const Vector3 Vector3::NEG_Y_UNIT( 0, -1, 0 );
const Vector3 Vector3::NEG_Z_UNIT( 0, 0, -1 );

const double LineSegment::TOLERANCE_DISTANCE = 0.001;
