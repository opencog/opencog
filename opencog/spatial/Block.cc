/*
 * opencog/spatial/Block.cc
 *
 * Copyright (C) 2002-2011 OpenCog Foundation
 * All Rights Reserved
 * Author(s): Troy Huang
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

#include <opencog/spatial/Block.h>

using namespace opencog;
using namespace opencog::spatial;

void Block::calculateSolidBoundingBox()
{
    // Backup the original radius
    double originRadius = this->expansionRadius;
    // We don't want to have expansion radius in calculation.
    this->expansionRadius = 0;
    this->solidBoundingBox = new math::BoundingBox((Entity*)this);
    this->expansionRadius = originRadius;
}
