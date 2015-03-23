/*
 * opencog/spatial/TangentBugCommons.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Dan Zwell, Carlos Lopes
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

#include <opencog/spatial/TangentBugCommons.h>

#include <cmath>
#include <limits>
#include <stdio.h>

using namespace opencog;
using namespace opencog::spatial;

/**
 * TBRay class
 */
TBRay::TBRay() : x(0), y(0)
{
}

TBRay::TBRay(const TBRay& other) : x(other.x), y(other.y)
{
}

TBRay::TBRay(double _x, double _y) : x(_x), y(_y)
{
}

TBRay::~TBRay()
{
}

double TBRay::length() const
{
    return std::sqrt(std::pow(this->x, 2) + std::pow(this->y, 2));
}

TBRay TBRay::operator-(const TBRay& rhs) const
{
    return TBRay(this->x - rhs.x, this->y - rhs.y);
}

TBRay& TBRay::operator=(const TBRay & rhs)
{
    this -> x = rhs.x;
    this -> y = rhs.y;
    return *this;
}

bool TBRay::operator==(const TBRay& rhs) const
{
    return (this -> x - rhs.x < std::numeric_limits<double>::epsilon() &&
            this -> y - rhs.y < std::numeric_limits<double>::epsilon());
}

TBRay TBRay::operator*(double scalar) const
{
    return TBRay(this->x*scalar, this->y*scalar);
}

TBRay TBRay::operator/(double scalar) const
{
    return TBRay(this->x / scalar, this->y / scalar);
}

double TBRay::operator*(const TBRay& other) const
{
    return this->x * other.x + this->y * other.y;
}

TBRay TBRay::normalise() const
{
    return normalize();
}

TBRay TBRay::normalize() const
{
    double len = length();

    // Do not divide by 0. If len == TBRay(0,0) then return zero ray
    if (len > TBRay(std::numeric_limits<double>::epsilon(),
                  std::numeric_limits<double>::epsilon()).length()) {
        return TBRay(this->x / len, this->y / len);
    }
    return TBRay(std::numeric_limits<double>::epsilon(),
               std::numeric_limits<double>::epsilon());
}

/**
 * TBPoint class
 */
TBPoint::TBPoint()
{
}

TBPoint::TBPoint(const point_2d& other) : point_2d(other)
{
}

TBPoint::TBPoint(coord x, coord y) : point_2d(x, y)
{
}

TBPoint::~TBPoint()
{
}

TBPoint TBPoint::operator+(const TBRay& ray) const
{
    // Though we should only be dealing with unsigned ints, we still need rounding
    // to be the same for pos and neg numbers--we need
    // TBPoint(0,0) + TBRay(-0.6, -0.6) != TBPoint(0,0)
    return TBPoint((coord) std::floor(first + ray.x + 0.5),
                 (coord) std::floor(second + ray.y + 0.5));
}

TBRay TBPoint::operator-(const TBPoint& other) const
{
    return TBRay((signed)(first - other.first), (signed)(second - other.second));
}

TBPoint& TBPoint::operator=(const TBPoint & rhs)
{
    point_2d::operator=(rhs);
    return *this;
}

/**
 * look_info stuct
 */
look_info::look_info() : collided(false)
{
}

look_info::look_info(const TBPoint& look_from, const TBPoint& look_to) :
        last_point_before_hit(look_to), last_point(look_to), collided(false)
{
}

