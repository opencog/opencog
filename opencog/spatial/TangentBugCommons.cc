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

#include "TangentBugCommons.h"

#include <cmath>
#include <limits>
#include <stdio.h>

using namespace Spatial;
using namespace TangentBugBits;

/**
 * TangentBugBits::Ray class
 */
Ray::Ray() : x(0), y(0)
{
}

Ray::Ray(const Ray& other) : x(other.x), y(other.y)
{
}

Ray::Ray(double _x, double _y) : x(_x), y(_y)
{
}

Ray::~Ray()
{
}

double Ray::length() const
{
    return std::sqrt(std::pow(this->x, 2) + std::pow(this->y, 2));
}

Ray Ray::operator-(const Ray& rhs) const
{
    return Ray(this->x - rhs.x, this->y - rhs.y);
}

Ray& Ray::operator=(const Ray & rhs)
{
    this -> x = rhs.x;
    this -> y = rhs.y;
    return *this;
}

bool Ray::operator==(const Ray& rhs) const
{
    return (this -> x - rhs.x < std::numeric_limits<double>::epsilon() &&
            this -> y - rhs.y < std::numeric_limits<double>::epsilon());
}

Ray Ray::operator*(double scalar) const
{
    return Ray(this->x*scalar, this->y*scalar);
}

Ray Ray::operator/(double scalar) const
{
    return Ray(this->x / scalar, this->y / scalar);
}

double Ray::operator*(const Ray& other) const
{
    return this->x * other.x + this->y * other.y;
}

Ray Ray::normalise() const
{
    return normalize();
}

Ray Ray::normalize() const
{
    double len = length();

    // Do not divide by 0. If len == Ray(0,0) then return zero ray
    if (len > Ray(std::numeric_limits<double>::epsilon(),
                  std::numeric_limits<double>::epsilon()).length()) {
        return Ray(this->x / len, this->y / len);
    }
    return Ray(std::numeric_limits<double>::epsilon(),
               std::numeric_limits<double>::epsilon());
}

/**
 * TangentBugBits::Point class
 */
Point::Point()
{
}

Point::Point(const point_2d& other) : point_2d(other)
{
}

Point::Point(coord x, coord y) : point_2d(x, y)
{
}

Point::~Point()
{
}

Point Point::operator+(const Ray& ray) const
{
    // Though we should only be dealing with unsigned ints, we still need rounding
    // to be the same for pos and neg numbers--we need
    // Point(0,0) + Ray(-0.6, -0.6) != Point(0,0)
    return Point((coord) std::floor(first + ray.x + 0.5),
                 (coord) std::floor(second + ray.y + 0.5));
}

Ray Point::operator-(const Point& other) const
{
    return Ray((signed)(first - other.first), (signed)(second - other.second));
}

Point& Point::operator=(const Point & rhs)
{
    point_2d::operator=(rhs);
    return *this;
}

/**
 * TangentBugBits::look_info stuct
 */
look_info::look_info() : collided(false)
{
}

look_info::look_info(const Point& look_from, const Point& look_to) :
        last_point_before_hit(look_to), last_point(look_to), collided(false)
{
}

