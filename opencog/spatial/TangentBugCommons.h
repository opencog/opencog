/*
 * opencog/spatial/TangentBugCommons.h
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

#ifndef _SPATIAL_TANGENT_BUG_COMMONS_H_
#define _SPATIAL_TANGENT_BUG_COMMONS_H_

#include <opencog/util/Logger.h>
#include <opencog/util/numeric.h>

#include <cstdio>
#include <set>
#include <string>
#include <iostream>
#include <boost/functional/hash.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

namespace Spatial
{
namespace TangentBugBits {

class Point;
//typedef int obstacle_id;
typedef unsigned int coord;
typedef std::pair<coord, coord> point_2d; // Parent class of Point

typedef boost::unordered_set<int> int_set;
typedef boost::unordered_map<point_2d, int_set, boost::hash<point_2d> > point_map;

/**
 * Ray class
 */
class Ray
{

public:

    double x, y;

    Ray();
    Ray(const Ray& other);
    Ray(double _x, double _y);
    ~Ray();

    double length() const;

    Ray& operator=(const Ray& rhs);
    Ray operator-(const Ray& rhs) const;
    Ray operator*(double scalar) const;
    Ray operator/(double scalar) const;
    bool operator==(const Ray& rhs) const;

    // Dot product
    double operator*(const Ray& other) const;

    Ray normalise() const;
    Ray normalize() const;

    friend class Spatial::TangentBugBits::Point;
    friend std::ostream& operator<<(std::ostream& out, const Ray& ray);

}; // Ray class

/**
 * TangentBugBits::Point class
 */
struct Point : public Spatial::TangentBugBits::point_2d {

    Point();
    Point(const point_2d& other);
    Point(coord x, coord y);
    ~Point();

    // Rounds to nearest point.
    Point operator+(const Ray& ray) const;
    Ray operator-(const Point& other) const;
    Point& operator=(const Point& rhs);

    friend std::ostream& operator<<(std::ostream& out, const Point& pt);

}; // TangentBugBits::Point class

/**
 * TangentBugBits::look_info struct
 */
struct look_info {

    Point last_point_before_hit;
    Point last_point;
    bool collided;

    // We need to keep the original direction that was looked down. After all,
    // to if we merely stored points p1 and p2, we would later have to
    // construct the ray p2-p1. But this would not be the same exact direction
    // as the original ray, and this new ray might pass through occupied
    // points that the old ray did not. That way lies madness, especially when
    // you try to shorten the ray (p2-p1) and this new, shortened ray ends up
    // inside an occupied point.
    Ray orig_direction;

    look_info();
    look_info(const Point& look_from, const Point& look_to = Point());

}; // TangentBugBits::look_info struct

inline std::ostream& operator<<(std::ostream& out, const Ray& r)
{
    static size_t maxlen = 30;
    char s[maxlen];
    snprintf(s, maxlen, "(%0.5f,%0.5f)", r.x, r.y);
    out << s;
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Point& pt)
{
    out << "(" << pt.first << "," << pt.second << ")";
    return out;
}

// Random int from a gaussian distribution. Neg numbers are clipped to 0
/*        static unsigned int pos_gaussian_rand(unsigned int std_dev, coord mean){
            int random = mean +
                static_cast<coord>(std_dev *
                        std::sqrt(-2 * std::log(rng.randDoubleOneExcluded())) *
                        std::cos(2 * PI * rng.randDoubleOneExcluded()));

            return (random >= 0 ? (unsigned) random : 0);
        }*/

} // namespace TangentBugBits
} // namespace Spatial

#endif

