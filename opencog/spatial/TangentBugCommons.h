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

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {

        class TBPoint;
        //typedef int obstacle_id;
        typedef unsigned int coord;
        typedef std::pair<coord, coord> point_2d; // Parent class of Point

        typedef boost::unordered_set<int> int_set;
        typedef boost::unordered_map<point_2d, int_set, boost::hash<point_2d> > point_map;

        /**
         * Ray class
         */
        class TBRay
        {

        public:

            double x, y;

            TBRay();
            TBRay(const TBRay& other);
            TBRay(double _x, double _y);
            ~TBRay();

            double length() const;

            TBRay& operator=(const TBRay& rhs);
            TBRay operator-(const TBRay& rhs) const;
            TBRay operator*(double scalar) const;
            TBRay operator/(double scalar) const;
            bool operator==(const TBRay& rhs) const;

            // Dot product
            double operator*(const TBRay& other) const;

            TBRay normalise() const;
            TBRay normalize() const;

            friend class TBPoint;
            friend std::ostream& operator<<(std::ostream& out, const TBRay& ray);

        }; // TBRay class

        /**
         * TBPoint class
         */
        struct TBPoint : public point_2d {

            TBPoint();
            TBPoint(const point_2d& other);
            TBPoint(coord x, coord y);
            ~TBPoint();

            // Rounds to nearest point.
            TBPoint operator+(const TBRay& ray) const;
            TBRay operator-(const TBPoint& other) const;
            TBPoint& operator=(const TBPoint& rhs);

            friend std::ostream& operator<<(std::ostream& out, const TBPoint& pt);

        }; // TBPoint class

        /**
         * TangentBugBits::look_info struct
         */
        struct look_info {

            TBPoint last_point_before_hit;
            TBPoint last_point;
            bool collided;

            // We need to keep the original direction that was looked down. After all,
            // to if we merely stored points p1 and p2, we would later have to
            // construct the ray p2-p1. But this would not be the same exact direction
            // as the original ray, and this new ray might pass through occupied
            // points that the old ray did not. That way lies madness, especially when
            // you try to shorten the ray (p2-p1) and this new, shortened ray ends up
            // inside an occupied point.
            TBRay orig_direction;

            look_info();
            look_info(const TBPoint& look_from, const TBPoint& look_to = TBPoint());

        }; // TangentBugBits::look_info struct

        inline std::ostream& operator<<(std::ostream& out, const TBRay& r)
        {
            static size_t maxlen = 30;
            char s[maxlen];
            snprintf(s, maxlen, "(%0.5f,%0.5f)", r.x, r.y);
            out << s;
            return out;
        }

        inline std::ostream& operator<<(std::ostream& out, const TBPoint& pt)
        {
            out << "(" << pt.first << "," << pt.second << ")";
            return out;
        }

    } // spatial
/** @}*/
} // opencog

#endif

