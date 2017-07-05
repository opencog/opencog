/*
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "TimeOctomap.h"
#include "opencog/util/oc_assert.h"

namespace std {

std::ostream& operator<<(std::ostream& out, const opencog::time_pt& pt)
{
    // XXX FIXME -- make this print milliseconds
    out << std::chrono::system_clock::to_time_t(pt);
    return out;
}

std::ostream& operator<<(std::ostream& out, const opencog::duration_c& du)
{
    out << "foooo duration";
    return out;
}

std::ostream& operator<<(std::ostream& out, const opencog::time_list& tl)
{
    out << "(";
    for (const opencog::time_pt& pt : tl)
        out << std::chrono::system_clock::to_time_t(pt) << " ";
    out << ")";
    return out;
}

std::ostream& operator<<(std::ostream& out, const octomap::point3d_list& pl)
{
    out << "(";
    for (const auto& pt : pl)
        out << pt << " ";
    out << ")";
    return out;
}
}
