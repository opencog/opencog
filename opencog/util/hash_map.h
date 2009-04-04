/*
 * opencog/util/hash_map.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
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

#ifndef _OPENCOG_HASH_MAP_H
#define _OPENCOG_HASH_MAP_H

#include <string>
#ifdef WIN32
#include <hash_map>
#else
#include <ext/hash_map>
#endif // WIN32

namespace opencog {
    using __gnu_cxx::hash_map;
    using __gnu_cxx::hash;
}

namespace __gnu_cxx
{
template<> struct hash<std::string> {
    size_t operator()(const std::string& x) const {
        return hash<const char*>()(x.c_str());
    }
};
}

#endif // _OPENCOG_HASH_MAP_H
