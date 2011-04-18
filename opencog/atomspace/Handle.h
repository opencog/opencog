/*
 * opencog/atomspace/Handle.h
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
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

#ifndef _OPENCOG_HANDLE_H
#define _OPENCOG_HANDLE_H

#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <boost/unordered_set.hpp>

namespace opencog
{

// UUID == Universally Unique Identifier
typedef unsigned long UUID;

class Handle
{

friend class TLB;
friend class AtomStorage;
friend class SchemeSmob;
friend class AtomspaceHTabler;

private:

    UUID uuid;

public:

    static const Handle UNDEFINED;

    explicit Handle(const UUID u) : uuid(u) {};
    Handle(const Handle& h) : uuid(h.uuid) {};
    explicit Handle() : uuid(UNDEFINED.uuid) {};
    ~Handle() {}

    inline UUID value(void) const {
        return uuid;
    }

    inline Handle& operator=(const Handle& h) {
        uuid = h.uuid;
        return *this;
    }

    inline bool operator==(const Handle& h) const { return uuid == h.uuid; }
    inline bool operator!=(const Handle& h) const { return uuid != h.uuid; }
    inline bool operator< (const Handle& h) const { return uuid <  h.uuid; }
    inline bool operator> (const Handle& h) const { return uuid >  h.uuid; }
    inline bool operator<=(const Handle& h) const { return uuid <= h.uuid; }
    inline bool operator>=(const Handle& h) const { return uuid >= h.uuid; }


    /**
     * Returns a negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger than
     * the second argument.
     *
     * @param The first handle element.
     * @param The second handle element.
     * @return A negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger then the
     * second argument.
     */
    static int compare(Handle h1, Handle h2)
    {
        if (h1 < h2) return -1;
        if (h1 > h2) return 1;
        return 0;
    }
};

typedef std::vector<Handle> HandleSeq;
typedef std::vector<HandleSeq> HandleSeqSeq;
typedef boost::unordered_set<Handle, boost::hash<opencog::Handle> > UnorderedHandleSet;

static inline std::string operator+ (const char *lhs, Handle h)
{
    std::string rhs = lhs;
    char buff[25];
    snprintf(buff, 24, "%lu)", h.value());
    return rhs + buff;
}

static inline std::string operator+ (const std::string &lhs, Handle h)
{
    char buff[25];
    snprintf(buff, 24, "%lu)", h.value());
    return lhs + buff;
}

inline std::size_t hash_value(Handle const& h)
{
    return static_cast<std::size_t>(h.value());
}

} // namespace opencog

namespace std { 
inline std::ostream& operator<<(std::ostream& out, const opencog::Handle& h) {
    out << h.value();
    return out;
}
} //namespace std

#endif // _OPENCOG_HANDLE_H
