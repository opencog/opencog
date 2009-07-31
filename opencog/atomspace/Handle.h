/*
 * opencog/atomspace/Handle.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <tr1/unordered_set>

namespace opencog
{

// UUID == Universally Unique Identifier
typedef unsigned long UUID;

class Handle
{

friend class TLB;
friend class ListRequest;
friend class AtomStorage;
friend class SchemeSmob;
friend class AtomspaceHTabler;
friend class CompositeTruthValue; // XXX fixme -- due to wacked fromString

private:

    UUID uuid;

    explicit Handle(const UUID u) : uuid(u) {};

public:

    static const Handle UNDEFINED;

    Handle(const Handle& h) : uuid(h.uuid) {};
    explicit Handle() : uuid(UNDEFINED.uuid) {};
    ~Handle() {}

    inline UUID value(void) const {
        return uuid;
    }

    inline std::string str(void) const {
        std::ostringstream oss;
        oss << uuid;
        return oss.str();
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

};

typedef std::vector<Handle> HandleSeq;
typedef std::vector<HandleSeq> HandleSeqSeq;

} // namespace opencog

namespace std { namespace tr1 {
    template<>
    struct hash<opencog::Handle> : public std::unary_function<const opencog::Handle&, std::size_t> {
        std::size_t operator()(const opencog::Handle& h) const
        { return static_cast<std::size_t>(h.value()); }
    };
}} //namespace std::tr1


namespace std { 
inline std::ostream& operator<<(std::ostream& out, const opencog::Handle& h) {
    out << h.value();
    return out;
}
} //namespace std

#endif // _OPENCOG_HANDLE_H
