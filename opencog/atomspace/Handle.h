/*
 * opencog/atomspace/Handle.h
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2013 Linas Vepstas <linasvepstas@gmail.com>
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
#include <memory>
#include <string>
#include <sstream>
#include <unordered_set>
#include <vector>

/** \addtogroup grp_atomspace
 *  @{
 */
namespace opencog
{

//! UUID == Universally Unique Identifier
typedef unsigned long UUID;


//! contains an unique identificator
class Atom;
typedef std::shared_ptr<Atom> AtomPtr;
class Handle : public AtomPtr
{

friend class TLB;
friend class AtomStorage;
friend class SchemeSmob;
friend class AtomspaceHTabler;

private:

    UUID uuid;

public:

    static const Handle UNDEFINED;

    explicit Handle(AtomPtr atom);
    explicit Handle(const UUID u) : uuid(u) {}
    explicit Handle() : uuid(UNDEFINED.uuid) {}
    Handle(const Handle& h) : AtomPtr(h), uuid(h.uuid) {}
    ~Handle() {}

    inline UUID value(void) const {
        return uuid;
    }

    inline Handle& operator=(const Handle& h) {
        if (this == &h) return *this;
        AtomPtr* base = static_cast<AtomPtr*>(this);
        base->operator=(h);
        this->uuid = h.uuid;
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
    static int compare(const Handle& h1, const Handle& h2)
    {
        if (h1 < h2) return -1;
        if (h1 > h2) return 1;
        return 0;
    }
};
 
//! gcc-4.7.2 needs this, because std::hash<opencog::Handle> no longer works.
//! (See very bottom of this file).
struct handle_hash : public std::unary_function<Handle, size_t>
{
   size_t operator()(const Handle& h) const
   {
       return static_cast<std::size_t>(h.value());
   }
};
 
//! Boost needs this function to be called by exactly this name.
inline std::size_t hash_value(Handle const& h)
{
    return static_cast<std::size_t>(h.value());
}

/// Compare handle uuid's ONLY. Do not compare atom pointers
/// (as one might be null, and the other one not null.)
struct handle_less
{
   bool operator()(const Handle& hl, const Handle& hr) const
   {
       return hl.value() < hr.value();
   }
};
 
//! a list of handles
typedef std::vector<Handle> HandleSeq;
//! a list of lists of handles
typedef std::vector<HandleSeq> HandleSeqSeq;
//! a hash that associates the handle to its unique identificator
typedef std::unordered_set<Handle, handle_hash> UnorderedHandleSet;

//! append string representation of the Hash to the string
static inline std::string operator+ (const char *lhs, Handle h)
{
    std::string rhs = lhs;
    char buff[25];
    snprintf(buff, 24, "%lu)", h.value());
    return rhs + buff;
}

//! append string representation of the Hash to the string
static inline std::string operator+ (const std::string &lhs, Handle h)
{
    char buff[25];
    snprintf(buff, 24, "%lu)", h.value());
    return lhs + buff;
}

} // namespace opencog

namespace std { 
inline std::ostream& operator<<(std::ostream& out, const opencog::Handle& h)
{
    out << h.value();
    return out;
}

#ifdef THIS_USED_TO_WORK_GREAT_BUT_IS_BROKEN_IN_GCC472
// I have no clue why gcc-4.7.2 broke this, and neither does google or
// stackoverflow.  Use handle_hash, above, instead.

template<>
inline std::size_t std::hash<opencog::Handle>::operator()(opencog::Handle h) const
{  
    return static_cast<std::size_t>(h.value());
}
#endif // THIS_USED_TO_WORK_GREAT_BUT_IS_BROKEN_IN_GCC472

} //namespace std

/** @}*/
#endif // _OPENCOG_HANDLE_H
