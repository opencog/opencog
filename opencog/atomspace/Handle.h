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

#include <cstddef>
#include <cstdio>
#include <iostream>
#include <climits>
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
typedef std::unordered_set<UUID> UnorderedUUIDSet;


class Atom;
typedef std::shared_ptr<Atom> AtomPtr;

//! contains an unique identificator
class AtomTable;
class Handle
{

friend class AtomTable;
friend class AtomStorage;         // persistance
friend class AtomspaceHTabler;    // persistance

private:

    UUID _uuid;
    AtomPtr _ptr;

    Atom* resolve();
    Atom* cresolve() const;
    static AtomPtr do_res(const Handle*);
    static std::vector<const AtomTable*> _resolver;
public:

    static const Handle UNDEFINED;

    explicit Handle(AtomPtr atom);
    explicit Handle(const UUID u) : _uuid(u) {}
    explicit Handle() : _uuid(ULONG_MAX) {}
    Handle(const Handle& h) : _uuid(h._uuid), _ptr(h._ptr) {}
    ~Handle() {}

    inline UUID value(void) const {
        return _uuid;
    }

    inline Handle& operator=(const Handle& h) {
        if (this->_uuid == h._uuid) {
            Atom* a = h._ptr.get();
            // The 'typical' case here is where a isn't null,
            // but this is.  The weirdo case is where both
            // aren't null, and yet differ.
            // Mostly, we want to avoid the CPU overhead of calling
            // resolve(), it we can; so the goal of this if-stmt is
            // to upgrade the ptr from null to non-null.
            if (a != NULL and a != this->_ptr.get())
                this->_ptr = h._ptr;
            return *this;
        }
        this->_uuid = h._uuid;
        this->_ptr = h._ptr;
        return *this;
    }

    Handle& operator=(const AtomPtr& a);

    inline Atom* operator->() {
        Atom* ptr = _ptr.get();
        if (ptr) return ptr;
        if (ULONG_MAX == _uuid) return NULL;
        return resolve();
    }

    inline Atom* operator->() const {
        Atom* ptr = _ptr.get();
        if (ptr) return ptr;
        if (ULONG_MAX == _uuid) return NULL;
        return cresolve();
    }

    // Allows expressions like "if(h)..." to work when h has a non-null pointer.
    explicit inline operator bool() const noexcept {
        if (_ptr.get()) return true;
        return NULL != cresolve(); // might be null because we haven't resolved it yet!
    }

    inline bool operator==(std::nullptr_t) const noexcept {
        if (_ptr.get()) return false;
        return NULL == cresolve(); // might be null because we haven't resolved it yet!
    }

    inline bool operator!=(std::nullptr_t) const noexcept {
        if (_ptr.get()) return true;
        return NULL != cresolve(); // might be null because we haven't resolved it yet!
    }

    // Handles are equivalent when their uuid's compare. It may happen
    // that one has a null pointer, and the other one doesn't; we don't
    // care about that. It should never ever happen that we have two
    // identical uuid's but inequivalent pointers!!
    inline bool operator==(const Handle& h) const noexcept { return _uuid == h._uuid; }
    inline bool operator!=(const Handle& h) const noexcept { return _uuid != h._uuid; }
    inline bool operator< (const Handle& h) const noexcept { return _uuid <  h._uuid; }
    inline bool operator> (const Handle& h) const noexcept { return _uuid >  h._uuid; }
    inline bool operator<=(const Handle& h) const noexcept { return _uuid <= h._uuid; }
    inline bool operator>=(const Handle& h) const noexcept { return _uuid >= h._uuid; }


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

    AtomPtr resolve_ptr();
    static void set_resolver(const AtomTable*);
    static void clear_resolver(const AtomTable*);

    operator AtomPtr() const {
        if (_ptr.get()) return _ptr;
        if (ULONG_MAX == _uuid) return AtomPtr();
        Handle h(*this);
        return h.resolve_ptr();
    }
    operator AtomPtr() {
        if (_ptr.get()) return _ptr;
        if (ULONG_MAX == _uuid) return AtomPtr();
        return resolve_ptr();
    }
};

static inline bool operator== (std::nullptr_t, const Handle& rhs) noexcept
    { return rhs == NULL; }

static inline bool operator!= (std::nullptr_t, const Handle& rhs) noexcept
    { return rhs != NULL; }

class HandlePredicate {
public:
    inline bool operator()(const Handle& h) { return this->test(h); }
    virtual bool test(const Handle& h) { return true; }
};
class AtomPredicate {
public:
    inline bool operator()(AtomPtr a) { return this->test(a); }
    virtual bool test(AtomPtr) { return true; }
};
class AtomComparator {
public:
    inline bool operator()(AtomPtr a, AtomPtr b) { return this->test(a,b); }
    virtual bool test(AtomPtr, const AtomPtr) { return true; }
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
