/*
 * opencog/util/hashing.h
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

#ifndef _OPENCOG_HASHING_H
#define _OPENCOG_HASHING_H

#include <boost/functional/hash.hpp>

#include <opencog/util/tree.h>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

//! Functor returning the address of an object pointed by an iterator.
/**
 * Useful for defining the hash function of an iterator.
 */
template<typename It>
struct obj_ptr_hash {
    size_t operator()(const It& it) const {
        return boost::hash_value(&(*it));
    }
};

template < typename T,
typename Hash = boost::hash<T> >
struct deref_hash {
    deref_hash(const Hash& h = Hash()) : hash(h) {}
    size_t operator()(const T& t) const {
        return hash(*t);
    }
    Hash hash;
};

template < typename T,
typename Equals = std::equal_to<T> >
struct deref_equals {
    deref_equals(const Equals& e = Equals()) : equals(e) {}
    bool operator()(const T& x, const T& y) const {
        return equals(*x, *y);
    }
    Equals equals;
};

template<typename T>
std::size_t hash_value(const tree<T>& tr)
{
    return boost::hash_range(tr.begin(), tr.end());
}

//! Functor comparing the addresses of objects pointed by 
//! tree iterators.
/**
 * Useful for storing iterators in a std::map.
 * (the tree has pointer to node, we use that to identify the
 * tree node uniquely).
 */
template<typename It>
struct obj_ptr_cmp {
    bool operator()(const It& lit, const It& rit) const {
        return ((void *) lit.node) < ((void *) rit.node);
    }
};

/** @}*/
} //~namespace opencog

#endif // _OPENCOG_HASHING_H
