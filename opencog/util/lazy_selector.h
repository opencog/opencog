/*
 * opencog/util/lazy_selector.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * Authors Moshe Looks, Nil Geisweiller
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

#ifndef _OPENCOG_LAZY_SELECTOR_H
#define _OPENCOG_LAZY_SELECTOR_H

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/hashed_index.hpp>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

using boost::multi_index_container;
using namespace boost::multi_index;

/**
 * That class allows to select integers in [0,n)
 * but never select twice the same
 * When the operator is called more than n times an assertion is raised
 */
class lazy_selector
{	
    typedef std::pair<unsigned int, unsigned int> uint_pair;

    //! tags for accessing both sides of a bidirectional map
    struct from{};
    //! tags for accessing both sides of a bidirectional map
    struct to{};

    /** A bidirectional map one-to-many is simulated as a multi_index_container
     * of pairs of (unsinged int,unsigned int) with first unique index, 
     * and second non unique index.
     *
     * That structure is used to represent what element should be returned when
     * the selected element has already been chosen.
     * 
     * The one-to-many mapping is used because an already chosen element
     * should only have one possible choice to return instead of itself, 
     * but on the other hand several already chosen elements can points to the
     * same alternative. Of course if one of them is chosen and must return
     * a given alternative all element pointing to it must be updated which
     * is why we use a multi_index_container to quickly access either the first
     * or the second element of the pair.
     */

    typedef multi_index_container<
        uint_pair,
        indexed_by<
            hashed_unique<
                tag<from>,member<uint_pair, unsigned int, &uint_pair::first> >,
            hashed_non_unique<
                tag<to>, member<uint_pair, unsigned int, &uint_pair::second> >
            >
        > uint_one_to_many_map;
    typedef uint_one_to_many_map::iterator uint_one_to_many_map_it;
    typedef uint_one_to_many_map::const_iterator uint_one_to_many_map_cit;

public:
    lazy_selector(unsigned int n);
    virtual ~lazy_selector() {}
    bool empty() const;

    //! returns the number of elements < _n that can still be chosen
    unsigned int count_n_free() const;

    //! returns the selected number (never twice the same)
    unsigned int operator()();

    void reset_range(unsigned int new_n);

protected:
    //! size of the integer list [0,_n)
    unsigned int _n; 

    //! a method that choses an int in [0,_n)
    virtual unsigned int select() = 0;

private:
    //! lower index
    unsigned int _l; 
    uint_one_to_many_map _map;

    //! an index is free if it has never been chosen before
    //! that is if no links are going out of it
    inline bool is_free(unsigned int idx) const; 

    //! increase _l until it is located on a free index
    inline void increase_l_till_free();

    //! modify all links coming into src_to so that they now go to dst_to
    inline void modify_target(unsigned int src_to, unsigned int dst_to);
};

/** @}*/
} //~namespace opencog

#endif
