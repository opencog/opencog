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

#include <unordered_set>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

/**
 * That class allows to select integers in [0,n)
 * but never select twice the same
 * When the operator is called more than n times an assertion is raised
 */
class lazy_selector
{	
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

    //! Keep track of all encountered values (to return _l instead of it)
    std::unordered_set<unsigned int> _picked;

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
