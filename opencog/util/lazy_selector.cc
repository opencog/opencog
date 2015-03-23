/*
 * opencog/util/lazy_selector.cc
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

#include "lazy_selector.h"
#include "exceptions.h"
#include "oc_assert.h"
#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <vector>
#include <boost/bind.hpp>
#include <boost/iterator/counting_iterator.hpp>

namespace opencog
{

using std::make_pair;
using std::count_if;
using boost::counting_iterator;
using boost::bind;

lazy_selector::lazy_selector(unsigned int u, unsigned int l)
    : _u(u), _l(l) {
    OC_ASSERT(u - l > 0, "you cannot select any thing from an empty list");
}

bool lazy_selector::empty() const {
    return _l >= _u;
}

unsigned int lazy_selector::count_n_free() const {
    return count_if(counting_iterator<unsigned int>(_l),
                    counting_iterator<unsigned int>(_u),
                    bind(&lazy_selector::is_free, this, _1));
}

void lazy_selector::reset_range(unsigned int new_u) {
    _u = new_u;
}
void lazy_selector::reset_range(unsigned int new_u, unsigned int new_l) {
	OC_ASSERT(new_l >= _l,
	          "You cannot reset the lower bound by a lower number, "
	          "due to the workings of the algorithm.");
    _u = new_u;
    _l = new_l;
}

/**
 * returns the selected number (never twice the same)
 *
 * It works by creating a mapping between already selected indices and
 * free indices (never selected yet).
 * 
 * To chose the free indices, we use an index _l, that
 * goes from the lower range to the upper range and only corresponds to
 * free indices. So every time _l corresponds to a non free index
 * it is incremented until it gets to a free index.
 *
 */
unsigned int lazy_selector::operator()()
{
    OC_ASSERT(!empty(), "lazy_selector - selector is empty.");

    unsigned int sel_idx = select();
    
    // If the selected index points to nothing then the result is
    // itself otherwise it is _l
    unsigned int res = is_free(sel_idx) ? sel_idx : _l;

    // Move _l from res so that it is now a free index
    if(res == _l) increase_l_till_free();

    _picked.insert(res);

    return res;
}

bool lazy_selector::is_free(unsigned int idx) const {
    return _picked.find(idx) == _picked.end();
}

void lazy_selector::increase_l_till_free() {
    do {
        _l++;
    } while(!is_free(_l));
}

} //~namespace opencog
