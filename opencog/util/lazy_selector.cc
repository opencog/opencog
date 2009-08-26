/*
 * opencog/util/lazy_selector.cc
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

#include "lazy_selector.h"
#include "exceptions.h"
#include "oc_assert.h"

namespace opencog
{

lazy_selector::lazy_selector(unsigned int n) 
    : _n(n), _l(0), _u(n-1) {
    OC_ASSERT(n>0, "you cannot select any thing from an empty list");
}

bool lazy_selector::empty() const {
    return _n == 0;
}

void lazy_selector::resize(unsigned int new_n) {
    OC_ASSERT(_n < new_n);
    _n = new_n;
}

/** returns the selected number (never twice the same)
 *
 * it works by pointing any already selected number
 * either to _l or _u.
 * If the selected number is _l then it points to _u and vice versa.
 * If it is both (because _l == _u) then it points to _n.
 * Yes _n is out of the range but that is what is doing the trick
 * so the range can be resize higher.
 *
 * Each time a pointer is created toward _l, it increments,
 * and each time a pointer is created toward _u, it decrements.
 *
 * All the rest of the algo is pointer maintance so that a pointer
 * never points to an already chosen value.
 *
 */
unsigned int lazy_selector::operator()()
{
    OC_ASSERT(!empty(), "lazy_selector - selector is empty.");

    unsigned int sel = select();

    

    // if(sel != _u) {
        //TODO
    //} else if (sel != _l) {
        //TODO
    //} else if 



    unsigned int idx = select();
    _n--;

    hash_map<unsigned int, unsigned int>::iterator it = _map.find(idx);
    if (idx == _n) {
        if (it != _map.end()) {
            idx = it->second;
            _map.erase(it);
        }
        return idx;
    }

    unsigned int res = (it == _map.end()) ? idx : it->second;
    if(it != _map.end())
        _map.erase(it);
    hash_map<unsigned int, unsigned int>::iterator last = _map.find(_n);
    if (last == _map.end()) {
        _map.insert(std::make_pair(idx, _n));
    } else {
        _map.insert(std::make_pair(idx, last->second));
        _map.erase(last);
    }
    return res;
}

} //~namespace opencog
