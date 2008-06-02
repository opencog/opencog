/*
 * src/Util/lazy_random_selector.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Carlos Lopes <dlopes@vettalabs.com>
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

#include "lazy_random_selector.h"
#include "exceptions.h"
#include "numeric.h"
#include "RandGen.h"

namespace opencog
{

int lazy_random_selector::operator()()
{

    cassert(TRACE_INFO, !empty(), "lazy_random_selector - selector is empty.");
    int idx = rng.randint(_n--);

    hash_map<int, int>::iterator it = _map.find(idx);
    if (idx == _n) {
        if (it != _map.end()) {
            idx = it->second;
            _map.erase(it);
        }
        return idx;
    }
    int res = (it == _map.end()) ? idx : it->second;
    hash_map<int, int>::iterator last = _map.find(_n);
    if (last == _map.end()) {
        _map.insert(std::make_pair(idx, _n));
    } else {
        _map.insert(std::make_pair(idx, last->second));
        _map.erase(last);
    }
    return res;
}

} //~namespace opencog
