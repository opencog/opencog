/** Counter.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_COUNTER_H
#define _OPENCOG_COUNTER_H

#include <map>

namespace opencog {

/**
 * Class that mimics python Counter container
 */

template<typename T, typename CT>
struct Counter : public std::map<T, CT> {
    typedef std::map<T, CT> super;
    Counter() {}
    template<typename IT>
    Counter(IT from, IT to) {
        while(from != to) {
            operator[](*from) += 1;  // we don't use ++ to put the
                                     // least assumption on on CT
        }
    }
    /// @todo add method to add subtract, multiply, etc Counters, or
    /// scalar and Counter, etc...
};

} // ~namespace opencog

#endif // _OPENCOG_COUNTER_H
