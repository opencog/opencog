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
#include <initializer_list>
#include <boost/operators.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/map.hpp>
#include <opencog/util/foreach.h>

namespace opencog {

using boost::adaptors::map_values;

/**
 * Class that mimics python Counter container
 */

template<typename T, typename CT>
struct Counter : public std::map<T, CT>,
    boost::addable<Counter<T, CT>>
{
    typedef std::map<T, CT> super;
    typedef typename super::value_type value_type;
    // this will be replaced by C++11 constructor delegation instead
    // of init
    template<typename IT>
    void init(IT from, IT to) {
        while(from != to) {
            this->operator[](*from) += 1;  // we don't use ++ to put the
                                           // least assumption on on CT
            ++from;
        }
    }
    Counter() {}
    template<typename IT>
    Counter(IT from, IT to) {
        init(from, to);
    }
    template<typename Container>
    Counter(const Container& c) {
        init(c.begin(), c.end());
    }
    Counter(const std::initializer_list<value_type>& il) {
        foreach(const auto& v, il)
            this->operator[](v.first) = v.second;
    }

    // return the count of a key, possibly returning a default if none
    // is present. That method different that operator[] because it
    // doesn't insert the element if it is not present. This is very
    // useful for multi-threading programming, and prevents data race
    // bug
    CT get(const T& key, CT c = CT()) const {
        typename super::const_iterator it = this->find(key);
        return it == this->cend()? c : it->second;
    }
    
    // return the total of all counted elements
    CT total_count() const {
        return boost::accumulate(*this | map_values, 0);
    }
    
    // add 2 counters, for example
    // c1 = {'a':1, 'b':1}
    // c2 = {'b':1, 'c':3}
    // after
    // c1 += c2
    // now
    // c1 = {'a':1, 'b':2, 'c':3}
    Counter& operator+=(const Counter& other) {
        foreach(const auto& v, other)
            this->operator[](v.first) += v.second;
        return *this;
    }
    /// @todo add method to subtract, multiply, etc Counters, or
    /// scalar and Counter, etc...
};

template<typename T, typename CT>
std::ostream& operator<<(std::ostream& out, const Counter<T, CT>& c) {
    typedef Counter<T, CT> counter_t;
    out << "{";
    for(typename counter_t::const_iterator it = c.begin(); it != c.end();) {
        out << it->first << ": " << it->second;
        ++it;
        if(it != c.end())
            out << ", ";
    }
    out << "}";
    return out;
}

} // ~namespace opencog

#endif // _OPENCOG_COUNTER_H
