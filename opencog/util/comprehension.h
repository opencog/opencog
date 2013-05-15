/** comprehension.h --- 
 *
 * Copyright (C) 2012 Nil Geisweiller
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_COMPREHENSION_H
#define _OPENCOG_COMPREHENSION_H

#include <boost/range/algorithm/transform.hpp>

namespace opencog {

/**
 * Container comprehension constructors. I didn't find anything satisfactory here
 * http://en.wikipedia.org/wiki/List_comprehension#C.2B.2B
 *
 * Though the solution involing Boost.Range is nearly satisfactory, I
 * want to be able to construct and assing the list at once.
 *
 * The usage is extremely simple:
 *
 * auto res = {vector,list,set}_comp(container, function, filter);
 *
 * the results res will be of type std::vector, std::list or std::set
 * (as defined in the prefix of the *_comp), no matter the type of
 * container.
 *
 * function and filter can be STL functors or lambdas (but
 * not boost.pheonix).
 *
 * Of course the return type of filter must boolean.
 *
 * TODO: implement filter!
 */
    
/**
 * vector comprehension (STL functor version)
 */
template<typename Container, typename Function>
auto vector_comp(const Container& c, const Function& f)
    -> std::vector<typename Function::result_type>
{
    std::vector<typename Function::result_type> v;
    boost::transform(c, std::back_inserter(v), f);
    return v;
}

/**
 * vector comprehension (lambda version)
 */
template<typename Container, typename Function>
auto vector_comp(const Container& c, const Function& f)
    -> std::vector<decltype(f(std::declval<typename Container::value_type>()))>
{
    std::vector<decltype(f(std::declval<typename Container::value_type>()))> v;
    boost::transform(c, std::back_inserter(v), f);
    return v;
}

/**
 * list comprehension (STL functor version)
 */
template<typename Container, typename Function>
auto list_comp(const Container& c, const Function& f)
    -> std::list<typename Function::result_type>
{
    std::list<typename Function::result_type> l;
    boost::transform(c, std::back_inserter(l), f);
    return l;
}

/**
 * list comprehension (lambda version)
 */
template<typename Container, typename Function>
auto list_comp(const Container& c, const Function& f)
    -> std::list<decltype(f(std::declval<typename Container::value_type>()))>
{
    std::list<decltype(f(std::declval<typename Container::value_type>()))> l;
    boost::transform(c, std::back_inserter(l), f);
    return l;
}

/**
 * set comprehension (STL functor version)
 */
template<typename Container, typename Function>
auto set_comp(const Container& c, const Function& f)
    -> std::set<typename Function::result_type>
{
    std::set<typename Function::result_type> v;
    boost::transform(c, std::inserter(v, v.end()), f);
    return v;
}

/**
 * set comprehension (lambda version)
 */
template<typename Container, typename Function>
auto set_comp(const Container& c, const Function& f)
    -> std::set<decltype(f(std::declval<typename Container::value_type>()))>
{
    std::set<decltype(f(std::declval<typename Container::value_type>()))> v;
    boost::transform(c, std::inserter(v, v.end()), f);
    return v;
}

}

#endif // _OPENCOG_COMPREHENSION_H

