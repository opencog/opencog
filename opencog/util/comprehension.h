/* comprehension.h --- 
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
#include <boost/range/adaptor/filtered.hpp>

#include <opencog/util/functional.h>

namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

/** @name Container comprehension constructors
 * I didn't find anything satisfactory here
 * http://en.wikipedia.org/wiki/List_comprehension#C.2B.2B
 *
 * Though the solution involing Boost.Range is nearly satisfactory, I
 * want to be able to construct and assing the list at once.
 *
 * The usage is extremely simple:
 *
 * auto res = {vector,list,set}_comp(container, function, filter);
 *
 * Or if not filter is necessary just omit it
 *
 * auto res = {vector,list,set}_comp(container, function);
 *
 * the results res will be of type std::vector, std::list or std::set
 * (as defined in the prefix of the *_comp), no matter the type of
 * container.
 *
 * function and filter can be STL functors or lambdas (but
 * not boost.pheonix).
 *
 * Of course the return type of filter must boolean.
 */
///@{


//! default filter (returns true)
typedef const_function<bool> const_bool;
static const const_bool default_filter(true);

    
//! vector comprehension (STL functor version)
template<typename Container, typename Function, typename Filter = const_bool>
auto vector_comp(const Container& c, const Function& func,
                 const Filter& filter = default_filter)
    -> std::vector<typename Function::result_type>
{
    std::vector<typename Function::result_type> v;
    boost::transform(c | boost::adaptors::filtered(filter),
                     std::back_inserter(v), func);
    return v;
}

//! vector comprehension (lambda version)
template<typename Container, typename Function, typename Filter = const_bool>
auto vector_comp(const Container& c, const Function& func,
                 const Filter& filter = default_filter)
    -> std::vector<decltype(func(std::declval<typename Container::value_type>()))>
{
    std::vector<decltype(func(std::declval<typename Container::value_type>()))> v;
    boost::transform(c | boost::adaptors::filtered(filter),
                     std::back_inserter(v), func);
    return v;
}

//! list comprehension (STL functor version)
template<typename Container, typename Function, typename Filter = const_bool>
auto list_comp(const Container& c, const Function& func,
               const Filter& filter = default_filter)
    -> std::list<typename Function::result_type>
{
    std::list<typename Function::result_type> l;
    boost::transform(c | boost::adaptors::filtered(filter),
                     std::back_inserter(l), func);
    return l;
}

//! list comprehension (lambda version)
template<typename Container, typename Function, typename Filter = const_bool>
auto list_comp(const Container& c, const Function& func,
               const Filter& filter = default_filter)
    -> std::list<decltype(func(std::declval<typename Container::value_type>()))>
{
    std::list<decltype(func(std::declval<typename Container::value_type>()))> l;
    boost::transform(c | boost::adaptors::filtered(filter),
                     std::back_inserter(l), func);
    return l;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Really weird for set_comp I'm getting the following error (gcc 4.7.3)                                                  //
//                                                                                                                        //
// [ 16%] Building CXX object opencog/comboreduct/CMakeFiles/comboreduct.dir/table/table_io.cc.o                          //
// In file included from /home/nilg/OpenCog/opencog/opencog/comboreduct/table/table_io.cc:36:0:                           //
// /home/nilg/OpenCog/opencog/opencog/util/comprehension.h:128:8: error: expected type-specifier                          //
// /home/nilg/OpenCog/opencog/opencog/util/comprehension.h:128:8: error: expected initializer                             //
// /home/nilg/OpenCog/opencog/opencog/util/comprehension.h:142:8: error: expected type-specifier                          //
// /home/nilg/OpenCog/opencog/opencog/util/comprehension.h:142:8: error: expected initializer                             //
//                                                                                                                        //
// I might be is due to having table_io.cc use both boost and std namespace...                                            //
//                                                                                                                        //
// Anyway it's weird enough that I'd rather not even try to attempt fix that for now, I think it might well be a gcc bug. //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
/// @todo set comprehension (STL functor version)
// template<typename Container, typename Function, typename Filter = const_bool>
// auto set_comp(const Container& c, const Function& func,
//               const Filter& filter = default_filter)
//     -> std::set<typename Function::result_type>
// {
//     std::set<typename Function::result_type> s;
//     boost::transform(c | boost::adaptors::filtered(filter),
//                      std::inserter(s, s.end()), func);
//     return s;
// }

/// @todo set comprehension (lambda version)
// template<typename Container, typename Function, typename Filter = const_bool>
// auto set_comp(const Container& c, const Function& func,
//               const Filter& filter = default_filter)
//     -> std::set<decltype(func(std::declval<typename Container::value_type>()))>
// {
//     std::set<decltype(func(std::declval<typename Container::value_type>()))> v;
//     boost::transform(c | boost::adaptors::filtered(filter),
//                      std::inserter(v, v.end()), func);
//     return v;
// }

}

///@}
/** @}*/

#endif // _OPENCOG_COMPREHENSION_H

