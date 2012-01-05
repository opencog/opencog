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
 * @todo for now only vector and list comprehensions are coded. 2 issues:
 * 1) no filter,
 * 2) can only use object function.
 * I'm pretty sure both issues can be addressed (and will try when time permits)
 */
    
/**
 * vector comprehension
 */
template<typename Container, typename Function>
std::vector<typename Function::result_type> vector_comp(const Container& c,
                                                        const Function& f)
{
    std::vector<typename Function::result_type> v;
    boost::transform(c, std::back_inserter(v), f);
    return v;
}

/**
 * list comprehension
 */
template<typename Container, typename Function>
std::list<typename Function::result_type> list_comp(const Container& c,
                                                    const Function& f)
{
    std::list<typename Function::result_type> l;
    boost::transform(c, std::back_inserter(l), f);
    return l;
}

}

#endif // _OPENCOG_COMPREHENSION_H


