/** printContainer.h --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
 *
 * Author: Nil Geisweiller <nilg@nilg-laptop>
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


#ifndef _OPENCOG_PRINTCONTAINER_H
#define _OPENCOG_PRINTCONTAINER_H

#include <iterator>
#include <algorithm>
#include <iostream>

namespace opencog {
    
    /**
     * stream out all elements in [from, to( with delimiter
     * 'delimiter', opening with 'left' and closing with 'right'.
     *
     * For instance if N = {1, 2, 3, 4} then
     * ostreamContainer(std::cout, N.begin(), N.end(), "{", ";", "}")
     * displays "{1;2;3;4}"
     *
     * @param empty_rl is a flag indicating whether to print left and
     *                 right when the container is empty. If true then
     *                 they are always printed, if false then they
     *                 printed only when the container is not empty
     */
    template<class Out, class It>
    Out& ostreamContainer(Out& out,
                          It from,
                          It to,
                          const std::string& delimiter = " ",
                          const std::string& left = "",
                          const std::string& right = "",
                          bool empty_lr = true)
    {
        if(empty_lr || from!=to)
            out << left;
        if(from != to) {
            It last = --to;
            std::copy(from, last, std::ostream_iterator<typename It::value_type>(out, delimiter.c_str()));
            out << *last;
        }
        if(empty_lr || from!=to)
            out << right;
        return out;
    }

    /**
     * like above but consider the entire container
     */
    template<class Out, class Con>
    Out& ostreamContainer(Out& out,
                          const Con& container,
                          const std::string& delimiter = " ",
                          const std::string& left = "", 
                          const std::string& right = "",
                          bool empty_lr = true)
    {
        return ostreamContainer(out, container.begin(), container.end(),
                                delimiter, left, right, empty_lr);
    }

    /**
     * like ostreamContainer but uses std::cout
     */
    template<class It>
    void printContainer(It from,
                        It to,
                        const std::string& delimiter = " ",
                        const std::string& left = "",
                        const std::string& right = "",
                        bool empty_lr = true)
    {
        ostreamContainer(std::cout, from, to,
                         left, delimiter, right, empty_lr);
    }
    template<class Con>
    void printContainer(const Con& container,
                        const std::string& delimiter = " ",
                        const std::string& left = "", 
                        const std::string& right = "",
                        bool empty_lr = true)
    {
        ostreamContainer(std::cout, container,
                         left, delimiter, right, empty_lr);
    }

    /**
     * like above but returns a string
     */
    template<class It>
    std::string containerToStr(It from,
                               It to,
                               const std::string& delimiter = " ",
                               const std::string& left = "",
                               const std::string& right = "",
                               bool empty_lr = true)
    {
        std::stringstream ss;
        return ostreamContainer(ss, from, to,
                                left, delimiter, right, empty_lr).str();
    }
    template<class Con>
    std::string containerToStr(const Con& container,
                               const std::string& delimiter = " ",
                               const std::string& left = "", 
                               const std::string& right = "",
                               bool empty_lr = true)
    {
        std::stringstream ss;
        return ostreamContainer(ss, container,
                                left, delimiter, right, empty_lr).str();
    }
    
} // ~namespace opencog
#endif // _OPENCOG_PRINTCONTAINER_H
