/* iostreamContainer.h --- 
 *
 * Copyright (C) 2010 OpenCog Foundation
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
#include <ctype.h>

#include <boost/lexical_cast.hpp>

#include "foreach.h"
#include "oc_assert.h"

namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

    /** @name IO stream container
     * functions to read, write, print or convert to string generic containers
     */
    ///@{
    
    
    /**
     * stream out all elements in [from, to( with delimiter
     * 'delimiter', opening with 'left' and closing with 'right'.
     *
     * For instance if N = {1, 2, 3, 4} then
     * ostreamContainer(std::cout, N.begin(), N.end(), ";", "{", "}")
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
            while(from != to) {
                out << *from;
                if(++from != to)
                    out << delimiter;
            }
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
     * like ostreamContainer but adding a new line at the end
     */
    template<class Out, class It>
    Out& ostreamlnContainer(Out& out,
                            It from,
                            It to,
                            const std::string& delimiter = " ",
                            const std::string& left = "",
                            const std::string& right = "",
                            bool empty_lr = true)
    {
        ostreamContainer(out, from, to, delimiter, left, right, empty_lr);
        out << std::endl;
        return out;
    }

    /**
     * like ostreamContainer but adding a new line at the end
     */
    template<class Out, class Con>
    Out& ostreamlnContainer(Out& out,
                            const Con& container,
                            const std::string& delimiter = " ",
                            const std::string& left = "", 
                            const std::string& right = "",
                            bool empty_lr = true)
    {
        ostreamContainer(out, container.begin(), container.end(),
                         delimiter, left, right, empty_lr);
        out << std::endl;
        return out;
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
                         delimiter, left, right, empty_lr);
    }
    template<class Con>
    void printContainer(const Con& container,
                        const std::string& delimiter = " ",
                        const std::string& left = "", 
                        const std::string& right = "",
                        bool empty_lr = true)
    {
        ostreamContainer(std::cout, container,
                         delimiter, left, right, empty_lr);
    }

    /**
     * like printContainer but with an endline at the end
     */
    template<class It>
    void printlnContainer(It from,
                          It to,
                          const std::string& delimiter = " ",
                          const std::string& left = "",
                          const std::string& right = "",
                          bool empty_lr = true)
    {
        ostreamlnContainer(std::cout, from, to,
                           delimiter, left, right, empty_lr);
    }
    template<class Con>
    void printlnContainer(const Con& container,
                          const std::string& delimiter = " ",
                          const std::string& left = "", 
                          const std::string& right = "",
                          bool empty_lr = true)
    {
        ostreamlnContainer(std::cout, container,
                           delimiter, left, right, empty_lr);
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
                                delimiter, left, right, empty_lr).str();
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
                                delimiter, left, right, empty_lr).str();
    }

    //! used by istreamContainer
    inline bool exists_white_space(const std::string& str) {
        foreach(const char& c, str) if(isspace(c)) return true;
        return false;
    }
    //! used by istreamContainer
    inline bool all_white_space(const std::string& str) {
        foreach(const char& c, str) if(!isspace(c)) return false;
        return true;
    }

    /**
     * used by istreamContainer to check that the provided delimiter,
     * left or right are well formed, that is do not
     * containwhite-space and non-white-space characters in the same
     * string
     */
    inline bool in_well_form(const std::string& str) {
        return !exists_white_space(str) || all_white_space(str);
    }

    /**
     * stream all elements in 'in' and put them in the container.
     * For instance 
     * std::stringstream ss("[1 2 3 4] 5");
     * std::vector<int> nums;
     * istreamContainer(ss, back_inserter(nums), "[", "]");
     * inserts 1 to 4 in nums.
     * @note it is assumed the delimiter is a white-space
     * @todo upgrade that function to work with any delimiter
     *
     * @param in        istream
     * @param out       container out iterator
     * @param left      left string expected
     * @param right     right string expected
     *
     * @note left or right should not contain white-space and
     * non-white-space characters in the same string, for instance ",
     * ." is forbiden.  The other assumption is that the string
     * contains at least one element.
     *
     * In case things or in are not as expected an OC_ASSERT is
     * raised.
     * @todo it may be more appropriate to throw an expection rather
     * than raising an OC_ASSERT
     */
    template<class In, class OutIt>
    In& istreamContainer(In& in,
                         OutIt out,
                         const std::string& left = "",
                         const std::string& right = "")
    {
        typedef typename OutIt::container_type::value_type T;

        OC_ASSERT(in_well_form(left));
        OC_ASSERT(in_well_form(right));

        std::string s;
        in >> s;
        OC_ASSERT(s.substr(0, left.size()) == left,
                  "left = %s is not a substring of s = %s",
                  left.c_str(), s.c_str());
        s = s.substr(left.size());
        *out++ = boost::lexical_cast<T>(s);

        while(!in.eof()) {
            in >> s;
            try {
                *out++ = boost::lexical_cast<T>(s);
            }
            catch(boost::bad_lexical_cast &)
            {
                // check if right is not appended to the last element
                int appended_pos = s.size() - right.size();
                OC_ASSERT(appended_pos > 0
                          && s.rfind(right) == (size_t)appended_pos);
                *out++ = boost::lexical_cast<T>(s.substr(0, appended_pos));
                break;
            }
        }
        return in;
    }
    
    ///@}

/** @}*/
} // ~namespace opencog
#endif // _OPENCOG_PRINTCONTAINER_H
