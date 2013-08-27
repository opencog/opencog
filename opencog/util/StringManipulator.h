/*
 * opencog/util/StringManipulator.h
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

#ifndef _OPENCOG_STRINGMANIPULATOR_H_
#define _OPENCOG_STRINGMANIPULATOR_H_

#include <string>
#include <vector>
#include <sstream>

namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

//! Returns a string from the given argument by using the << operator
template <typename T>
std::string toString(T data)
{
    std::ostringstream oss;
    oss << data;
    return oss.str();
}

//! container for string-related static methods
class StringManipulator {

public:

    //! Convert a string to upper case
    /**
     * @param str The string to be converted in upper cases
     * @return The string converted
     */
    static std::string toUpper(const std::string& str);

    //! Convert a string to lower case
    /**
     * @param str The string to be converted in lower cases
     * @return The string converted
     */
    static std::string toLower(const std::string& str);

    //! Clean the string
    static std::string clean(const std::string& str);

    //! Split the string according to the passed delimiter
    /**
     * @param str The string to be split
     * @param delimiter The delimiter used to split the string. Blank space
     *                  is default.
     * @return A vector containing the itens splited.
     */
    static std::vector<std::string> split(std::string& str, const std::string& delimiter = " ");

    //! Trim the given string
    static void trim(std::string& str);

    //! Check if a string is a number or not.
    /**
     * @param str The string to be checked
     * @return True if it is a number and false if not
     */
    static bool isNumber(const std::string& str);

}; // class

/** @}*/
}  // namespace

#endif /*_OPENCOG_STRINGMANIPULATOR_H_*/
