/*
 * src/util/misc.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_MISC_H
#define _OPENCOG_MISC_H

#include <iterator>
#include <functional>

#ifndef WIN32
#include <cxxabi.h>
#endif

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

/**
 * Counts the number of bits in 1 in the given unsigned long argument.
 */
unsigned int bitcount(unsigned long n);

template <typename _OutputIterator>
void tokenize(const std::string& str,
              _OutputIterator tokens,
              const std::string& delimiters = " ")
{

    // skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos) {
        // found a token, add it to the vector.
        *(tokens++) = str.substr(lastPos, pos - lastPos);
        // skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

template<typename _T>
struct safe_deleter : public std::unary_function<_T*&, void>
{
    void operator()(_T*& __ptr) {
        if (__ptr) {
            delete __ptr;
            __ptr = 0;
        }
    }
};

#ifndef WIN32
std::string demangle(const std::string& mangled);
#endif

/** @}*/
} // namespace opencog

#endif // _OPENCOG_MISC_H
