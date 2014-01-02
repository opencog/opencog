/*
 * opencog/atomspace/VersionHandle.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#ifndef _OPENCOG_VERSIONHANDLE_H
#define _OPENCOG_VERSIONHANDLE_H

#include <opencog/atomspace/Handle.h>
#include <opencog/util/exceptions.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

enum IndicatorType {HYPOTHETICAL = 1, CONTEXTUAL, UNKNOWN};

struct VersionHandle
{
    IndicatorType indicator;
    //! substantive is a Handle corresponding to the context or hypothesis
    //! (i.e.it would be the first argument of the context of hypothetical link)
    Handle substantive;

    //! Default constructor, gets a NULL_VERSION_HANDLE.
    VersionHandle();

    //! subs represents the context or hypothesis,
    //! not the atom to make the handle for
    VersionHandle(IndicatorType ind, Handle subs);
    VersionHandle( const VersionHandle& other );

    // Needed for comparison within vtree
    bool operator<(const VersionHandle &other) const;
    bool operator>(const VersionHandle &other) const;
    bool operator==(const VersionHandle &other) const;
    bool operator!=(const VersionHandle &other) const;
    VersionHandle& operator=( const VersionHandle& other );

    static const char* indicatorToStr(IndicatorType) throw (InvalidParamException);
    static IndicatorType strToIndicator(const char*) throw (InvalidParamException);

};

#define NULL_VERSION_HANDLE VersionHandle()

struct hashVersionHandle
{
    int operator()(VersionHandle vh) const;
};

std::size_t hash_value(VersionHandle const& b);

struct eqVersionHandle
{
    bool operator()(VersionHandle vh1, VersionHandle vh2) const;
};

#define isNullVersionHandle(vh) (vh.substantive == Handle::UNDEFINED)

} // namespace opencog

//overload of operator<< to print VersionHandle
namespace std { 
    inline std::ostream& operator<<(std::ostream& out,
                                    const opencog::VersionHandle& vh) {
    out << "(indicator=" << vh.indicatorToStr(vh.indicator)
        << ",substantive=" << vh.substantive << ")";
    return out;
}

/** @}*/
} // ~namespace std

#endif // _OPENCOG_VERSIONHANDLE_H
