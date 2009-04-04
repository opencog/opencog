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

#include <opencog/atomspace/CoreUtils.h>
#include <boost/functional/hash.hpp>

namespace opencog
{

enum IndicatorType {HYPOTHETICAL = 0, CONTEXTUAL, UNKNOWN};

struct VersionHandle {
    IndicatorType indicator;
    Handle substantive;

    // Default constructor, gets a NULL_VERSION_HANDLE.
    VersionHandle();
    VersionHandle(IndicatorType ind, Handle subs);

    // Needed for comparison within vtree
    bool operator<(const VersionHandle &other) const;
    bool operator>(const VersionHandle &other) const;
    bool operator==(const VersionHandle &other) const;
    bool operator!=(const VersionHandle &other) const;

    static const char* indicatorToStr(IndicatorType) throw (InvalidParamException);
    static IndicatorType strToIndicator(const char*) throw (InvalidParamException);
};

#define NULL_VERSION_HANDLE VersionHandle()

struct hashVersionHandle {
    int operator()(VersionHandle vh) const;
};

std::size_t hash_value(VersionHandle const& b);

struct eqVersionHandle {
    bool operator()(VersionHandle vh1, VersionHandle vh2) const;
};

#define isNullVersionHandle(vh) TLB::isInvalidHandle(vh.substantive)

} // namespace opencog

#endif // _OPENCOG_VERSIONHANDLE_H
