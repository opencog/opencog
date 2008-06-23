/*
 * src/AtomSpace/VersionHandle.h
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

#ifndef _VERSIONHANDLE_H_
#define _VERSIONHANDLE_H_

#include "CoreUtils.h"

namespace opencog
{

enum IndicatorType {HYPOTHETICAL = 0, CONTEXTUAL, UNKNOWN};

struct VersionHandle {
    IndicatorType indicator;
    Handle substantive;

    // Default constructor, gets a NULL_VERSION_HANDLE.
    VersionHandle();
    VersionHandle(IndicatorType ind, Handle subs);

    static const char* indicatorToStr(IndicatorType) throw (InvalidParamException);
    static IndicatorType strToIndicator(const char*) throw (InvalidParamException);
};

#define NULL_VERSION_HANDLE VersionHandle()

struct hashVersionHandle {
    int operator()(VersionHandle vh) const;
};

struct eqVersionHandle {
    bool operator()(VersionHandle vh1, VersionHandle vh2) const;
};

#define isNullVersionHandle(vh) TLB::isInvalidHandle(vh.substantive)

} // namespace opencog

#endif //_VERSIONHANDLE_H_
