/*
 * opencog/persist/file/CompositeRenumber.cc
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

#include <stdlib.h>

#include <opencog/util/platform.h>

#include "CompositeRenumber.h"
#include "CoreUtils.h"

using namespace opencog;

void CompositeRenumber::updateVersionHandles(CompositeTruthValue &ctv,
                                             HandleMap<AtomPtr> *handles)
{
    VersionedTruthValueMap newVersionedTVs;
    for (VersionedTruthValueMap::const_iterator itr = ctv.versionedTVs.begin();
            itr != ctv.versionedTVs.end(); itr++) {
        VersionHandle key = itr->first;
        CoreUtils::updateHandle(&(key.substantive), handles);
        newVersionedTVs[key] = itr->second;
    }
    ctv.versionedTVs = newVersionedTVs;
}
