/*
 * opencog/persist/file/CompositeRenumber.h
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

#ifndef _OPENCOG__COMPOSITE_RENUMBER_H_
#define _OPENCOG__COMPOSITE_RENUMBER_H_

#include <opencog/util/platform.h>

#include <opencog/atomspace/HandleMap.h>
#include <opencog/atomspace/CompositeTruthValue.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

class CompositeRenumber
{
public:
    /**
     * Updates all VersionHandles of the versioned TVs in this object
     * using the HandleMap passed as argument.
     * @param A HandleMap that maps old Handles to new ones.
     */
    typedef std::shared_ptr<HandleMap<AtomPtr>> HandMapPtr;
    static void updateVersionHandles(CompositeTruthValue&, HandMapPtr);
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG__COMPOSITE_RENUMBER_H_
