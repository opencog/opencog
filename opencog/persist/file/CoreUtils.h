/*
 * opencog/persist/file/CoreUtils.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_CORE_UTILS_H_
#define _OPENCOG_CORE_UTILS_H_

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/HandleMap.h>

namespace opencog
{
/** \addtogroup grp_persist
 *  @{
 */

/** Module for including any core-specific common utilities */
class CoreUtils
{

public:

    /**
     * This method is used to translate an old handle to a new one mapped
     * in a hash table.
     *
     * Handle-remapping is very specific of the save-to-file, 
     * restore-from-file persistence code.  It does potentially 
     * dangerous things with UUID's, and can easily trash a perfectly
     * good hypergraph if misused. Its hard to debug. Caution!
     *
     * @param Handle which will be translated.
     * @param Table that maps from old to new handles.
     */
    static void updateHandle(Handle*, HandleMap<AtomPtr>*) throw (RuntimeException);
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_CORE_UTILS_H_
