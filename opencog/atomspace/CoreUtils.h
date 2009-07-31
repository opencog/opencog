/*
 * opencog/atomspace/CoreUtils.h
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
#include <opencog/atomspace/HandleMap.h>
#include <opencog/util/exceptions.h>

namespace opencog
{

class Atom;

/** Module for including any core-specific common utilities */
class CoreUtils
{

public:

    /**
     * This method is used to translate an old handle to a new one mapped
     * in a hash table.
     *
     * @param Handle which will be translated.
     * @param Table that maps from old to new handles.
     */
    static void updateHandle(Handle *, HandleMap<Atom *> *) throw (RuntimeException);

    /**
     * Handle sort criterion used by qsort. It returns a negative value,
     * zero or a positive value if the first argument is respectively
     * smaller than, equal to, or larger then the second argument.
     *
     * @param The pointer to the first handle element.
     * @param The pointer to the second handle element.
     * @return A negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger than the
     * second argument.
     */
    static int handleCompare(const void*, const void*);

    /** XXX Deprecated, do not use in new code. */
    static int compare(Handle ha, Handle hb) {
        return Handle::compare(ha, hb);
    }
};

} // namespace opencog

#endif // _OPENCOG_CORE_UTILS_H_
