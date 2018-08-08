/*
 * SCMLoaderCallback.h
 *
 * Copyright (C) 2017 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
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

#ifndef _OPENCOG_SCMLOADERCALLBACK_H
#define _OPENCOG_SCMLOADERCALLBACK_H

#include <fstream>
#include <string>
#include <opencog/atoms/base/Handle.h>

namespace opencog
{

/**
 * This interface is supposed to be implemented by objects that will be notified
 * everytime (just before and just after) SCMLoader process and insert (into a
 * AtomSpace) a toplevel atom from the source file.
 */
class SCMLoaderCallback 
{

public:

    /**
     * This method is called just before inserting the atom (and possibly the
     * embedded atoms) defined in the passed Scheme source.
     *
     * @param schemeText The Scheme definition of the atom that is going to be
     * inserted
     */
    virtual void beforeInserting(const std::string &schemeText) = 0;

    /**
     * This method is called just after inserting a new toplevel atom from the
     * source schema file.
     *
     * @param toplevelAtom The handle of the toplevel atom inserted into the
     * AtomSpace
     */
    virtual void afterInserting(const Handle &toplevelAtom) = 0;
};

}

#endif // _OPENCOG_SCMLOADERCALLBACK_H
