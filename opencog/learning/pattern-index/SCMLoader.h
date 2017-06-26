/*
 * SCMLoader.h
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

#ifndef _OPENCOG_SCMLOADER_H
#define _OPENCOG_SCMLOADER_H

#include <fstream>
#include <string>
#include <opencog/atomspace/AtomSpace.h>

#include "SCMLoaderCallback.h"

namespace opencog
{

/**
 * A helper class that implements a simple strategy to load HUGE .scm
 * files. The issue with such huge files is that trying to load them directly in
 * guile or either pre-compiling them using "guild compile" requires an insane 
 * amount of RAM.
 *
 * To load the atoms from such huge .scm files, load() will read each 
 * top-level atom from the file and execute it using SchemeEval, populating the
 * passed AtomSpace properly.
 */
class SCMLoader 
{

public:

    /**
     * Load the contents of an .scm file and populates the passed
     * AtomSpace.
     *
     * @return true if the passed file is somehow unusable or false otherwise
     * @param fileName Full path of the file to be parsed
     * @param atomSpace The AtomSpace to be populated
     * @param callbackListener An object that will be notified just before and
     * just after each toplevel atom insertion.
     */
    static bool load(const std::string &fileName, AtomSpace &atomSpace, SCMLoaderCallback *callbackListener = NULL);

private:

    static void parseFile(std::fstream &fin, AtomSpace &atomSpace, int fileSize, SCMLoaderCallback *callbackListener);
};

}

#endif // _OPENCOG_SCMLOADER_H
