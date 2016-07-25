/*
 * PatternBranch.h
 *
 * Copyright (C) 2016 OpenCog Foundation
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

#ifndef _OPENCOG_PATTERNBRANCH_H
#define _OPENCOG_PATTERNBRANCH_H

#include "TypeFramePattern.h"
#include <opencog/atoms/base/Atom.h>

namespace opencog
{

class TypeFramePattern;

/**
 *
 */
class PatternBranch 
{

public:

    PatternBranch();
    ~PatternBranch();

private:

    Type type;
    Arity arity;
    TypeFramePattern *pattern;
};

}

#endif // _OPENCOG_PATTERNBRANCH_H
