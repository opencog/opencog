/*
 * TypeFramePattern.h
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

#ifndef _OPENCOG_TYPEFRAMEPATTERN_H
#define _OPENCOG_TYPEFRAMEPATTERN_H

#include "PatternBranch.h"
#include "TypeFrame.h"
#include "TypeFrameIndex.h"
#include <vector>
#include <string>

namespace opencog
{

class PatternBranch;
class TypeFrameIndex;

/**
 *
 */
class TypeFramePattern 
{

public:

    std::vector<int> occurrences;

    TypeFramePattern();
    ~TypeFramePattern();
    void add(TypeFrame &frame, int offset);
    void buildSubPatternsIndex(TypeFrameIndex *index, TypeFrame &pattern);
    void printForDebug(std::string indent, std::string prefix, bool showNames = false);

private:

    typedef std::vector<class TypeFramePattern *> BranchVector;
    typedef std::map<TypeFrame, BranchVector> TypeFrameMap;
    typedef std::map<std::string, class TypeFramePattern *> NodeNameMap;

    bool DEBUG = false;
    TypeFrameMap linkBranches;
    NodeNameMap nodeBranches;

    void recursiveAdd(TypeFrame &frame, int offset, int cursor);
};

}

#endif // _OPENCOG_TYPEFRAMEPATTERN_H
