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
#include <vector>
#include <string>

namespace opencog
{

class PatternBranch;

/**
 *
 */
class TypeFramePattern 
{

public:

    TypeFramePattern();
    ~TypeFramePattern();
    void add(TypeFrame &frame);
    void printForDebug(std::string indent, std::string prefix, bool showNames = false);


    /*
    typedef std::map<std::pair<Type, Arity>, 
             TypeFramePattern *, 
             struct 
             {
                bool operator()(const std::pair<Type, Arity> &a, const std::pair<Type, Arity> &b) const {
                    (a.first < b.first) or ((a.first == b.first) and (a.second < b.second))
                }
             }> BranchMap;
             */


private:

    typedef std::vector<class TypeFramePattern *> BranchVector;
    typedef std::map<TypeFrame::TypeVector, BranchVector> TypeVectorMap;

    void recursiveAdd(TypeFrame &frame, int cursor);
    bool DEBUG = false;

    TypeVectorMap branches;
};

}

#endif // _OPENCOG_TYPEFRAMEPATTERN_H
