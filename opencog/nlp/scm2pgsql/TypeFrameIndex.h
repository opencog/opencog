/*
 * TypeFrameIndex.h
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

#ifndef _OPENCOG_TYPEFRAMEINDEX_H
#define _OPENCOG_TYPEFRAMEINDEX_H

#include <string>
#include "TypeFrame.h"

namespace opencog
{

/**
 *
 */
class TypeFrameIndex 
{

public:

    TypeFrameIndex();

    // TODO Implement this
    ~TypeFrameIndex();

    bool addFromScheme(const std::string &txt, int offset);
    bool addFrame(TypeFrame &frame, int offset);
    void buildSubPatternsIndex();

private:

    typedef std::set<int> IntegerSet;
    typedef std::map<TypeFrame, IntegerSet, TypeFrame::LessThan> PatternMap;

    bool DEBUG = false;
    std::vector<TypeFrame> frames;
    PatternMap occurrenceSet;

    void addPatternOccurrence(TypeFrame &pattern, int pos);
    std::vector<TypeFrame> computeSubPatterns(TypeFrame &baseFrame, int cursor);
    void printForDebug(bool showNodeNames = false);
};

}

#endif // _OPENCOG_TYPEFRAMEINDEX_H
