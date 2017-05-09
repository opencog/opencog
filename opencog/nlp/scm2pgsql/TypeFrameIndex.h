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

    typedef std::set<int> IntegerSet;

private:

    typedef std::map<std::string, IntegerSet> StringMap;
    typedef std::vector<std::pair<int, int>> IntPairVector;
    typedef std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> IntPairPairVector;
    typedef std::map<TypeFrame, IntegerSet, TypeFrame::LessThan> PatternMap;
    typedef std::pair<int, std::vector<int>> ResultPair;

    bool DEBUG = true;
    bool TOPLEVEL_ONLY = true;
    std::vector<TypeFrame> frames;
    PatternMap occurrenceSet;
    IntegerSet occurrenceUniverse;

    void query(std::vector<ResultPair> &result, TypeFrame &keyExpression, StringMap &variableOccurrences, const TypeFrame &queryFrame, int cursor);
    void addPatternOccurrence(TypeFrame &pattern, int pos);
    std::vector<TypeFrame> computeSubPatterns(TypeFrame &baseFrame, int cursorn, int pos);
    void selectCurrentElement(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor);
    void buildConstraints(IntPairVector &constraints, StringMap &variableOccurrences);
    void buildJointConstraints(IntPairPairVector &constraints, std::vector<StringMap> &variableOccurrences);
    void buildQueryTerm(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor);
    void printForDebug(bool showNodeNames = false);
    void printRecursionResult(std::vector<ResultPair> &v);


public:

    TypeFrameIndex();

    // TODO Implement this
    ~TypeFrameIndex();

    bool addFromScheme(const std::string &scm, int offset);
    void query(IntegerSet &result, const std::string &queryScm);
    void query(IntegerSet &result, const TypeFrame &queryFrame);

    bool addFrame(TypeFrame &frame, int offset);
    TypeFrame getFrameAt(int index);
    void buildSubPatternsIndex();
};

}

#endif // _OPENCOG_TYPEFRAMEINDEX_H
