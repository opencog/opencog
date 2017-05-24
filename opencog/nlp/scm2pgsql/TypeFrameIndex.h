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
    typedef std::set<TypeFrame, TypeFrame::LessThan> TypeFrameSet;
    typedef std::map<std::string, TypeFrame> VarMapping;
    typedef std::pair<TypeFrameSet, VarMapping> ResultPair;

private:

    typedef std::map<std::string, IntegerSet> StringMap;
    typedef std::vector<std::pair<int, int>> IntPairVector;
    typedef std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> IntPairPairVector;
    typedef std::map<TypeFrame, IntegerSet, TypeFrame::LessThan> PatternMap;

    static const int OPERATOR_NOP;
    static const int OPERATOR_AND;
    static const int OPERATOR_OR;
    static const int OPERATOR_NOT;

    bool DEBUG = false;
    bool TOPLEVEL_ONLY = true;
    std::vector<TypeFrame> frames;
    PatternMap occurrenceSet;
    IntegerSet occurrenceUniverse;

    void query(std::vector<ResultPair> &result, TypeFrame &keyExpression, std::vector<VarMapping> &forbiddenMappings, int &logicOperator, const TypeFrame &queryFrame, int cursor, bool distinct);
    void addPatternOccurrence(TypeFrame &pattern, int pos);
    void addArity2Patterns(std::vector<TypeFrame> &answer, std::vector<TypeFrame> &recurseResult1, std::vector<TypeFrame> &recurseResult2, TypeFrame &baseFrame, int cursor);
    std::vector<TypeFrame> computeSubPatterns(TypeFrame &baseFrame, int cursorn, int pos);
    void selectCurrentElement(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor);
    void buildConstraints(IntPairVector &constraints, StringMap &variableOccurrences);
    void buildJointConstraints(IntPairPairVector &constraints, std::vector<StringMap> &variableOccurrences);
    void buildQueryTerm(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor);
    bool compatibleVarMappings(const VarMapping &map1, const VarMapping &map2, bool distinct);
    bool isForbiddenMapping(const VarMapping &mapping, const std::vector<VarMapping> &forbiddenVector);
    void typeFrameSetUnion(TypeFrameSet &answer, const TypeFrameSet &set1, const TypeFrameSet &set2);
    void varMappingUnion(VarMapping &answer, const VarMapping &map1, const VarMapping &map2);
    void permutation(std::vector<std::vector<int>> &answer, int *array, int current, int size);
    void addPermutations(std::vector<std::vector<int>> &answer, std::vector<int> base);
    void addSymmetrucPermutations(TypeFrameSet &answer, const TypeFrame &frame, unsigned int cursor);

    void printRecursionResult(const std::vector<ResultPair> &v) const;
    void printVarMapping(const VarMapping &map) const;
    void printTypeFrameSet(const TypeFrameSet &set) const;



public:

    static const int LIMIT_FOR_SYMMETRIC_LINKS_PERMUTATION;

    TypeFrameIndex();

    // TODO Implement this
    ~TypeFrameIndex();

    bool addFromScheme(const std::string &scm, int offset);
    void query(std::vector<ResultPair> &result, const std::string &queryScm, bool distinct = true);
    void query(std::vector<ResultPair> &result, const TypeFrame &queryFrame, bool distinct = true);

    bool addFrame(TypeFrame &frame, int offset);
    TypeFrame getFrameAt(int index);
    void buildSubPatternsIndex();

    void printForDebug(bool showNodeNames = false) const;
};

}

#endif // _OPENCOG_TYPEFRAMEINDEX_H
