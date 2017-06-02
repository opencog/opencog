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
#include <queue>
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
    typedef std::map<std::string, TypeFrame> VarMapping;
    typedef std::set<TypeFrame, TypeFrame::LessThan> TypeFrameSet;
    typedef std::pair<TypeFrameSet, VarMapping> ResultPair;
    typedef enum RankingMetric {I_SURPRISINGNESS, N_I_SURPRISINGNESS, II_SURPRISINGNESS, N_II_SURPRISINGNESS} RankingMetric;
    typedef enum CoherenceFunction {CONST_1} CoherenceFunction;
    typedef enum CoherenceModulatorG {ONE_OVER_COHERENCE} CoherenceModulatorG;
    typedef enum CoherenceModulatorH {COHERENCE} CoherenceModulatorH;

private:

    typedef std::map<std::string, IntegerSet> StringMap;
    typedef std::vector<std::pair<int, int>> IntPairVector;
    typedef std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> IntPairPairVector;
    typedef std::map<TypeFrame, IntegerSet, TypeFrame::LessThan> PatternMap;
    typedef std::pair<float, TypeFrame> WeightedFrame;
    struct WeightedFrameGreaterThan {
        bool operator()(const WeightedFrame &a, const WeightedFrame &b) const {
            return (a.first > b.first);
        }
    };
    typedef std::priority_queue<WeightedFrame, std::vector<WeightedFrame>, TypeFrameIndex::WeightedFrameGreaterThan> PatternHeap;
    typedef std::map<TypeFrame, int, TypeFrame::LessThan> PatternCountMap;
    typedef std::map<TypeFrame, std::pair<float, float>, TypeFrame::LessThan> SubPatternProbMap;
    typedef std::map<TypeFrame, float, TypeFrame::LessThan> PatternFloatMap;

    static const int OPERATOR_NOP;
    static const int OPERATOR_AND;
    static const int OPERATOR_OR;
    static const int OPERATOR_NOT;

    static CoherenceFunction coherenceFunction;
    static CoherenceModulatorG coherenceModulatorG;
    static CoherenceModulatorH coherenceModulatorH;

    bool DEBUG = false;
    bool LOCAL_DEBUG = true;

    TypeFrame auxVar1;

    bool TOPLEVEL_ONLY = true;
    std::vector<TypeFrame> frames;
    PatternMap occurrenceSet;
    IntegerSet occurrenceUniverse;
    PatternCountMap patternCountCache;
    SubPatternProbMap subPatternProbCache;
    PatternFloatMap patternQualityCache;
    float floatUniverseCount;

    void query(std::vector<ResultPair> &result, TypeFrame &keyExpression, std::vector<VarMapping> &forbiddenMappings, int &logicOperator, const TypeFrame &queryFrame, int cursor, bool distinct, bool noPermutations) const;
    void addPatternOccurrence(TypeFrame &pattern, int pos);
    void addArity2Patterns(std::vector<TypeFrame> &answer, std::vector<TypeFrame> &recurseResult1, std::vector<TypeFrame> &recurseResult2, TypeFrame &baseFrame, int cursor);
    std::vector<TypeFrame> computeSubPatterns(TypeFrame &baseFrame, int cursorn, int pos);
    void selectCurrentElement(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor) const;
    void buildConstraints(IntPairVector &constraints, StringMap &variableOccurrences) const;
    void buildQueryTerm(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor) const;
    bool compatibleVarMappings(const VarMapping &map1, const VarMapping &map2, bool distinct) const;
    bool mapCover(const VarMapping &map1, const VarMapping &map2) const;
    bool equivalentVarMappings(const VarMapping &map1, const VarMapping &map2, bool distinct) const;
    bool isForbiddenMapping(const VarMapping &mapping, const std::vector<VarMapping> &forbiddenVector) const;
    void typeFrameSetUnion(TypeFrameSet &answer, const TypeFrameSet &set1, const TypeFrameSet &set2) const;
    void varMappingUnion(VarMapping &answer, const VarMapping &map1, const VarMapping &map2) const;
    void permutation(std::vector<std::vector<int>> &answer, int *array, int current, int size);
    void addPermutations(std::vector<std::vector<int>> &answer, std::vector<int> base);
    void addSymmetrucPermutations(TypeFrameSet &answer, const TypeFrame &frame, unsigned int cursor);
    void buildCompoundFrames(std::vector<TypeFrame> &answer, int components) const;
    bool containsEquivalentFrame(const std::vector<TypeFrame> &v, const TypeFrame &f) const;
    void addPatterns(std::vector<TypeFrame> &answer, const TypeFrame &base) const;
    float computeQuality(const TypeFrame &pattern, RankingMetric metric);
    float computeISurprinsingness(const TypeFrame &pattern, bool normalized);
    float computeIISurprinsingness(const TypeFrame &pattern, bool normalized);
    unsigned int countPattern(const TypeFrame &pattern);
    void addSupersetFrames(std::vector<TypeFrame> &answer, const TypeFrame &frame) const;
    void addSubsetFrames(std::vector<TypeFrame> &subset, std::vector<TypeFrame> &star, const TypeFrame &frame) const;
    void addInferredSetFrames(std::vector<TypeFrame> &subset, std::vector<TypeFrame> &star, const TypeFrame &frame, bool subSetFlag) const;
    std::pair<float,float> minMaxIndependentProb(const TypeFrame &pattern);
    std::pair<float,float> minMaxSubsetProb(const TypeFrame &pattern);
    std::pair<float,float> minMaxSupersetProb(const TypeFrame &pattern);
    float computeCoherence(const TypeFrame &frame) const;
    float gFunction(float x) const;
    float hFunction(float x) const;
    float one_over_x(float x) const;

    void printFrameVector(const std::vector<TypeFrame> &v) const;
    void printRecursionResult(const std::vector<ResultPair> &v) const;
    void printVarMapping(const VarMapping &map) const;
    void printTypeFrameSet(const TypeFrameSet &set) const;

public:

    static unsigned int MINIMAL_FREQUENCY_TO_COMPUTE_SURPRISINGNESS;
    static unsigned int LIMIT_FOR_SYMMETRIC_LINKS_PERMUTATION;
    static bool PATTERN_COUNT_CACHE_ENABLED;
    static bool INDEPENDENT_SUBPATTERN_PROB_CACHE_ENABLED;
    static bool PATTERN_QUALITTY_CACHE_ENABLED;

    TypeFrameIndex();

    // TODO Implement this
    ~TypeFrameIndex();

    bool addFromScheme(const std::string &scm, int offset);
    // distinct enforces that to different variables are assigned different
    // TypeFrames
    void query(std::vector<ResultPair> &result, const std::string &queryScm, bool distinct = true, bool noPermutations = false) const;
    void query(std::vector<ResultPair> &result, const TypeFrame &queryFrame, bool distinct = true, bool noPermutations = false) const;

    bool addFrame(TypeFrame &frame, int offset);
    TypeFrame getFrameAt(int index);
    void buildSubPatternsIndex();
    void minePatterns(std::vector<std::pair<float,TypeFrame>> &answer, unsigned int components, unsigned int maxAnswers, RankingMetric metric);

    void printForDebug(bool showNodeNames = true) const;
};

}

#endif // _OPENCOG_TYPEFRAMEINDEX_H
