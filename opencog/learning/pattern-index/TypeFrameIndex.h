/*
 * TypeFrameIndex.h
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

#ifndef _OPENCOG_TYPEFRAMEINDEX_H
#define _OPENCOG_TYPEFRAMEINDEX_H

#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include "TypeFrame.h"
#include "PatternHeap.h"

namespace opencog
{

/*
 * Implementation of the index abstraction. 
 *
 * The index is actually a kind of "inverted list" of patterns pointing to the
 * original data. Current implementation store original data (a set of TypeFrame
 * wich is a representation of an atom) as a vector so the inverted list points
 * to position is this vector. This could be easily changed to store original
 * data in disk and make the inverted list point to location of actual data in
 * the disk.
 *
 * The simple trick used to build the inverted list is this. When a TypeFrame
 * (e.g. a Link) is inserted in the index, all the possible patterns that
 * matches this TypeFrame are computed and added to to index pointing to the
 * given TypeFrame.
 *
 * For example, if the following TypeFrame
 *
 * (EvaluationLink
 *   (PredicateNode "stateOf")
 *   (ListLink
 *     (ConceptNode "Florida")
 *     (COnceptNode "USA")
 *   )
 * )
 *
 * is inserted in the index. All the following patterns are recorded in the
 * inverted list pointing to this Typeframe.
 *
 * (EvaluationLink
 *   *
 *   *
 * )
 *
 * (EvaluationLink
 *   (PredicateNode "stateOf")
 *   *
 * )
 *
 * (EvaluationLink
 *   *
 *   (ListLink
 *     (ConceptNode "Florida")
 *     (ConceptNode "USA")
 *   )
 * )
 *
 * (EvaluationLink
 *   (PredicateNode "stateOf")
 *   (ListLink
 *     *
 *     (ConceptNode "USA")
 *   )
 * )
 *
 * (EvaluationLink
 *   (PredicateNode "stateOf")
 *   (ListLink
 *     (ConceptNode "Florida")
 *     *
 *   )
 * )
 *
 * (EvaluationLink
 *   *
 *   (ListLink
 *     *
 *     (ConceptNode "USA")
 *   )
 * )
 *
 * (EvaluationLink
 *   *
 *   (ListLink
 *     (ConceptNode "Florida")
 *     *
 *   )
 * )
 *
 * To avoid combinatorial explosion, only arity-2 links have all its patterns
 * possibilities explored this way (this could be easily extended to cover
 * greater arity links).
 *
 * TODO: consider removing the querying and mining algorithms to specialized
 * classes.
 */
class TypeFrameIndex 
{

public:

    typedef std::set<int> IntegerSet;
    typedef std::map<std::string, TypeFrame> VarMapping;
    typedef std::vector<VarMapping> VarMappingSeq;
    typedef std::set<TypeFrame, TypeFrame::LessThan> TypeFrameSet;
    typedef std::set<TypeFrame, TypeFrame::LessThanUsingEquivalence> EquivalentTypeFrameSet;
    typedef std::pair<TypeFrameSet, VarMapping> ResultPair;

    // TODO 
    // This is not C++ style. Change these to use struct's () operator
    typedef enum RankingMetric {I_SURPRISINGNESS, N_I_SURPRISINGNESS, II_SURPRISINGNESS, N_II_SURPRISINGNESS} RankingMetric;
    typedef enum CoherenceFunction {CONST_1} CoherenceFunction;
    typedef enum CoherenceModulatorG {ONE_OVER_COHERENCE} CoherenceModulatorG;
    typedef enum CoherenceModulatorH {COHERENCE} CoherenceModulatorH;
    // more public stuff below

private:

    typedef std::map<std::string, IntegerSet> StringMap;
    typedef std::vector<std::pair<int, int>> IntPairVector;
    typedef std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> IntPairPairVector;
    typedef std::map<TypeFrame, IntegerSet, TypeFrame::LessThan> PatternMap;
    //typedef std::unordered_map<TypeFrame, IntegerSet, TypeFrame::HashCode, TypeFrame::EqualsTo> PatternMap;
    typedef std::map<TypeFrame, int, TypeFrame::LessThan> PatternCountMap;
    typedef std::map<TypeFrame, std::pair<float, float>, TypeFrame::LessThan> SubPatternProbMap;
    typedef std::map<TypeFrame, float, TypeFrame::LessThan> PatternFloatMap;

    static const int OPERATOR_NOP;
    static const int OPERATOR_AND;
    static const int OPERATOR_OR;
    static const int OPERATOR_NOT;

    bool DEBUG = false;
    bool LOCAL_DEBUG = false;

    std::mutex miningResultsMutex;
    std::mutex compoundFrameMutex;
    std::queue<TypeFrame> compoundFrameQueue;
    std::vector<std::thread *> evaluationThreads;
    std::vector<bool> threadEnabled;
    bool compoundFramesEnded;
    PatternHeap miningResultsHeap;
    TypeFrame auxVar1;
    bool TOPLEVEL_ONLY = false;
    std::vector<TypeFrame> frames;
    PatternMap occurrenceSet;
    IntegerSet occurrenceUniverse;
    PatternCountMap patternCountCache;
    float floatUniverseCount;
    double time1, time2;

    void query(std::vector<ResultPair> &result,
               const TypeFrame &queryFrame,
               bool distinct,
               bool noPermutations) const;
    void query(std::vector<ResultPair> &result,
               TypeFrame &keyExpression,
               VarMappingSeq &forbiddenMappings,
               int &logicOperator,
               const TypeFrame &queryFrame,
               int cursor, bool distinct,
               bool noPermutations) const;
    void addPatternOccurrence(TypeFrame &pattern, int pos);
    void addSymmetricPermutations(TypeFrameSet &answer,
                                  const TypeFrame &frame,
                                  unsigned int cursor);
    void addArity2Patterns(std::vector<TypeFrame> &answer,
                           std::vector<TypeFrame> &recurseResult1,
                           std::vector<TypeFrame> &recurseResult2,
                           const TypeFrame &baseFrame,
                           int cursor);
    std::vector<TypeFrame> computeSubPatterns(const TypeFrame &baseFrame,
                                              int cursorn,
                                              int pos);
    void selectCurrentElement(TypeFrame &answer,
                              StringMap &variableOccurrences,
                              const TypeFrame &baseFrame,
                              int cursor) const;
    void buildConstraints(IntPairVector &constraints,
                          StringMap &variableOccurrences) const;
    void buildQueryTerm(TypeFrame &answer,
                        StringMap &variableOccurrences,
                        const TypeFrame &baseFrame,
                        int cursor) const;
    bool compatibleVarMappings(const VarMapping &map1,
                               const VarMapping &map2,
                               bool distinct) const;

	/**
	 * Return true iff all values (TypeFrames) of map2 are in map1
	 */
    bool mapCover(const VarMapping &map1,
                  const VarMapping &map2) const;
    bool equivalentVarMappings(const VarMapping &map1,
                               const VarMapping &map2,
                               bool distinct) const;
    bool isForbiddenMapping(const VarMapping &mapping,
                            const VarMappingSeq &forbiddenVector) const;
    void typeFrameSetUnion(TypeFrameSet &answer,
                           const TypeFrameSet &set1,
                           const TypeFrameSet &set2) const;
    void varMappingUnion(VarMapping &answer,
                         const VarMapping &map1,
                         const VarMapping &map2) const;
    void addPatterns(std::vector<TypeFrame> &answer,
                     const TypeFrame &base) const;
    float computeQuality(const TypeFrame &pattern);
    float computeISurprinsingness(const TypeFrame &pattern, bool normalized);
    float computeIISurprinsingness(const TypeFrame &pattern, bool normalized);
    unsigned int countPattern(const TypeFrame &pattern);
    void addSupersetFrames(std::vector<TypeFrame> &answer,
                           const TypeFrame &frame) const;
    void addSubsetFrames(std::vector<TypeFrame> &subset,
                         std::vector<TypeFrame> &star,
                         const TypeFrame &frame) const;
    void addInferredSetFrames(std::vector<TypeFrame> &subset,
                              std::vector<TypeFrame> &star,
                              const TypeFrame &frame,
                              bool subSetFlag) const;
    std::pair<float, float> minMaxIndependentProb(const TypeFrame &pattern);
    std::pair<float, float> minMaxSubsetProb(const TypeFrame &pattern);
    std::pair<float, float> minMaxSupersetProb(const TypeFrame &pattern);
    bool enqueueCompoundFrame(const TypeFrame &compoundFrame);
    bool dequeueCompoundFrame(TypeFrame &compoundFrame);
    void setCompoundFramesEnded();
    bool checkCompoundPatternsEnded();
    void addMiningResult(float quality, const TypeFrame &frame);
    float computeCoherence(const TypeFrame &frame) const;
    float gFunction(float x) const;
    float hFunction(float x) const;
    float one_over_x(float x) const;

    void printFrameVector(const std::vector<TypeFrame> &v) const;
    void printRecursionResult(const std::vector<ResultPair> &v) const;
    void printVarMapping(const VarMapping &map) const;
    void printTypeFrameSet(const TypeFrameSet &set) const;

public:

    unsigned int LIMIT_FOR_UNORDERED_LINKS_PERMUTATION;
    unsigned int MINIMAL_FREQUENCY_TO_COMPUTE_QUALITY_METRIC;
    unsigned int MAX_SIZE_OF_COMPOUND_FRAMES_QUEUE;
    unsigned int NUMBER_OF_EVALUATION_THREADS;
    unsigned int PATTERNS_GRAM;
    unsigned int MAXIMUM_NUMBER_OF_MINING_RESULTS;
    bool PATTERN_COUNT_CACHE_ENABLED;
    bool DIFFERENT_ASSIGNMENT_FOR_DIFFERENT_VARS;
    bool PERMUTATIONS_OF_VARS_CONSIDERED_EQUIVALENT;
    CoherenceFunction COHERENCE_FUNCTION;
    CoherenceModulatorG COHERENCE_MODULATOR_G;
    CoherenceModulatorH COHERENCE_MODULATOR_H;
    RankingMetric PATTERN_RANKING_METRIC;
    std::set<Type> ALLOWED_TOP_LEVEL_TYPES;
    std::set<Type> ALLOWED_VAR_SUBSTITUTION;

    TypeFrameIndex();
    ~TypeFrameIndex();

    bool addFromScheme(const std::string &scm, int offset);
    bool add(Handle handle, int offset);
    void query(std::vector<ResultPair> &result, const std::string &queryScm) const;
    void query(std::vector<ResultPair> &result, const TypeFrame &queryFrame) const;

    bool addFrame(TypeFrame &frame, int offset);
    TypeFrame getFrameAt(int index);
    void buildSubPatternsIndex();
    void evaluatePatterns();
    void minePatterns(std::vector<std::pair<float,TypeFrame>> &answer);

    void printForDebug(bool showNodeNames = true) const;
};

}

#endif // _OPENCOG_TYPEFRAMEINDEX_H
