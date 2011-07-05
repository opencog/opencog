/*
 * opencog/embodiment/Learning/behavior/BehaviorDescriptionMatcher.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef BEHAVIORDESCRIPTIONMATCHER_H
#define BEHAVIORDESCRIPTIONMATCHER_H

#include <vector>
#include <map>
#include <set>
#include <opencog/atomspace/AtomSpace.h>
#include "CompositeBehaviorDescription.h"
#include "BehaviorCategory.h"
#include "PredicateHandleSet.h"

namespace behavior
{

class BehaviorDescriptionMatcher
{

private:

    int sumAllSetsCount;
    std::set<PredicateHandleSet> allSets;
    std::map<PredicateHandleSet, int> allSetsCount;
    std::map<PredicateHandleSet, PredicateHandleSet> mapping;
    std::map<PredicateHandleSet, float> mappingSimilarity;
    std::map<PredicateHandleSet, std::vector<int> > setDistribution;
    std::map<PredicateHandleSet, long> upperBound;
    std::map<PredicateHandleSet, long> lowerBound;
    bool matricesBuilt;
    int*** categoryMatrices;
    std::map<PredicateHandleSet, unsigned int> matrixIndex;
    float** averageMatrix;
    AtomSpace *atomSpace; // used to retrieve similarity links (in buildMapping()

    void computeSetDistribution(BehaviorCategory &category);
    void buildMapping(BehaviorCategory &category, CompositeBehaviorDescription &behaviorDescription);
    void buildUnionOfPredicateSets(BehaviorCategory &category);
    float computeSetSimilarity(const PredicateHandleSet &set1, const PredicateHandleSet &set2) const;
    float intersectionOverUnion(const PredicateHandleSet &set1, const PredicateHandleSet &set2) const;
    float weightedIntersectionOverUnion(const PredicateHandleSet &set1, const PredicateHandleSet &set2) const;
    float computeSetsDistributionFitness(CompositeBehaviorDescription &behaviorDescription, bool relevanceFlag = false);
    float computeSetsDistributionRelevance(BehaviorCategory &category);
    void computeSucessorMatrix(BehaviorCategory &category);
    void buildSucessorMatrix(CompositeBehaviorDescription &bd, int **matrix, bool categoryFlag = false);
    void destroyMatrices(BehaviorCategory &category);
    float computeSucessorFitness(CompositeBehaviorDescription &bd, int index = -1);
    float computeSucessorRelevance(BehaviorCategory &category);
    void computeTimeLengthBoundaries(BehaviorCategory &category);
    float computeIntervalFitness(CompositeBehaviorDescription &behaviorDescription);
    float computeIntervalRelevance(BehaviorCategory &category);

    //WARNING : temporary replacement
    float computeHandleSetSimilarity(const PredicateHandleSet& ps1, const PredicateHandleSet& ps2) const;

public:

    // ***********************************************/
    // Constructors/destructors

    ~BehaviorDescriptionMatcher();
    BehaviorDescriptionMatcher(AtomSpace *atomSpace = NULL);

    // ***********************************************/
    // Public API

    void setAtomSpace(AtomSpace *atomSpace);

    /**
     * Given a category and a behavior description, this method computes the
     * odds that behavior description belongs to the category,
     * answering with a [0,1] float.
     *
     * The algorithm used in this method is described in OpenCog's wikipage
     * under the topic BehaviorSimilarityAlgorithm
     *
     * @param behaviorDescription CompositeBehaviorDescription to be evaluated
     * @param category BehaviorCategory to be considered
     */
    float computePertinenceDegree(BehaviorCategory &category, CompositeBehaviorDescription &behaviorDescription, bool timingRelevanceFlag = true);

    /**
    * like above but the category is replaced by a single
    * CompositeBehaviorDescription
    * This is a temporary replacement, it consider only sequence of events
    */
    float computePertinenceDegree(const CompositeBehaviorDescription &bd1,
                                  const CompositeBehaviorDescription &bd2,
                                  bool timingRelevanceFlag = true) const;

    // ***********************************************/
    // Debug/tests

    std::string toStringTestCase(BehaviorCategory &category, CompositeBehaviorDescription &behaviorDescription);
    std::string toStringAllSets(BehaviorCategory &category);
    std::string toStringMapping(BehaviorCategory &category, CompositeBehaviorDescription &behaviorDescription);
    std::string toStringSetDistribution(BehaviorCategory &category);
    std::string toStringSetDistributionFitness(BehaviorCategory &category, std::vector<CompositeBehaviorDescription> &v);
    std::string toStringMatrices(BehaviorCategory &category);
    std::string toStringSucessorFitness(BehaviorCategory &category, std::vector<CompositeBehaviorDescription> &v);
    std::string toStringBounds(BehaviorCategory &category);

}; // class
}  // namespace

#endif
