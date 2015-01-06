/*
 * opencog/embodiment/Learning/behavior/BehaviorDescriptionMatcher.cc
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

#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>
#include "BehaviorDescriptionMatcher.h"
#include "EvaluationLinkSimilarityEvaluator.h"
#include <math.h>

//WARNING : temporary just for debug
#include <opencog/atomspace/Node.h>
//if the sets are empty then there improtance in the score
//similarity of pauses must be less important
//than action similarity because otherwise the pauses between the action
//create massive local optima since a lot of programs using loops
//can score easily
//The coef 1.0/20.0 is obtained at nose there might be a better one
#define PAUSE_COEF 1.0/20.0

using namespace behavior;

BehaviorDescriptionMatcher::~BehaviorDescriptionMatcher()
{
}

BehaviorDescriptionMatcher::BehaviorDescriptionMatcher(AtomSpace *atomSpace)
{
    this->atomSpace = atomSpace;
    matricesBuilt = false;
}

void BehaviorDescriptionMatcher::destroyMatrices(BehaviorCategory &category)
{

    std::vector<CompositeBehaviorDescription> bds = category.getEntries();

    if (matricesBuilt) {
        for (unsigned int i = 0; i < bds.size(); i++) {
            for (unsigned int j = 0; j < allSets.size(); j++) {
                delete [] categoryMatrices[i][j];
            }
            delete [] categoryMatrices[i];
        }
        delete [] categoryMatrices;
        for (unsigned int j = 0; j < allSets.size(); j++) {
            delete [] averageMatrix[j];
        }
        delete [] averageMatrix;
        matricesBuilt = false;
    }
}

float BehaviorDescriptionMatcher::computePertinenceDegree(BehaviorCategory &category, CompositeBehaviorDescription &behaviorDescription, bool timingRelevanceFlag)
{

    buildMapping(category, behaviorDescription);

    // Sets (or chunks/pred-chunks in Ben's terminology) which apperar in BD and category

    computeSetDistribution(category);
    float f1 = computeSetsDistributionFitness(behaviorDescription);
    float w1 = computeSetsDistributionRelevance(category);

    // Relative order among sets in each BD

    computeSucessorMatrix(category);
    float f2 = computeSucessorFitness(behaviorDescription);
    float w2 = computeSucessorRelevance(category);
    destroyMatrices(category);

    // Time length of intervals

    float f3;
    float w3;
    if (timingRelevanceFlag) {
        computeTimeLengthBoundaries(category);
        f3 = computeIntervalFitness(behaviorDescription);
        w3 = computeIntervalRelevance(category);
    } else {
        f3 = 1;
        w3 = 0;
    }

    float answer = ((w1 * f1) + (w2 * f2) + (w3 * f3)) / (w1 + w2 + w3);

    //printf("w1 = %f\n", w1);
    //printf("w2 = %f\n", w2);
    //printf("w3 = %f\n", w3);
    //printf("f1 = %f\n", f1);
    //printf("f2 = %f\n", f2);
    //printf("f3 = %f\n", f3);
    //printf("answer = %f\n", answer);

    return answer;
}

//********************************************************************************
//WARNING : temporary replacement

float BehaviorDescriptionMatcher::computePertinenceDegree(const CompositeBehaviorDescription &bd1, const CompositeBehaviorDescription &bd2, bool timingRelevanceFlag) const
{
    //WARNING : this a temporary replacement of the real computePertinenceDegree until the bug is fixed
    std::vector<PredicateHandleSet> tls1 = bd1.getTimelineSets();
    std::vector<PredicateHandleSet> tls2 = bd2.getTimelineSets();
    unsigned s1 = tls1.size();
    unsigned s2 = tls2.size();
    float acc = 0; //the accumlated similarity score before being normalized to correspond to f
    float f = 0;
    unsigned pause_count = 0;
    OC_ASSERT(s1 > 0 || s2 > 0, "tls1 or tls2 should have at least one 'PredicateHandleSet'.");
    if (s1 > s2) {
        std::vector<PredicateHandleSet>::iterator i1 = tls1.begin();
        std::vector<PredicateHandleSet>::iterator i2 = tls2.begin();
        for (; i2 != tls2.end(); ++i1, ++i2) {
            if (i1->empty() && i2->empty())
                pause_count++;
            else acc += computeHandleSetSimilarity(*i1, *i2);
        }
        //Some explanation :
        //
        //in order to have f == 1.0 when the score is perfect
        //f needs to be normalized to compensate pauses because they
        //have PAUSE_COEF score instead of 1.0
        //
        //so :
        //
        //f = (acc + pause_count*PAUSE_COEF)
        //  / ((s1 - pause_count) + pause_count*PAUSE_COEF)
        //
        //that is :
        //
        //f = (acc + pause_count*PAUSE_COEF)
        //  / (s1 + pause_count*(PAUSE_COEF - 1.0))
        f = (acc + pause_count * PAUSE_COEF)
            / (s1 + pause_count * (PAUSE_COEF - 1.0));
    } else {
        std::vector<PredicateHandleSet>::iterator i2 = tls2.begin();
        for (std::vector<PredicateHandleSet>::iterator i1 = tls1.begin(); i1 != tls1.end(); ++i1, ++i2) {
            if (i1->empty() && i2->empty())
                pause_count++;
            else acc += computeHandleSetSimilarity(*i1, *i2);
        }
        //Some explanation (similar to the above one) :
        //
        //in order to have f == 1.0 when the score is perfect
        //f needs to be normalized to compensate pauses because they
        //have PAUSE_COEF score instead of 1.0
        //
        //so :
        //
        //f = (acc + pause_count*PAUSE_COEF)
        //  / ((s1 - pause_count) + pause_count*PAUSE_COEF)
        //
        //that is :
        //
        //f = (acc + pause_count*PAUSE_COEF)
        //  / (s1 + pause_count*(PAUSE_COEF - 1.0))
        f = (acc + pause_count * PAUSE_COEF)
            / (s2 + pause_count * (PAUSE_COEF - 1.0));
    }
    return f;
}

float BehaviorDescriptionMatcher::computeHandleSetSimilarity(const PredicateHandleSet& ps1, const PredicateHandleSet& ps2) const
{
    unsigned s1 = ps1.getSize();
    unsigned s2 = ps2.getSize();
    //print for debug
    //for (Handle h : ps1.getSet()) {
    //  std::cout << "PS1 H : " << h->toString() << std::endl;
    //}
    //for (Handle h : ps2.getSet()) {
    //  std::cout << "PS2 H : " << h->toString() << std::endl;
    //}
    OC_ASSERT(s1 == 0 || s1 == 1,
                     "PredicateHandleSet ps1 should have 0 or 1 item.");
    OC_ASSERT(s2 == 0 || s2 == 1,
                     "PredicateHandleSet ps2 should have 0 or 1 item.");
    if (s1 == 0 && s2 == 0)
        return 1.0;
    else if ( (s1 == 1 && s2 == 0) || (s1 == 0 && s2 == 1) )
        return 0.0;
    else { //s1==1 && s2==1
        Handle h1 = *ps1.getSet().begin();
        Handle h2 = *ps2.getSet().begin();

        OC_ASSERT(atomSpace->getType(h1) == EVALUATION_LINK && atomSpace->getArity(h1) == 2,
                         "Handle h1 should be an 'EVALUATION_LINK' and have arity 2.");
        OC_ASSERT(atomSpace->getType(h2) == EVALUATION_LINK && atomSpace->getArity(h2) == 2,
                         "Handle h2 should be an 'EVALUATION_LINK' and have arity 2.");

        Handle hargs1 = atomSpace->getOutgoing(h1, 1);
        Handle hargs2 = atomSpace->getOutgoing(h2, 1);

        OC_ASSERT(atomSpace->getType(hargs1) == LIST_LINK && atomSpace->getArity(hargs1) >= 2,
                         "Handle hargs1 should be an 'LIST_LINK' and have arity 2.");
        OC_ASSERT(atomSpace->getType(hargs2) == LIST_LINK && atomSpace->getArity(hargs2) >= 2,
                         "Handle hargs2 should be an 'LIST_LINK' and have arity 2.");

        Handle hact1 = atomSpace->getOutgoing(hargs1, 1);
        Handle hact2 = atomSpace->getOutgoing(hargs2, 1);

        //print for debug
        //std::cout << "act1 : " << hact1->toString() << std::endl;
        //std::cout << "act2 : " << hact2->toString() << std::endl;
        //~print for debug

        OC_ASSERT( atomSpace->isNode(hact1), "hact1 is not a 'Node'");
        OC_ASSERT( atomSpace->isNode(hact2), "hact2 is not a 'Node'");

        if (atomSpace->getName(hact1) == atomSpace->getName(hact2)) {
            int a1 = atomSpace->getArity(hargs1);
            int a2 = atomSpace->getArity(hargs2);
            if (a1 == a2) {
                for (int i = 2; i < a1; ++i) {
                    //compare action parameters
                    Handle arg_p_1 = atomSpace->getOutgoing(hargs1, i);
                    Handle arg_p_2 = atomSpace->getOutgoing(hargs2, i);
                    //we only compare if there type is not NUMBER_NODE
                    if (atomSpace->getType(arg_p_1) != NUMBER_NODE
                            && atomSpace->getType(arg_p_2) != NUMBER_NODE
                            && atomSpace->getName(arg_p_1) != atomSpace->getName(arg_p_2)) {
                        return 0.0;
                    }
                }
                return 1.0;
            } else return 0.0;
        } else return 0.0;
    }
}


//********************************************************************************
// Debug/tests

std::string BehaviorDescriptionMatcher::toStringTestCase(BehaviorCategory &category, CompositeBehaviorDescription &behaviorDescription)
{

    buildMapping(category, behaviorDescription);

    computeSetDistribution(category);
    float f1 = computeSetsDistributionFitness(behaviorDescription);
    float w1 = computeSetsDistributionRelevance(category);

    computeSucessorMatrix(category);
    float f2 = computeSucessorFitness(behaviorDescription);
    float w2 = computeSucessorRelevance(category);
    destroyMatrices(category);

    computeTimeLengthBoundaries(category);
    float f3 = computeIntervalFitness(behaviorDescription);
    float w3 = computeIntervalRelevance(category);

    float fitness = ((w1 * f1) + (w2 * f2) + (w3 * f3)) / (w1 + w2 + w3);

    std::string answer;
    char tmp[128];

    sprintf(tmp, "w1 = %f\n", w1);
    answer.append(tmp);
    sprintf(tmp, "w2 = %f\n", w2);
    answer.append(tmp);
    sprintf(tmp, "w3 = %f\n", w3);
    answer.append(tmp);
    sprintf(tmp, "f1 = %f\n", f1);
    answer.append(tmp);
    sprintf(tmp, "f2 = %f\n", f2);
    answer.append(tmp);
    sprintf(tmp, "f3 = %f\n", f3);
    answer.append(tmp);
    sprintf(tmp, "fitness = %f\n", fitness);
    answer.append(tmp);

    return answer;
}

std::string BehaviorDescriptionMatcher::toStringAllSets(BehaviorCategory &category)
{

    buildUnionOfPredicateSets(category);

    std::string answer = "{";
    std::set<PredicateHandleSet>::iterator it = allSets.begin();
    while (it != allSets.end()) {
        answer.append((*it).toString(*atomSpace));
        ++it;
        if (it != allSets.end()) {
            answer.append(",");
        }
    }
    answer.append("}");

    return answer;
}

std::string BehaviorDescriptionMatcher::toStringMapping(BehaviorCategory &category, CompositeBehaviorDescription &behaviorDescription)
{

    buildMapping(category, behaviorDescription);

    std::string answer = "{";
    std::map<PredicateHandleSet, PredicateHandleSet>::iterator it = mapping.begin();
    while (true) {
        answer.append("(");
        answer.append((*it).first.toString(*atomSpace));
        answer.append("->");
        answer.append((*it).second.toString(*atomSpace));
        answer.append(")");
        ++it;
        if (it != mapping.end()) {
            answer.append(",");
        } else {
            break;
        }
    }
    answer.append("}");

    return answer;
}

std::string BehaviorDescriptionMatcher::toStringSetDistribution(BehaviorCategory &category)
{

    buildUnionOfPredicateSets(category);
    computeSetDistribution(category);
    std::string answer;

    std::map<PredicateHandleSet, std::vector<int> >::iterator it = setDistribution.begin();
    while (true) {
        answer.append((*it).first.toString(*atomSpace));
        answer.append(": ");
        answer.append("{");
        for (unsigned int i = 0; i < (*it).second.size(); i++) {
            char tmp[128];
            sprintf(tmp, "%d", ((*it).second)[i]);
            answer.append(tmp);
            if (i != ((*it).second.size() - 1)) {
                answer.append(",");
            }
        }
        answer.append("}");
        ++it;
        answer.append("\n");
        if (it == setDistribution.end()) {
            break;
        }
    }

    return answer;
}

std::string BehaviorDescriptionMatcher::toStringSetDistributionFitness(BehaviorCategory &category, std::vector<CompositeBehaviorDescription> &v)
{

    buildUnionOfPredicateSets(category);
    computeSetDistribution(category);

    //printf("allSets = %s\n", toStringAllSets(category).c_str());
    //printf("setDistribution = %s\n", toStringSetDistribution(category).c_str());

    std::string answer;
    answer.append("Relevance = ");
    float w1 = computeSetsDistributionRelevance(category);
    char tmp[128];
    sprintf(tmp, "%f\n", w1);
    answer.append(tmp);
    for (unsigned int i = 0; i < v.size(); i++) {
        //printf("mapping = %s\n", toStringMapping(category, v[i]).c_str());
        answer.append(v[i].toStringTimeline());
        answer.append(": ");
        buildMapping(category, v[i]);
        float f1 = computeSetsDistributionFitness(v[i]);
        sprintf(tmp, "%f", f1);
        answer.append(tmp);
        answer.append("\n");
    }

    return answer;
}

std::string BehaviorDescriptionMatcher::toStringSucessorFitness(BehaviorCategory &category, std::vector<CompositeBehaviorDescription> &v)
{

    buildUnionOfPredicateSets(category);
    computeSucessorMatrix(category);

    //printf("allSets = %s\n", toStringAllSets(category).c_str());
    //printf("matrices = %s\n", toStringMatrices(category).c_str());

    std::string answer;
    answer.append("Relevance = ");
    //printf("Computing relevance\n");
    float w1 = computeSucessorRelevance(category);
    //printf("relevance done\n");
    char tmp[128];
    sprintf(tmp, "%f\n", w1);
    answer.append(tmp);
    for (unsigned int i = 0; i < v.size(); i++) {
        //printf("mapping = %s\n", toStringMapping(category, v[i]).c_str());
        answer.append(v[i].toStringTimeline());
        answer.append(": ");
        buildMapping(category, v[i]);
        float f = computeSucessorFitness(v[i]);
        sprintf(tmp, "%f", f);
        answer.append(tmp);
        answer.append("\n");
    }

    return answer;
}

std::string BehaviorDescriptionMatcher::toStringMatrices(BehaviorCategory &category)
{

    buildUnionOfPredicateSets(category);
    computeSucessorMatrix(category);

    std::string answer;
    std::vector<CompositeBehaviorDescription> bds = category.getEntries();
    for (unsigned int i = 0; i < bds.size(); i++) {
        answer.append(bds[i].toStringTimeline());
        answer.append("\n");
        for (unsigned int j = 0; j < allSets.size(); j++) {
            for (unsigned int k = 0; k < allSets.size(); k++) {
                char tmp[128];
                sprintf(tmp, "%d", categoryMatrices[i][j][k]);
                answer.append(tmp);
                if (k != (allSets.size() - 1)) {
                    answer.append(" ");
                } else {
                    answer.append("\n");
                }
            }
        }
    }

    return answer;
}

std::string BehaviorDescriptionMatcher::toStringBounds(BehaviorCategory &category)
{

    buildUnionOfPredicateSets(category);
    computeTimeLengthBoundaries(category);

    std::string answer;
    std::set<PredicateHandleSet>::iterator it = allSets.begin();
    while (it != allSets.end()) {
        answer.append((*it).toString(*atomSpace));
        char tmp[128];
        sprintf(tmp, ": (%ld, ", lowerBound[*it]);
        answer.append(tmp);
        sprintf(tmp, "%ld)\n", upperBound[*it]);
        answer.append(tmp);
        ++it;
    }

    return answer;
}

//********************************************************************************
// Private API

float BehaviorDescriptionMatcher::computeIntervalFitness(CompositeBehaviorDescription &bd)
{

    std::vector<PredicateHandleSet> sets = bd.getTimelineSets();
    std::vector<long> intervals = bd.getTimelineIntervals();
    OC_ASSERT(sets.size() == intervals.size(),
                     "TimelineSet and TimelineIntervals should have equal sizes (computeIntervalFitness).");

    float sum = 0;
    for (unsigned int i = 0; i < sets.size(); i++) {
        long t = intervals[i];
        long ub = upperBound[mapping[sets[i]]];
        long lb = lowerBound[mapping[sets[i]]];
        long limit = ub - lb;
        float v;
        if ((lb <= t) && (t <= ub)) {
            v = 1;
        } else {
            long d;
            if (t > ub) {
                d = t - ub;
            } else {
                d = lb - t;
            }
            if (d > limit) {
                v = 0;
            } else {
                v = 1 - ((float) d / limit);
            }
        }
        sum += (v * mappingSimilarity[sets[i]]);
    }

    return sum / sets.size();
}

float BehaviorDescriptionMatcher::computeIntervalRelevance(BehaviorCategory &category)
{

    float sum = 0;
    int count = 0;

    std::vector<CompositeBehaviorDescription> bds = category.getEntries();
    for (unsigned int i = 0; i < bds.size(); i++) {
        std::vector<PredicateHandleSet> sets = bds[i].getTimelineSets();
        std::vector<long> intervals = bds[i].getTimelineIntervals();
        OC_ASSERT(sets.size() == intervals.size(),
                         "TimelineSet and TimelineIntervals should have equal sizes (computeIntervalRelevance).");
        for (unsigned int j = 0; j < sets.size(); j++) {
            long d1 = intervals[j] - lowerBound[sets[j]];
            long d2 = upperBound[sets[j]] - intervals[j];
            long baseLength = upperBound[sets[j]] - lowerBound[sets[j]];
            long d = (d1 > d2 ? d1 : d2);
            if (baseLength == 0) {
                sum += 1;
            } else {
                sum += 1 - (d / baseLength);
            }
            count++;
        }
    }

    return sum / count;
}

void BehaviorDescriptionMatcher::computeTimeLengthBoundaries(BehaviorCategory &category)
{

    upperBound.clear();
    lowerBound.clear();

    std::vector<CompositeBehaviorDescription> bds = category.getEntries();
    for (unsigned int i = 0; i < bds.size(); i++) {
        std::vector<PredicateHandleSet> sets = bds[i].getTimelineSets();
        std::vector<long> intervals = bds[i].getTimelineIntervals();
        OC_ASSERT(sets.size() == intervals.size(),
                         "TimelineSet and TimelineIntervals should have equal sizes (computeTimeLengthBoundaries).");
        for (unsigned int j = 0; j < sets.size(); j++) {
            if (upperBound.find(sets[j]) == upperBound.end()) {
                upperBound[sets[j]] = intervals[j];
                lowerBound[sets[j]] = intervals[j];
            } else {
                if (intervals[j] > upperBound[sets[j]]) {
                    upperBound[sets[j]] = intervals[j];
                }
                if (intervals[j] < lowerBound[sets[j]]) {
                    lowerBound[sets[j]] = intervals[j];
                }
            }
        }
    }
}

float BehaviorDescriptionMatcher::computeSucessorFitness(CompositeBehaviorDescription &bd, int index)
{

    //printf("Computing fitness of %s\n", bd.toStringTimeline().c_str());

    int bdSize = bd.getTimelineSets().size();
    if (bdSize < 2) {
        return 1;
    }
    int DMAX = (bdSize * (bdSize - 1));

    //printf("DMAX = %d\n", DMAX);

    //printf("allSets.size() = %u\n", allSets.size());
    int **bdMatrix = NULL;
    //printf("index = %d\n", index);
    if (index == -1) {
        bdMatrix = new int *[allSets.size()];
        for (unsigned int j = 0; j < allSets.size(); j++) {
            bdMatrix[j] = new int[allSets.size()];
        }
        for (unsigned int j = 0; j < allSets.size(); j++) {
            for (unsigned int k = 0; k < allSets.size(); k++) {
                bdMatrix[j][k] = 0;
            }
        }
        buildSucessorMatrix(bd, bdMatrix, false);
    }

    /*
    if (index == -1) {
        std::string answer;
        for (unsigned int j = 0; j < allSets.size(); j++) {
            for (unsigned int k = 0; k < allSets.size(); k++) {
                char tmp[128];
                sprintf(tmp, "%d", bdMatrix[j][k]);
                answer.append(tmp);
                if (k != (allSets.size() - 1)) {
                    answer.append(" ");
                } else {
                    answer.append("\n");
                }
            }
        }
        printf("bdMatrix =\n%s\n\n", answer.c_str());
    }
    */

    float sumDifferences = 0;
    for (unsigned int j = 0; j < allSets.size(); j++) {
        for (unsigned int k = 0; k < allSets.size(); k++) {
            if (index == -1) {
                sumDifferences +=  fabs(bdMatrix[j][k] - averageMatrix[j][k]);
            } else {
                sumDifferences +=  fabs(categoryMatrices[index][j][k] - averageMatrix[j][k]);
            }
        }
    }

    //printf("sumDifferences = %f\n", sumDifferences);

    if (index == -1) {
        for (unsigned int j = 0; j < allSets.size(); j++) {
            delete [] bdMatrix[j];
        }
        delete [] bdMatrix;
    }

    float f;
    if (sumDifferences >= DMAX) {
        f = 0;
    } else {
        f = (1 - (sumDifferences / DMAX));
    }

    return f;
}

float BehaviorDescriptionMatcher::computeSucessorRelevance(BehaviorCategory &category)
{

    std::vector<CompositeBehaviorDescription> bds = category.getEntries();

    float sum = 0;
    for (unsigned int i = 0; i < bds.size(); i++) {
        sum += computeSucessorFitness(bds[i], i);
    }

    float answer = sum / bds.size();
    return answer;
}

void BehaviorDescriptionMatcher::buildSucessorMatrix(CompositeBehaviorDescription &bd, int **matrix, bool categoryFlag)
{

    //printf("Buinding sucessor matrix for %s\n", bd.toStringTimeline().c_str());
    std::vector<PredicateHandleSet> sets = bd.getTimelineSets();

    for (unsigned int i = 1; i < sets.size(); i++) {
        for (unsigned int j =  0; j < i; j++) {
            if (categoryFlag) {
                //printf("matrixIndex[sets[i]] = %u\n", matrixIndex[sets[i]]);
                //printf("matrixIndex[sets[j]] = %u\n", matrixIndex[sets[j]]);
                matrix[matrixIndex[sets[i]]][matrixIndex[sets[j]]]++;
            } else {
                //printf("sets[i] = %s\n", sets[i].toString().c_str());
                //printf("mapping[sets[i]] = %s\n", mapping[sets[i]].toString().c_str());
                //printf("matrixIndex[mapping[sets[i]]] = %u\n", matrixIndex[mapping[sets[i]]]);
                //printf("sets[j] = %s\n", sets[j].toString().c_str());
                //printf("mapping[sets[j]] = %s\n", mapping[sets[j]].toString().c_str());
                //printf("matrixIndex[mapping[sets[j]]] = %u\n", matrixIndex[mapping[sets[j]]]);
                matrix[matrixIndex[mapping[sets[i]]]][matrixIndex[mapping[sets[j]]]]++;
            }
        }
    }
    //printf("Done\n");
}

void BehaviorDescriptionMatcher::computeSucessorMatrix(BehaviorCategory &category)
{

    std::vector<CompositeBehaviorDescription> bds = category.getEntries();

    if (! matricesBuilt) {
        //printf("Allocating\n");
        categoryMatrices = new int **[bds.size()];
        for (unsigned int i = 0; i < bds.size(); i++) {
            categoryMatrices[i] = new int *[allSets.size()];
            for (unsigned int j = 0; j < allSets.size(); j++) {
                categoryMatrices[i][j] = new int[allSets.size()];
            }
        }

        averageMatrix = new float *[allSets.size()];
        for (unsigned int j = 0; j < allSets.size(); j++) {
            averageMatrix[j] = new float[allSets.size()];
        }

        {
            std::set<PredicateHandleSet>::iterator it = allSets.begin();
            unsigned int j = 0;
            while (it != allSets.end()) {
                matrixIndex[*it] = j;
                ++j;
                ++it;
            }
        }

        matricesBuilt = true;
    }

    //printf("Zeroing\n");
    for (unsigned int i = 0; i < bds.size(); i++) {
        for (unsigned int j = 0; j < allSets.size(); j++) {
            for (unsigned int k = 0; k < allSets.size(); k++) {
                categoryMatrices[i][j][k] = 0;
            }
        }
    }

    //printf("Building matrices\n");
    for (unsigned int i = 0; i < bds.size(); i++) {
        buildSucessorMatrix(bds[i], categoryMatrices[i], true);
    }

    //printf("Averaging\n");
    for (unsigned int j = 0; j < allSets.size(); j++) {
        for (unsigned int k = 0; k < allSets.size(); k++) {
            averageMatrix[j][k] = 0;
            for (unsigned int i = 0; i < bds.size(); i++) {
                averageMatrix[j][k] += categoryMatrices[i][j][k];
            }
        }
    }

    for (unsigned int j = 0; j < allSets.size(); j++) {
        for (unsigned int k = 0; k < allSets.size(); k++) {
            averageMatrix[j][k] /= bds.size();
        }
    }
    //printf("Done\n");
}

float BehaviorDescriptionMatcher::computeSetsDistributionRelevance(BehaviorCategory &category)
{

    float sum = 0;

    std::vector<CompositeBehaviorDescription> bds = category.getEntries();
    for (unsigned int i = 0; i < bds.size(); i++) {
        sum += computeSetsDistributionFitness(bds[i], true);
    }

    return sum / bds.size();
}

float BehaviorDescriptionMatcher::computeSetsDistributionFitness(CompositeBehaviorDescription &behaviorDescription, bool relevanceFlag)
{

    //printf("Computing setDistributionFitness of %s\n", behaviorDescription.toStringTimeline().c_str());
    std::vector<PredicateHandleSet> bdSets = behaviorDescription.getTimelineSets();
    std::map<PredicateHandleSet, int> bdCount;
    std::set<PredicateHandleSet> bdSetsNoRepetition;
    for (unsigned int i = 0; i < bdSets.size(); i++) {
        bdSetsNoRepetition.insert(bdSets[i]);
        if (bdCount.find(bdSets[i]) == bdCount.end()) {
            bdCount[bdSets[i]] = 1;
        } else {
            bdCount[bdSets[i]] = bdCount[bdSets[i]] + 1;
        }
    }

    if (bdSetsNoRepetition.empty()) {
        return 0;
    }

    float sum = 0;
    for(std::set<PredicateHandleSet>::iterator it = bdSetsNoRepetition.begin(); it != bdSetsNoRepetition.end(); ++it) {
        //printf("count[%s] = %d\n", (*it).toString().c_str(), bdCount[*it]);
        if (! relevanceFlag) {
            //printf("mapping[%s] = %s\n", (*it).toString().c_str(), mapping[*it].toString().c_str());
            //printf("mappingSimilarity[%s] = %f\n", (*it).toString().c_str(), mappingSimilarity[*it]);
        }
        int count = bdCount[*it];
        std::vector<int> v;
        if (relevanceFlag) {
            v = setDistribution[*it];
        } else {
            v = setDistribution[mapping[*it]];
        }
        if (v.empty()) {
            return 0;
        }
        float s = 0;
        for (unsigned int i = 0; i < v.size(); i++) {
            if (v[i] == count) {
                if (relevanceFlag) {
                    s += (((float) 1) / v.size()) * allSetsCount[*it];
                } else {
                    s += (((float) 1) / v.size()) * allSetsCount[mapping[*it]];
                }
            }
        }
        //printf("s = %f\n", s);
        float w = 1;
        if ( ! relevanceFlag) {
            w = mappingSimilarity[*it];
        }
        sum += s * w;
    }

    //printf("sumAllSetsCount = %d\n", sumAllSetsCount);
    float f = (sum / sumAllSetsCount);
    if (f > 1) {
        f = 1;
    }
    //printf("f = %f\n", f);
    return f;
}


void BehaviorDescriptionMatcher::buildUnionOfPredicateSets(BehaviorCategory &category)
{

    allSets.clear();
    allSetsCount.clear();
    sumAllSetsCount = 0;

    std::vector<CompositeBehaviorDescription> bds = category.getEntries();
    for (unsigned int i = 0; i < bds.size(); i++) {
        std::vector<PredicateHandleSet> sets = bds[i].getTimelineSets();
        for (unsigned int j = 0; j < sets.size(); j++) {
            sumAllSetsCount++;
            if (allSets.find(sets[j]) == allSets.end()) {
                allSets.insert(sets[j]);
                allSetsCount[sets[j]] = 1;
            } else {
                allSetsCount[sets[j]] = allSetsCount[sets[j]] + 1;
            }
        }
    }
}


float BehaviorDescriptionMatcher::computeSetSimilarity(const PredicateHandleSet &set1, const PredicateHandleSet &set2) const
{

    if (set1.getSize() == 0) {
        if (set2.getSize() == 0) {
            printf("both sets are 0 size\n");
            return 1.0;
        } else {
            printf("only set1 is 0 size\n");
            return 0.0;
        }
    } else {
        if (set2.getSize() == 0) {
            printf("only set2 is 0 size\n");
            return 0.0;
        }
    }

    /* can no longer use the presence of atomspace pointer to determine
     * similarity measure to use..
     if (atomSpace == NULL) {
        printf("atomspace null\n");
        return intersectionOverUnion(set1, set2);
    } else {
        printf("not null\n");
        return weightedIntersectionOverUnion(set1, set2);
    }*/
    float returnValue = weightedIntersectionOverUnion(set1,set2);
    if (returnValue == 0) returnValue = intersectionOverUnion(set1,set2);
    return returnValue;

}

float BehaviorDescriptionMatcher::intersectionOverUnion(const PredicateHandleSet &set1, const PredicateHandleSet &set2) const
{

    int intersectionSize = 0;
    for (std::set<Handle>::iterator it = set1.getSet().begin(); it != set1.getSet().end(); ++it) {
        if (set2.getSet().find(*it) != set2.getSet().end()) {
            intersectionSize++;
        }
    }

    return ((float) intersectionSize / (float) (set1.getSize() + set2.getSize() - intersectionSize));
}

float BehaviorDescriptionMatcher::weightedIntersectionOverUnion(const PredicateHandleSet &set1, const PredicateHandleSet &set2) const
{

    float intersectionSize = 0;
    for (std::set<Handle>::iterator it1 = set1.getSet().begin(); it1 != set1.getSet().end(); ++it1) {
        float bestSimilarity = 0;
        for (std::set<Handle>::iterator it2 = set2.getSet().begin(); it2 != set2.getSet().end(); ++it2) {
            float similarity = EvaluationLinkSimilarityEvaluator::similarity(*atomSpace, *it1, *it2);
            if (similarity > bestSimilarity) {
                bestSimilarity = similarity;
            }
        }
        intersectionSize += bestSimilarity;
    }

    return (intersectionSize / (float) (set1.getSize() + set2.getSize() - intersectionSize));
}

void BehaviorDescriptionMatcher::buildMapping(BehaviorCategory &category, CompositeBehaviorDescription &behaviorDescription)
{

    buildUnionOfPredicateSets(category);

    std::vector<PredicateHandleSet> bdSets = behaviorDescription.getTimelineSets();

    if (allSets.empty() || bdSets.empty()) {
        return;
    }

    mapping.clear();
    mappingSimilarity.clear();

    for (unsigned int i = 0; i < bdSets.size(); i++) {
        //printf("mapping %s\n", bdSets[i].toString(*atomSpace).c_str());
        float betterSimilarity = -1;
        for (std::set<PredicateHandleSet>::iterator it = allSets.begin(); it != allSets.end(); ++it) {
            //printf("comparing against %s\n", (*it).toString(*atomSpace).c_str());
            float similarity = computeSetSimilarity(bdSets[i], *it);
            //printf("similarity = %f\n", similarity);
            if (similarity > betterSimilarity) {
                betterSimilarity = similarity;
                //printf("updating\n");
                mapping[bdSets[i]] = *it;
                mappingSimilarity[bdSets[i]] = similarity;
                //printf("done\n");
                if (similarity == 1.0) {
                    //printf("breaking\n");
                    break;
                }
            }
        }
    }
}

void BehaviorDescriptionMatcher::computeSetDistribution(BehaviorCategory &category)
{

    setDistribution.clear();

    for (std::set<PredicateHandleSet>::iterator it = allSets.begin(); it != allSets.end(); ++it) {
        //printf("computing %s\n", (*it).toString().c_str());
        std::vector<CompositeBehaviorDescription> entries = category.getEntries();
        for (unsigned int i = 0; i < entries.size(); i++) {
            //printf("counting in %s\n", entries[i].toString().c_str());
            int count = 0;
            std::vector<PredicateHandleSet> sets = entries[i].getTimelineSets();
            for (unsigned int j = 0; j < sets.size(); j++) {
                if (sets[j] == (*it)) {
                    count++;
                }
            }
            //printf("result = %d\n", count);
            setDistribution[*it].push_back(count);
        }
    }
}

void BehaviorDescriptionMatcher::setAtomSpace(AtomSpace *atomSpace)
{
    this->atomSpace = atomSpace;
}

