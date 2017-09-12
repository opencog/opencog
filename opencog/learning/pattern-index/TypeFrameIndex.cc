#include <limits>
#include <unistd.h>
#include <chrono>
#include <time.h>

#include <opencog/util/Logger.h>

#include "TypeFrameIndex.h"
#include "TypeFrame.h"
#include "CombinationGenerator.h"
#include "PartitionGenerator.h"
#include "CartesianProductGenerator.h"

using namespace opencog;
using namespace std;

const int TypeFrameIndex::OPERATOR_NOP = 0;
const int TypeFrameIndex::OPERATOR_AND = 1;
const int TypeFrameIndex::OPERATOR_OR = 2;
const int TypeFrameIndex::OPERATOR_NOT = 3;

TypeFrameIndex::TypeFrameIndex() 
{
    // Parameters
    //
    // These raw defaults are overwritten when TypeFrameIndex is used thru
    // PatternIndexAPI.
    PATTERN_COUNT_CACHE_ENABLED = false;
    LIMIT_FOR_UNORDERED_LINKS_PERMUTATION = 5;
    NUMBER_OF_EVALUATION_THREADS = 4;
    MINIMAL_FREQUENCY_TO_COMPUTE_QUALITY_METRIC = 12;
    MAX_SIZE_OF_COMPOUND_FRAMES_QUEUE = 5000000;
    COHERENCE_FUNCTION = CONST_1;
    COHERENCE_MODULATOR_G = ONE_OVER_COHERENCE;
    COHERENCE_MODULATOR_H = COHERENCE;
    DIFFERENT_ASSIGNMENT_FOR_DIFFERENT_VARS = true;
    PERMUTATIONS_OF_VARS_CONSIDERED_EQUIVALENT = true;
    PATTERNS_GRAM = 3;
    MAXIMUM_NUMBER_OF_MINING_RESULTS = 10;
    PATTERN_RANKING_METRIC = N_I_SURPRISINGNESS;

    ALLOWED_TOP_LEVEL_TYPES.insert(INHERITANCE_LINK);
    ALLOWED_TOP_LEVEL_TYPES.insert(EVALUATION_LINK);

    ALLOWED_VAR_SUBSTITUTION.insert(CONCEPT_NODE);
    ALLOWED_VAR_SUBSTITUTION.insert(PREDICATE_NODE);

    auxVar1.push_back(TypePair(classserver().getType("VariableNode"), 0));
    auxVar1.setNodeNameAt(0, "V1");
    time1 = time2 = 0;
}

TypeFrameIndex::~TypeFrameIndex() 
{
}

float TypeFrameIndex::computeCoherence(const TypeFrame &frame) const
{
    switch (COHERENCE_FUNCTION) {
        case CONST_1: return 1;
        default: throw runtime_error("Unknown coherence function\n");
    }
}

float TypeFrameIndex::one_over_x(float x) const
{
    if (x < numeric_limits<float>::epsilon()) {
        return numeric_limits<float>::max();
    } else {
        return 1 / x;
    }
}

float TypeFrameIndex::gFunction(float x) const
{
    switch (COHERENCE_MODULATOR_G) {
        case ONE_OVER_COHERENCE: return one_over_x(x);
        default: throw runtime_error("Unknown coherence modulator G function\n");
    }
}

float TypeFrameIndex::hFunction(float x) const
{
    switch (COHERENCE_MODULATOR_H) {
        case COHERENCE: return x;
        default: throw runtime_error("Unknown coherence modulator H function\n");
    }
}

TypeFrame TypeFrameIndex::getFrameAt(int index)
{
    return frames.at(index);
}

void TypeFrameIndex::addPatterns(vector<TypeFrame> &answer,
                                 const TypeFrame &base) const
{
    TypeFrameSet nodes;
    base.buildNodesSet(nodes, ALLOWED_VAR_SUBSTITUTION, true);
    CombinationGenerator selected(nodes.size(), true, false);

    if (DEBUG) {
        base.printForDebug("Compound frame: ", "\n", true);
        printf("Nodes: ");
        for (TypeFrameSet::iterator it = nodes.begin(); it != nodes.end(); ++it) {
            (*it).printForDebug("", " ", true);
        }
        printf("\n");
    }

    TypeFrame pattern;
    while (! selected.depleted()) {
        if (DEBUG) selected.printForDebug("Selection: ", "\n");
        pattern.clear();
        pattern.append(base);
        int count = 0;
        for (TypeFrameSet::iterator it = nodes.begin(); it != nodes.end(); ++it) {
            if (selected.at(count)) {
                string varName = "V" + to_string(count);
                for (unsigned int k = 0; k < pattern.size(); k++) {
                    if (pattern.subFrameAt(k).equals((*it))) {
                        pattern[k].first = classserver().getType("VariableNode");
                        pattern.setNodeNameAt(k, varName);
                    }
                }
            }
            count++;
        }
        answer.push_back(pattern);
        if (DEBUG) pattern.printForDebug("Pattern: ", "\n", true);
        selected.generateNext();
    }
}

float TypeFrameIndex::computeQuality(const TypeFrame &pattern)
{
    float answer = 0;

    switch (PATTERN_RANKING_METRIC) {
        case I_SURPRISINGNESS: answer = computeISurprinsingness(pattern, false); break;
        case N_I_SURPRISINGNESS: answer = computeISurprinsingness(pattern, true); break;
        case II_SURPRISINGNESS: answer = computeIISurprinsingness(pattern, false); break;
        case N_II_SURPRISINGNESS: answer = computeIISurprinsingness(pattern, true); break;
        default: throw runtime_error("Unknown pattern ranking metric\n");
    }

    return answer;
}

unsigned int TypeFrameIndex::countPattern(const TypeFrame &pattern)
{
    if (PATTERN_COUNT_CACHE_ENABLED) {
        PatternCountMap::const_iterator it = patternCountCache.find(pattern);
        if (it != patternCountCache.end()) {
            return (*it).second;
        }
    }

    vector<TypeFrameIndex::ResultPair> queryAnswer;
    query(queryAnswer, pattern, true, false);
    unsigned int answer = queryAnswer.size();

    if (PATTERN_COUNT_CACHE_ENABLED) {
        patternCountCache.insert(PatternCountMap::value_type(pattern, answer));
    }

    return answer;
}

void TypeFrameIndex::addSupersetFrames(vector<TypeFrame> &answer,
                                       const TypeFrame &frame) const
{
    vector<TypeFrame> notUsed;
    addInferredSetFrames(answer, notUsed, frame, false);
}

void TypeFrameIndex::addSubsetFrames(vector<TypeFrame> &subset,
                                     vector<TypeFrame> &star,
                                     const TypeFrame &frame) const
{
    addInferredSetFrames(subset, star, frame, true);
}

void TypeFrameIndex::addInferredSetFrames(vector<TypeFrame> &subset,
                                          vector<TypeFrame> &star,
                                          const TypeFrame &frame,
                                          bool subSetFlag) const
{
    subset.clear();
    TypeFrameSet nodes;
    frame.buildNodesSet(nodes, ALLOWED_VAR_SUBSTITUTION, false);
    if (DEBUG) printTypeFrameSet(nodes);
    CombinationGenerator selected(nodes.size(), true, false);

    TypeFrame starFrame, compoundFrame;
    TypeFrame aux1;
    TypeFrame auxInternalVar1;
    auxInternalVar1.push_back(TypePair(classserver().getType("VariableNode"), 0));
    auxInternalVar1.setNodeNameAt(0, "IV1");

    while (! selected.depleted()) {
        if (DEBUG) selected.printForDebug("selected: ", "\n");
        int count = 0;
        for (TypeFrameSet::iterator it = nodes.begin(); it != nodes.end(); ++it) {
            if (selected.at(count++)) {
                aux1.clear();
                if (DEBUG) frame.printForDebug("base: ", "\n");
                if (DEBUG) (*it).printForDebug("key: ", "\n");
                if (DEBUG) auxInternalVar1.printForDebug("subst for: ", "\n");
                aux1 = frame.copyReplacingFrame((*it), auxInternalVar1);
                if (DEBUG) aux1.printForDebug("result: ", "\n");
                compoundFrame.clear();
                compoundFrame.push_back(TypePair(classserver().getType("AndLink"), 2));
                compoundFrame.append(aux1);
                if (subSetFlag) {
                    compoundFrame.push_back(TypePair(classserver().getType("InheritanceLink"), 2));
                    compoundFrame.append(auxInternalVar1);
                    compoundFrame.append(*it);
                    starFrame.clear();
                    starFrame.push_back(TypePair(classserver().getType("AndLink"), 2));
                    starFrame.append(frame);
                    starFrame.push_back(TypePair(classserver().getType("NotLink"), 1));
                    starFrame.append(compoundFrame);
                    star.push_back(starFrame);
                } else {
                    compoundFrame.push_back(TypePair(classserver().getType("InheritanceLink"), 2));
                    compoundFrame.append(*it);
                    compoundFrame.append(auxInternalVar1);
                }
                subset.push_back(compoundFrame);
            }
        }
        selected.generateNext();
    }
}

pair<float, float> TypeFrameIndex::minMaxIndependentProb(const TypeFrame &pattern)
{
    float minP = 0;
    float maxP = 0;

    if (pattern.typeAtEqualsTo(0, "AndLink")) {
        if (DEBUG) printf("pattern is AndLink\n");
        minP = numeric_limits<float>::max();
        unsigned int n = pattern.at(0).second;
        vector<int> argPos = pattern.getArgumentsPosition(0);
        PartitionGenerator partitionGenerator(n);
        TypeFrame subPattern;
        TypeFrame aux;
        while (! partitionGenerator.depleted()) {
            if (DEBUG) partitionGenerator.printForDebug("Partition: ", "\n");
            PartitionGenerator::IntegerSetSet partition = partitionGenerator.getPartition();
            float prod = 1;
            for (PartitionGenerator::IntegerSetSet::const_iterator it1 = partition.begin(); it1 != partition.end(); ++it1) {
                subPattern.clear();
                if ((*it1).size() > 1) {
                    subPattern.push_back(TypePair(classserver().getType("AndLink"), (*it1).size()));
                }
                for (PartitionGenerator::IntegerSet::const_iterator it2 = (*it1).begin(); it2 != (*it1).end(); ++it2) {
                    aux = pattern.subFrameAt(argPos.at(*it2));
                    subPattern.append(aux);
                }
                if (DEBUG) subPattern.printForDebug("subPattern: ", "\n", true);
                int count = countPattern(subPattern);
                if (DEBUG) printf("count = %u\n", count);
                if (count == 0) {
                    prod = 0;
                    break;
                } else {
                    float p = ((float) count) / floatUniverseCount;
                    prod *= p;
                }
            }
            if (prod > maxP) maxP = prod;
            if (prod < minP) minP = prod;
            partitionGenerator.generateNext();
        }
    }

    pair<float, float> answer = make_pair(minP, maxP);
    
    return answer;
}

pair<float, float> TypeFrameIndex::minMaxSupersetProb(const TypeFrame &pattern)
{
    float minP = 0;
    float maxP = 0;

    if (pattern.typeAtEqualsTo(0, "AndLink")) {
        if (DEBUG) printf("pattern is AndLink\n");
        unsigned int n = pattern.at(0).second;
        vector<int> argPos = pattern.getArgumentsPosition(0);
        TypeFrame aux;
        TypeFrame spotFrame;
        TypeFrame compoundFrame;
        for (unsigned int i = 0; i < n; i++) {
            spotFrame.clear();
            spotFrame = pattern.subFrameAt(argPos.at(i));
            vector<TypeFrame> supersetFrames;
            if (DEBUG) spotFrame.printForDebug("spotFrame: ", "\n");
            addSupersetFrames(supersetFrames, spotFrame);
            if (DEBUG) printFrameVector(supersetFrames);
            if (supersetFrames.size() > 0) {
                minP = numeric_limits<float>::max();
            }
            for (unsigned int k = 0; k < supersetFrames.size(); k++) {
                unsigned int countSpot = countPattern(spotFrame);
                if (DEBUG) printf("spot (count = %u): ", countSpot);
                if (DEBUG) spotFrame.printForDebug("", "\n");
                unsigned int countSuperset = countPattern(supersetFrames.at(k));
                if (DEBUG) printf("supersetFrames.at(%u) (count = %u): ", k, countSuperset);
                if (DEBUG) supersetFrames.at(k).printForDebug("", "\n");
                float pSpot = 0;
                float pSuperset = 0;
                float pCompound = 0;
                if ((countSpot > 0) && (countSuperset > 0)) {
                    pSpot = ((float) countSpot) / floatUniverseCount;
                    pSuperset = ((float) countSuperset) / floatUniverseCount;
                    compoundFrame.clear();
                    compoundFrame.push_back(TypePair(classserver().getType("AndLink"), n));
                    for (unsigned int j = 0; j < n; j++) {
                        if (j == i) {
                            compoundFrame.append(supersetFrames.at(k));
                        } else {
                            aux.clear();
                            aux = pattern.subFrameAt(argPos.at(j));
                            compoundFrame.append(aux);
                        }
                    }
                    unsigned int countCompound = countPattern(compoundFrame);
                    if (DEBUG) printf("compound (count = %u): ", countCompound);
                    if (DEBUG) compoundFrame.printForDebug("", "\n");
                    pCompound = ((float) countCompound) / floatUniverseCount;
                }
                float coherence = computeCoherence(supersetFrames.at(k));
                float yMax = (countSuperset == 0 ? 0 : gFunction(coherence) * pCompound * (pSpot / pSuperset));
                float yMin = (countSuperset == 0 ? 0 : hFunction(coherence) * pCompound * (pSpot / pSuperset));
                if (DEBUG) printf("yMax = %f\n", yMax);
                if (DEBUG) printf("yMin = %f\n", yMin);
                if (yMax > maxP) maxP = yMax;
                if (yMin < minP) minP = yMin;
            }
        }
    }

    return make_pair(minP, maxP);
}

pair<float, float> TypeFrameIndex::minMaxSubsetProb(const TypeFrame &pattern)
{
    float minP = 0;
    float maxP = 0;
    if (DEBUG) printf("frames.size() = %lu\n", frames.size());
    if (DEBUG) printf("floatUniverseCount = %f\n", floatUniverseCount);

    if (DEBUG) printf("minMaxSubsetProb()\n");
    if (pattern.typeAtEqualsTo(0, "AndLink")) {
        if (DEBUG) printf("pattern is AndLink\n");
        unsigned int n = pattern.at(0).second;
        vector<int> argPos = pattern.getArgumentsPosition(0);
        vector<vector<TypeFrame>> subsetVector;
        vector<vector<TypeFrame>> starVector;
        vector<TypeFrame> subsetFrames;
        vector<TypeFrame> starFrames;
        vector<unsigned int> sizes;
        TypeFrame aux;
        TypeFrame compoundFrame;
        TypeFrame compoundStar;
        for (unsigned int i = 0; i < n; i++) {
            aux.clear();
            aux = pattern.subFrameAt(argPos.at(i));
            if (DEBUG) aux.printForDebug("subFrame: ", "\n");
            addSubsetFrames(subsetFrames, starFrames, aux);
            if (DEBUG) printFrameVector(subsetFrames);
            if (DEBUG) printFrameVector(starFrames);
            subsetVector.push_back(subsetFrames);
            starVector.push_back(starFrames);
            // subsetFrames and starFrames have the same size
            sizes.push_back(subsetFrames.size());
        }
        CartesianProductGenerator cartesianGenerator(sizes);
        if (! cartesianGenerator.depleted()) {
            minP = numeric_limits<float>::max();
        }
        while (! cartesianGenerator.depleted()) {
            if (DEBUG) cartesianGenerator.printForDebug("Cartesian selection: ", "\n");
            compoundFrame.clear();
            compoundStar.clear();
            compoundFrame.push_back(TypePair(classserver().getType("AndLink"), n));
            compoundStar.push_back(TypePair(classserver().getType("AndLink"), n));
            float productMin = 1;
            float productMax = 1;
            for (unsigned int i = 0; i < n; i++) {
                compoundFrame.append(subsetVector.at(i).at(cartesianGenerator.at(i)));
                compoundStar.append(starVector.at(i).at(cartesianGenerator.at(i)));
                float coherence = computeCoherence(subsetVector.at(i).at(cartesianGenerator.at(i)));
                productMin *= hFunction(coherence);
                productMax *= gFunction(coherence);
            }
            if (DEBUG) compoundStar.printForDebug("compoundStar: ", "\n");
            pair<float, float> ind = minMaxIndependentProb(compoundStar);
            if (DEBUG) printf("ind = <%f,%f>\n", ind.first, ind.second);
            unsigned int countCompound = countPattern(compoundFrame);
            if (DEBUG) printf("%u: ", countCompound);
            if (DEBUG) compoundFrame.printForDebug("", "\n");
            float pCompound = ((float) countCompound) / floatUniverseCount;
            float yMin = productMin * (pCompound + ind.first);
            float yMax = productMax * pCompound;
            if (yMax > maxP) maxP = yMax;
            if (yMin < minP) minP = yMin;
            cartesianGenerator.generateNext();
        }
    }

    return make_pair(minP, maxP);
}

float TypeFrameIndex::computeISurprinsingness(const TypeFrame &pattern,
                                              bool normalized)
{
    if (DEBUG) printf("computeISurprinsingness()\n");
    if (DEBUG) pattern.printForDebug("", "\n");

    float answer = 0;
    unsigned int count = countPattern(pattern);
    //float pFull = ((float) count) / frames.size();
    float pFull = ((float) count) / floatUniverseCount;

    if (DEBUG) printf("count (pFull) = %u\n", count);
    if (DEBUG) printf("pFull = %f\n", pFull);

    if (count < MINIMAL_FREQUENCY_TO_COMPUTE_QUALITY_METRIC) {
        return 0;
    }

    pair<float, float> pair = minMaxIndependentProb(pattern);

    float c1 = pFull - pair.second;
    float c2 = pair.first - pFull;
    answer = (c1 > c2 ? c1 : c2);

    if (normalized) {
        answer = answer / pFull;
    }

    return answer;
}

float TypeFrameIndex::computeIISurprinsingness(const TypeFrame &pattern,
                                               bool normalized)
{
    if (DEBUG) printf("computeIISurprinsingness()\n");
    if (DEBUG) pattern.printForDebug("", "\n");

    float answer = 0;
    unsigned int count = countPattern(pattern);
    float pFull = ((float) count) / floatUniverseCount;

    if (DEBUG) printf("count (pFull) = %u\n", count);
    if (DEBUG) printf("pFull = %f\n", pFull);

    if (count < MINIMAL_FREQUENCY_TO_COMPUTE_QUALITY_METRIC) {
        return 0;
    }

    pair<float, float> minMaxInd = minMaxIndependentProb(pattern);
    if (DEBUG) printf("minMaxInd = <%f,%f>\n", minMaxInd.first, minMaxInd.second);

    pair<float, float> minMaxSuper = minMaxSupersetProb(pattern);
    if (DEBUG) printf("minMaxSuper = <%f,%f>\n", minMaxSuper.first, minMaxSuper.second);

    pair<float, float> minMaxSub = minMaxSubsetProb(pattern);
    if (DEBUG) printf("minMaxSub = <%f,%f>\n", minMaxSub.first, minMaxSub.second);

    float imaxP = minMaxInd.second;
    if (minMaxSuper.second > imaxP) imaxP = minMaxSuper.second;
    if (minMaxSub.second > imaxP) imaxP = minMaxSub.second;

    float iminP = minMaxInd.first;
    if (minMaxSuper.first > iminP) iminP = minMaxSuper.first;
    if (minMaxSub.first > iminP) iminP = minMaxSub.first;

    if (DEBUG) printf("iminP, imaxP = <%f,%f>\n", iminP, imaxP);

    float c1 = pFull - imaxP;
    float c2 = iminP - pFull;
    answer = (c1 > c2 ? c1 : c2);

    if (DEBUG) printf("(%f, %f) %f\n", c1, c2, answer);

    if (normalized) {
        answer = answer / pFull;
    }

    if (DEBUG) printf("Quality: %f\n", answer);

    return answer;
}

bool TypeFrameIndex::enqueueCompoundFrame(const TypeFrame &compoundFrame)
{
    lock_guard<mutex> lock(compoundFrameMutex);
    static unsigned int queueDebugCount = 0;
    if (DEBUG) printf("enqueueCompoundFrame() BEGIN\n");
    compoundFrameQueue.push(compoundFrame);
    if (DEBUG && (!(queueDebugCount++ % 100000))) printf("Queue size: %ld\n",  compoundFrameQueue.size());
    if (DEBUG) printf("enqueueCompoundFrame() END\n");
    return (compoundFrameQueue.size() > MAX_SIZE_OF_COMPOUND_FRAMES_QUEUE);
}

bool TypeFrameIndex::dequeueCompoundFrame(TypeFrame &compoundFrame)
{
    lock_guard<mutex> lock(compoundFrameMutex);
    if (DEBUG) printf("dequeueCompoundFrame() BEGIN\n");
    bool answer = false;
    if (compoundFrameQueue.size() > 0) {
        compoundFrame.clear();
        compoundFrame.append(compoundFrameQueue.front());
        compoundFrameQueue.pop();
        answer = true;
    }
    if (DEBUG) printf("dequeueCompoundFrame() END\n");
    return answer;
}

void TypeFrameIndex::setCompoundFramesEnded()
{
    lock_guard<mutex> lock(compoundFrameMutex);
    if (DEBUG) printf("setCompoundFramesEnded() BEGIN\n");
    compoundFramesEnded = true;
    if (DEBUG) printf("setCompoundFramesEnded() END\n");
}

bool TypeFrameIndex::checkCompoundPatternsEnded()
{
    lock_guard<mutex> lock(compoundFrameMutex);
    if (DEBUG) printf("checkCompoundPatternsEnded() BEGIN\n");
    if (DEBUG) printf("checkCompoundPatternsEnded() END\n");
    return compoundFramesEnded;
}

void TypeFrameIndex::addMiningResult(float quality, const TypeFrame &frame)
{
    lock_guard<mutex> lock(miningResultsMutex);
    miningResultsHeap.push(quality, frame);
}

// Executed by mining's working threads.
void TypeFrameIndex::evaluatePatterns()
{
    if (DEBUG) printf("evaluatePatterns() BEGIN\n");
    bool finished = false;
    TypeFrame compoundFrame;
    vector<TypeFrame> patterns;

    while (! finished) {
        if (dequeueCompoundFrame(compoundFrame)) {
            patterns.clear();
            addPatterns(patterns, compoundFrame);
            for (unsigned int i = 0; i < patterns.size(); i++) {
                float quality = computeQuality(patterns.at(i));
                addMiningResult(quality, patterns.at(i));
            }
        } else {
            if (checkCompoundPatternsEnded()) {
                finished = true;
            } else {
                if (DEBUG) printf("Evaluation thread sleeping\n");
                usleep(100000); // 100 miliseconds
            }
        }
    }
    if (DEBUG) printf("evaluatePatterns() END\n");
}

void TypeFrameIndex::minePatterns(vector<pair<float,TypeFrame>> &answer)
{
    logger().info("[PatternIndex] Start mining");
    answer.clear();

    // Setup
    this->patternCountCache.clear();
    miningResultsHeap.clear();
    miningResultsHeap.maxSize = MAXIMUM_NUMBER_OF_MINING_RESULTS;
    compoundFramesEnded = false;
    evaluationThreads.clear();
    for (int i = 0; i < (((int) NUMBER_OF_EVALUATION_THREADS) - 1); i++) {
        // Working threads that will actually evaluate patterns.
        // Threads will wait for compoundframes are ready to be evaluated.
        //
        // The -1 is because the main thread will work building the
        // compoundFrames so it will take one of the processors.
        // When main thread finish its job, the last working thread is
        // created.
        evaluationThreads.push_back(new thread(&TypeFrameIndex::evaluatePatterns, this));
    }
    EquivalentTypeFrameSet baseSet;

    if (DEBUG) printf("frames.size(): %lu\n", frames.size());
    // Build a set of subcomponents that will be used to build candidate
    // compoundFrames. subcomponents are tipically links that will be glued
    // together with an AndLink to build a compoundFrame. compoundFrames have
    // their Nodes replaced by variables to build patterns.
    //
    // A set is used here because a subcomponent could be more complex
    // subgraphs so we need to assure that two equivalent subcomponents are not
    // inserted in "base". The size of "base" is crux to determine the
    // processing time of the mining algorithm. Grown is NxNxNx...
    // (PATTERNS_GRAM times)
    for (unsigned int i = 0; i < frames.size(); i++) {
        if (ALLOWED_TOP_LEVEL_TYPES.find(frames.at(i).at(0).first) != ALLOWED_TOP_LEVEL_TYPES.end()) {
            baseSet.insert(frames.at(i));
        }
    }

    // Comb(n, p) 
    // n: number of possible components used to assemble a compoundFrame
    // p: compoundFrame gram
    floatUniverseCount = 1;
    unsigned int n = baseSet.size();
    for (unsigned int i = 0; i < PATTERNS_GRAM; i++) {
        floatUniverseCount = floatUniverseCount * ((float) (n - i));
        floatUniverseCount = floatUniverseCount / ((float) (i + 1));
    }

    // Copy the baseSet to a vector to ease further manipulation
    // (memory waste is irrelevant)
    vector<TypeFrame> base(baseSet.begin(), baseSet.end());
    if (DEBUG) printf("base.size(): %lu\n", base.size());

    // Only non-empty-intersection subcompounds are glued together to build a
    // compoundFrame (intersection == sharing of at least 1 Node). Building (a
    // priori) a list of non-empty-intersection elements for each element of 
    // "base" will ease the buiding of compoundFrames
    vector<vector<unsigned int>> neighbors;
    vector<unsigned int> aux;
    unsigned int progressIndicatorMax = 0;
    unsigned int progressIndicatorCount = 0;
    for (unsigned int i = 0; i < base.size(); i++) {
        if (DEBUG) printf("setting neighbors for: %u\n", i);
        if ((base.size() > 100) && (! (i % (base.size() / 100)))) logger().info("[PatternIndex] Step 1/2 %.0f%% done", ((float) i / base.size()) * 100);
        aux.clear();
        for (unsigned int j = i + 1; j < base.size(); j++) {
            if (base.at(i).nonEmptyNodeIntersection(base.at(j))) {
                aux.push_back(j);
            }
        }
        neighbors.push_back(aux);
        progressIndicatorMax += aux.size();
    }
    logger().info("[PatternIndex] Step 1/2 100%% done");

    // Populates the queue of compundFrames.
    // This queue is processed by the working threads
    vector<set<unsigned int>> selection;
    vector<vector<unsigned int>> selectionVector;
    vector<unsigned int> sizeVector;
    vector<unsigned int> v;
    set<unsigned int> s;
    TypeFrame compoundFrame;
    CartesianProductGenerator *cartesianGenerator;
    for (unsigned int i = 0; i < base.size(); i++) {
        // select a seed element from base
        if ((progressIndicatorMax > 10000) && (! (progressIndicatorCount % (progressIndicatorMax / 10000)))) logger().info("[PatternIndex] Step 2/2 %.2f%% done", ((float) progressIndicatorCount / progressIndicatorMax) * 100);
        progressIndicatorCount += neighbors.at(i).size();
        if (DEBUG) printf("base[%u] : %lu\n", i, neighbors.at(i).size());
        selection.clear();
        selectionVector.clear();
        sizeVector.clear();
        s.clear();
        s.insert(i);
        selection.push_back(s);
        for (unsigned int g = 1; g < PATTERNS_GRAM; g++) {
            s.clear();
            selection.push_back(s);
            for (set<unsigned int>::const_iterator it = selection.at(g - 1).begin(); it != selection.at(g - 1).end(); ++it) {
                selection.at(g).insert(neighbors.at(*it).begin(), neighbors.at(*it).end());
            }
        }
        for (unsigned int g = 0; g < PATTERNS_GRAM; g++) {
            v.clear();
            v.insert(v.end(), selection.at(g).begin(), selection.at(g).end());
            selectionVector.push_back(v);
            sizeVector.push_back(v.size());
        }
        // build every possible combination of PATTERNS_GRAM subcomponents that
        // have non-empty-intersection amongst themselves.
        cartesianGenerator = new CartesianProductGenerator(sizeVector, true, false);
        while (! cartesianGenerator->depleted()) {
            compoundFrame.clear();
            compoundFrame.push_back(TypePair(classserver().getType("AndLink"), 0));
            bool flag = true;
            for (unsigned int g = 0; g < PATTERNS_GRAM; g++) {
                if ((g > 0) && compoundFrame.containsEquivalent(base.at(selectionVector.at(g).at(cartesianGenerator->at(g))), 0)) {
                    // Avoids repeated subcomponents
                    flag = false;
                    break;
                } else {
                    compoundFrame.at(0).second++;
                    compoundFrame.append(base.at(selectionVector.at(g).at(cartesianGenerator->at(g))));
                }
            }
            if (flag) {
                if (DEBUG) {
                    printf("compound frame: (");
                    for (unsigned int g = 0; g < PATTERNS_GRAM; g++) {
                        printf("%u", selectionVector.at(g).at(cartesianGenerator->at(g)));
                        if (g < (PATTERNS_GRAM - 1)) {
                            printf(" ");
                        }
                    }
                    printf(")\n");
                    compoundFrame.printForDebug("", "\n");
                }
                if (enqueueCompoundFrame(compoundFrame)) {
                    usleep(1000000); // 1 second
                }
            }
            cartesianGenerator->generateNext();
        }
        delete cartesianGenerator;
    }
    logger().info("[PatternIndex] Step 2/2 100%% done");

    setCompoundFramesEnded();
    if (DEBUG) printf("Finished creating compound patterns. Waiting for threads to clear evaluation queue.\n");
    // The main thread finished its job of creating the compoundFrames so it
    // will sit down and wait for the working threads to finish their jobs.
    // So we start another working thread to avoid (potentially) letting an idle 
    // processor.
    evaluationThreads.push_back(new thread(&TypeFrameIndex::evaluatePatterns, this));

    for (unsigned int i = 0; i < evaluationThreads.size(); i++) {
        evaluationThreads.at(i)->join();
    }

    if (DEBUG) printf("Finished mining. heap size = %lu\n", miningResultsHeap.size());

    for (unsigned int i = 0; i < miningResultsHeap.size(); i++) {
        answer.push_back(miningResultsHeap.at(i));
    }
    logger().info("[PatternIndex] Finished mining");
}

void TypeFrameIndex::addSymmetricPermutations(TypeFrameSet &answer, const TypeFrame &frame, unsigned int cursor)
{
    unsigned int arity = frame.at(cursor).second;
    vector<int> argPos = frame.getArgumentsPosition(cursor);
    if (frame.typeAtIsSymmetricLink(cursor)) {
        if (arity == 2) {
            // optimization of commom case
            TypeFrame permutation;
            for (unsigned int i = 0; i <= cursor; i++) {
                permutation.pickAndPushBack(frame, i);
            }
            TypeFrame subFrame0 = frame.subFrameAt(argPos.at(0));
            TypeFrame subFrame1 = frame.subFrameAt(argPos.at(1));
            permutation.append(subFrame1);
            permutation.append(subFrame0);
            for (unsigned int i = permutation.size(); i < frame.size(); i++) {
                permutation.pickAndPushBack(frame, i);
            }
            answer.insert(permutation);
        } else {
            CartesianProductGenerator *cartesianGenerator;
            if (arity <= LIMIT_FOR_UNORDERED_LINKS_PERMUTATION) {
                cartesianGenerator = new CartesianProductGenerator(arity, arity, true, false);
                cartesianGenerator->generateNext(); // discard first combination (original order)
                TypeFrame permutation;
                while (! cartesianGenerator->depleted()) {
                    permutation.clear();
                    for (unsigned int i = 0; i <= cursor; i++) {
                        permutation.pickAndPushBack(frame, i);
                    }
                    for (unsigned int j = 0; j < arity; j++) {
                        permutation.append(frame.subFrameAt(argPos.at(cartesianGenerator->at(j))));
                    }
                    for (unsigned int i = permutation.size(); i < frame.size(); i++) {
                        permutation.pickAndPushBack(frame, i);
                    }
                    answer.insert(permutation);
                    cartesianGenerator->generateNext();
                }
                delete cartesianGenerator;
            }
        }
    }
    for (unsigned int i = 0; i < arity; i++) {
        addSymmetricPermutations(answer, frame, argPos.at(i));
    }
}

bool TypeFrameIndex::addFrame(TypeFrame &frame, int offset)
{
    if (DEBUG) frame.printForDebug("addFrame: ", "\n");
    bool exitStatus = true;
    if (frame.isValid() && frame.check()) {
        frames.push_back(frame);
        TypeFrameSet symmetricPermutations;
        addSymmetricPermutations(symmetricPermutations, frame, 0);
        for (TypeFrameSet::iterator it = symmetricPermutations.begin(); it != symmetricPermutations.end(); ++it) {
            if ((*it).check()) {
                frames.push_back(*it);
            } else {
                if (DEBUG) frame.printForDebug("frame = ", "\n");
                if (DEBUG) (*it).printForDebug("invalid permutation = ", "\n");
                throw runtime_error("Invalid symmetric permutation");
            }
        }
        if (DEBUG) {
            printf("%d: ", offset);
            frame.printForDebug("", "\n", true);
        }
        exitStatus = false;
    } else {
        logger().warn("[PatternIndex] DISCARDING INVALID FRAME (offset = " + to_string(offset) + ")");
        if (DEBUG) frame.printForDebug("INVALID FRAME: ", "\n");
    }

    return exitStatus;
}

bool TypeFrameIndex::add(Handle handle, int offset)
{
    bool exitStatus = true;
    TypeFrame frame(handle);
    if (frame.isValid()) {
        exitStatus = addFrame(frame, offset);
    } else {
        printf("INVALID FRAME <%s>\n", handle->toString().c_str());
    }
    return exitStatus;

}

bool TypeFrameIndex::addFromScheme(const string &txt, int offset)
{
    bool exitStatus = true;
    TypeFrame frame(txt);
    if (frame.isValid()) {
        exitStatus = addFrame(frame, offset);
    } else {
        if (DEBUG) printf("INVALID FRAME <%s>\n", txt.c_str());
    }
    return exitStatus;
}

void TypeFrameIndex::addArity2Patterns(vector<TypeFrame> &answer,
                                       vector<TypeFrame> &recurseResult1,
                                       vector<TypeFrame> &recurseResult2,
                                       TypeFrame &baseFrame,
                                       int cursor)
{
    TypeFrame pattern;

    // Link * *
    pattern = TypeFrame::EMPTY_PATTERN;
    pattern.pickAndPushBack(baseFrame, cursor); 
    pattern.push_back(TypeFrame::STAR_PATTERN); 
    pattern.push_back(TypeFrame::STAR_PATTERN); 

    if (DEBUG) {
        printf("Adding * * 2-link pattern\n");
        pattern.printForDebug("", "\n", true);
    }

    answer.push_back(pattern);

    // Link [recurse1] *
    for (unsigned int i = 0; i < recurseResult1.size(); i++) {
        pattern = TypeFrame::EMPTY_PATTERN;
        pattern.pickAndPushBack(baseFrame, cursor); 
        pattern.append(recurseResult1.at(i));
        pattern.push_back(TypeFrame::STAR_PATTERN);
        if (DEBUG) {
            printf("Adding [%u] * 2-link pattern\n", i);
            pattern.printForDebug("", "\n", true);
        }
        answer.push_back(pattern);
    }

    // Link * [recurse2]
    for (unsigned int j = 0; j < recurseResult2.size(); j++) {
        pattern = TypeFrame::EMPTY_PATTERN;
        pattern.pickAndPushBack(baseFrame, cursor);
        pattern.push_back(TypeFrame::STAR_PATTERN); 
        pattern.append(recurseResult2.at(j));
        if (DEBUG) {
            printf("Adding * [%u] 2-link pattern\n", j);
            pattern.printForDebug("", "\n", true);
        }
        answer.push_back(pattern);
    }

    // Link [recurse1] [recurse2]
    for (unsigned int i = 0; i < recurseResult1.size(); i++) {
        for (unsigned int j = 0; j < recurseResult2.size(); j++) {
            pattern = TypeFrame::EMPTY_PATTERN;
            pattern.pickAndPushBack(baseFrame, cursor);
            pattern.append(recurseResult1.at(i));
            pattern.append(recurseResult2.at(j));
            if (DEBUG) {
                printf("Adding [%u] [%u] 2-link pattern\n", i, j);
                pattern.printForDebug("", "\n", true);
            }
            answer.push_back(pattern);
        }
    }
}

// TODO: This method should allocate answers in the heap instead of the stack
// to avoid copying data all the away in the recursive calls
// TODO: Break this method in smaller pieces
vector<TypeFrame> TypeFrameIndex::computeSubPatterns(TypeFrame &baseFrame,
                                                          int cursor,
                                                          int pos)
{
    vector<TypeFrame> answer;
    if (DEBUG) baseFrame.printForDebug("baseFrame = ", "\n");

    unsigned int headArity = baseFrame.at(cursor).second;
    bool symmetricHead = baseFrame.typeAtIsSymmetricLink(cursor);
    vector<int> argPos = baseFrame.getArgumentsPosition(cursor);

    if (headArity == 0) {

        // Node

        //TypeFrame pattern1;
        TypeFrame pattern2;
        //pattern1.push_back(baseFrame.at(cursor));    // discards node name
        pattern2.pickAndPushBack(baseFrame, cursor); // uses node name

        if (DEBUG) {
            printf("Adding node pattern\n");
            printf("Name: %s\n", baseFrame.nodeNameAt(cursor).c_str());
            //pattern1.printForDebug("pattern1: ", "\n", true);
            pattern2.printForDebug("pattern2: ", "\n", true);
        }

        //answer.push_back(pattern1); 
        answer.push_back(pattern2);

    } else if (headArity == 2) {

        // Arity 2 Link, computes all possible patterns

        vector<TypeFrame> recurseResult1, recurseResult2;

        recurseResult1 = computeSubPatterns(baseFrame, argPos.at(0), pos);
        recurseResult2 = computeSubPatterns(baseFrame, argPos.at(1), pos);
        if (DEBUG) {
            for (unsigned int i = 0; i < recurseResult1.size(); i++) {
                recurseResult1.at(i).printForDebug("recurseResult1: ", "\n", true);
            }
            for (unsigned int i = 0; i < recurseResult2.size(); i++) {
                recurseResult2.at(i).printForDebug("recurseResult2: ", "\n", true);
            }
        }

        addArity2Patterns(answer, recurseResult1, recurseResult2, baseFrame, cursor);
        if (symmetricHead) {
            addArity2Patterns(answer, recurseResult2, recurseResult1, baseFrame, cursor);
        }

    } else {

        // Arity != 2 Link, doesn't compute all possible patterns (to avoid
        // combinatorial explosion). For an N-Arity Link, computes: 
        //
        // Link  *  *  * ...  *
        // Link T1  *  * ...  *
        // Link  * T2  * ...  *
        // Link  *  * T3 ...  *
        // Link  *  *  * ... TN
          
        // TODO: It would probably be OK to compute all patterns for 3-arity
        // links. So this should be done here and let this "generic" computation
        // only for links with arity > 3.

        vector<TypeFrame> recurseResult[headArity];
        for (unsigned int k = 0; k < headArity; k++) {
            recurseResult[k] = computeSubPatterns(baseFrame, argPos.at(k), pos);
        }

        TypeFrame pattern;

        // Link * * * ... *
        pattern = TypeFrame::EMPTY_PATTERN;
        pattern.pickAndPushBack(baseFrame, cursor);
        for (unsigned int k = 0; k < headArity; k++) {
            pattern.push_back(TypeFrame::STAR_PATTERN);
        }
        if (DEBUG) {
            printf("Adding * link pattern\n");
            pattern.printForDebug("", "\n", true);
        }
        answer.push_back(pattern);

        for (unsigned int k = 0; k < headArity; k++) {
            for (unsigned int i = 0; i < recurseResult[k].size(); i++) {
                pattern = TypeFrame::EMPTY_PATTERN;
                pattern.pickAndPushBack(baseFrame, cursor);
                for (unsigned int m = 0; m < k; m++) {
                    pattern.push_back(TypeFrame::STAR_PATTERN);
                }
                pattern.append(recurseResult[k].at(i));
                for (unsigned int m = k + 1; m < headArity; m++) {
                    pattern.push_back(TypeFrame::STAR_PATTERN);
                }
                if (DEBUG) {
                    printf("Adding [] [] link pattern (%u %u)\n", k, i);
                    pattern.printForDebug("", "\n", true);
                }
                answer.push_back(pattern);
            }
        }
        if (DEBUG) printf("Done [] []\n");
        if (DEBUG) printf("Size =  %lu\n", answer.size());
    }

    if (! TOPLEVEL_ONLY) {
        clock_t t1 = clock();
        for (unsigned int i = 0; i < answer.size(); i++) {
            if (DEBUG) printf("Adding %u/%lu\n", i, answer.size());
            addPatternOccurrence(answer.at(i), pos);
        }
        clock_t t2 = clock();
        double delta = ((double) (t2 - t1)) / CLOCKS_PER_SEC;
        time2 += delta;
    }
    return answer;
}

void TypeFrameIndex::addPatternOccurrence(TypeFrame &pattern, int pos)
{
    occurrenceUniverse.insert(pos);
    pattern.computeHashCode();
    PatternMap::iterator it = occurrenceSet.find(pattern);
    if (it == occurrenceSet.end()) {
        if (DEBUG) {
            printf("%d: ", pos);
            pattern.printForDebug("ADD NEW PATTERN ", "\n", true);
        }
        IntegerSet newSet;
        newSet.insert(pos);
        occurrenceSet.insert(PatternMap::value_type(pattern, newSet));
    } else {
        if (DEBUG) {
            printf("%d: ", pos);
            pattern.printForDebug("ADD PATTERN ", "\n", true);
            (*it).first.printForDebug("Found: ", "\n", true);
        }
        (*it).second.insert(pos);
    }
}

void TypeFrameIndex::buildSubPatternsIndex()
{
    if (DEBUG) printf("TypeFrameIndex::buildSubPatternsIndex()\nframes.size() = %lu\n", frames.size());
    for (unsigned int i = 0; i < frames.size(); i++) {
        if ((frames.size() > 100) && (! (i % (frames.size() / 100)))) logger().info("[PatternIndex] Building index %.0f%% done", ((float) i / frames.size()) * 100);
        TypeFrame currentFrame = frames.at(i);
        if (DEBUG) printf("Computing subpatterns of %u\n", i);
        if (DEBUG) currentFrame.printForDebug("currentFrame = ", "\n");
        clock_t t1 = clock();
        vector<TypeFrame> patterns = computeSubPatterns(currentFrame, 0, i);
        clock_t t2 = clock();
        double delta = ((double) (t2 - t1)) / CLOCKS_PER_SEC;
        time1 += delta;
        if (DEBUG) printf("%.2f\n", (time2 / time1) * 100);
        if (DEBUG) printf("Computing subpatterns of %u - DONE\n", i);
        if (TOPLEVEL_ONLY) {
            addPatternOccurrence(currentFrame, i);
            for (unsigned int j = 0; j < patterns.size(); j++) {
                addPatternOccurrence(patterns.at(j), i);
            }
        }
    }
    logger().info("[PatternIndex] Building index 100%% done");
    if (DEBUG) printForDebug(true);
}

void TypeFrameIndex::selectCurrentElement(TypeFrame &answer,
                                          StringMap &variableOccurrences,
                                          const TypeFrame &baseFrame,
                                          int cursor) const
{
    if (baseFrame.typeAtEqualsTo(cursor, "VariableNode")) {
        answer.push_back(TypeFrame::STAR_PATTERN);
        string key = baseFrame.nodeNameAt(cursor);
        StringMap::iterator it = variableOccurrences.find(key);
        if (it == variableOccurrences.end()) {
            IntegerSet newSet;
            newSet.insert(answer.size() - 1);
            variableOccurrences.insert(StringMap::value_type(key, newSet));
            if (DEBUG) printf("ADD NEW SET variable occurrence %s %lu\n", key.c_str(), answer.size() - 1);
        } else {
            if (DEBUG) printf("ADD variable occurrence %s %lu\n", key.c_str(), answer.size() - 1);
            (*it).second.insert(answer.size() - 1);
        }
    } else {
        answer.pickAndPushBack(baseFrame, cursor);
    }
}


void TypeFrameIndex::buildConstraints(IntPairVector &constraints,
                                      StringMap &variableOccurrences) const
{
    constraints.clear();
    StringMap::iterator it = variableOccurrences.begin();
    while (it != variableOccurrences.end()) {
        vector<int> v((*it).second.begin(), (*it).second.end());
        for (unsigned int i = 0; i < v.size(); i++) {
            for (unsigned int j = i + 1; j < v.size(); j++) {
                constraints.emplace_back(v.at(i), v.at(j));
            }
        }
        ++it;
    }
}

void TypeFrameIndex::buildQueryTerm(TypeFrame &answer,
                                    StringMap &variableOccurrences,
                                    const TypeFrame &baseFrame,
                                    int cursor) const
{

    selectCurrentElement(answer, variableOccurrences, baseFrame, cursor);
    vector<int> argPos = baseFrame.getArgumentsPosition(cursor);
    for (unsigned int i = 0; i < argPos.size(); i++) {
        if (baseFrame.at(argPos.at(i)).second == 0) {
            selectCurrentElement(answer, variableOccurrences, baseFrame, argPos.at(i));
        } else {
            buildQueryTerm(answer, variableOccurrences, baseFrame, argPos.at(i));
        }
    }
}

void TypeFrameIndex::query(vector<ResultPair> &result,
                           const string &queryScm) const
{
    TypeFrame queryFrame(queryScm);
    query(result, queryFrame, DIFFERENT_ASSIGNMENT_FOR_DIFFERENT_VARS, PERMUTATIONS_OF_VARS_CONSIDERED_EQUIVALENT);
}

void TypeFrameIndex::query(vector<ResultPair> &result,
                           const TypeFrame &queryFrame) const
{
    query(result, queryFrame, DIFFERENT_ASSIGNMENT_FOR_DIFFERENT_VARS, PERMUTATIONS_OF_VARS_CONSIDERED_EQUIVALENT);
}

void TypeFrameIndex::query(vector<ResultPair> &result,
                           const TypeFrame &queryFrame,
                           bool distinct,
                           bool noPermutations) const
{
    TypeFrame keyExpression;
    vector<VarMapping> forbiddenMappings;
    int headLogicOperator;

    query(result, keyExpression, forbiddenMappings, headLogicOperator, queryFrame, 0, distinct, noPermutations);
}

bool TypeFrameIndex::compatibleVarMappings(const VarMapping &map1,
                                           const VarMapping &map2,
                                           bool distinct) const
{
    if (DEBUG) {
        printf("TypeFrameIndex::compatibleVarMappings()\n");
        printVarMapping(map1);
        printf("\n");
        printVarMapping(map2);
    }

    for (VarMapping::const_iterator it1 = map1.begin(); it1 != map1.end(); ++it1) {
        VarMapping::const_iterator it2 = map2.find((*it1).first);
        if ((it2 != map2.end()) && (! (*it1).second.equals((*it2).second))) {
            if (DEBUG) {
                printf("Failed at %s\n", (*it1).first.c_str());
            }
            return false;
        }
    }
    if (distinct) {
        for (VarMapping::const_iterator it1 = map1.begin(); it1 != map1.end(); ++it1) {
            for (VarMapping::const_iterator it2 = map2.begin(); it2 != map2.end(); ++it2) {
                if (((*it1).first.compare((*it2).first) != 0) && ((*it1).second.equals((*it2).second))) {
                    if (DEBUG) {
                        printf("* Failed at %s\n", (*it1).first.c_str());
                    }
                    return false;
                }
            }
        }
    }

    return true;
}

void TypeFrameIndex::typeFrameSetUnion(TypeFrameSet &answer,
                                       const TypeFrameSet &set1,
                                       const TypeFrameSet &set2) const
{
    answer.clear();
    for (TypeFrameSet::const_iterator it1 = set1.begin(); it1 != set1.end(); ++it1) {
        answer.insert(*it1);
    }
    for (TypeFrameSet::const_iterator it2 = set2.begin(); it2 != set2.end(); ++it2) {
        answer.insert(*it2);
    }
}

void TypeFrameIndex::varMappingUnion(VarMapping &answer,
                                     const VarMapping &map1,
                                     const VarMapping &map2) const
{
    for (VarMapping::const_iterator it = map1.begin(); it != map1.end(); ++it) {
        answer.insert(*it);
    }
    for (VarMapping::const_iterator it = map2.begin(); it != map2.end(); ++it) {
        answer.insert(*it);
    }
}

bool TypeFrameIndex::isForbiddenMapping(const VarMapping &mapping,
                                        const vector<VarMapping> &forbiddenVector) const
{
    bool answer = false;
    for (unsigned int i = 0; i < forbiddenVector.size(); i++) {
        bool match = true;
        for (VarMapping::const_iterator it1 = forbiddenVector.at(i).begin(); it1 != forbiddenVector.at(i).end(); ++it1) {
            VarMapping::const_iterator it2 = mapping.find((*it1).first);
            if ((it2 != mapping.end()) && (! (*it1).second.equals((*it2).second))) {
                match = false;
                break;
            }
        }
        if (match) {
            answer = true;
            break;
        }
    }

    return answer;
}

void TypeFrameIndex::query(vector<ResultPair> &answer,
                           TypeFrame &keyExpression,
                           vector<VarMapping> &forbiddenMappings,
                           int &logicOperator,
                           const TypeFrame &queryFrame,
                           int cursor,
                           bool distinct,
                           bool noPermutations) const
{
    if (DEBUG) queryFrame.printForDebug("Query frame: ", "\n", true);
    if (DEBUG) printf("Cursor: %d\n", cursor);
    vector<ResultPair> unfilteredAnswer;
    answer.clear();
    keyExpression.clear();
    forbiddenMappings.clear();
    unsigned int arity = queryFrame.at(cursor).second;
    vector<int> argPos = queryFrame.getArgumentsPosition(cursor);
    vector<vector<ResultPair>> recursionQueryResult(arity);
    vector<TypeFrame> recursionKeyExpression(arity);
    vector<vector<VarMapping>> recursionForbiddenMappings(arity);
    vector<int> recursionHeadLogicOperator(arity);
    bool AndFlag = queryFrame.typeAtEqualsTo(cursor, "AndLink");
    bool OrFlag = queryFrame.typeAtEqualsTo(cursor, "OrLink");
    bool NotFlag = queryFrame.typeAtEqualsTo(cursor, "NotLink");
    if (DEBUG) printf("Head is %s\n", (AndFlag ? "AND" : (OrFlag ? "OR" : (NotFlag ? "NOT" : "LEAF"))));

    if (AndFlag || OrFlag || NotFlag) {
        // Recursive call on each clause
        keyExpression.pickAndPushBack(queryFrame, cursor);
        for (unsigned int i = 0; i < arity; i++) {
            query(recursionQueryResult.at(i), recursionKeyExpression.at(i), recursionForbiddenMappings.at(i), recursionHeadLogicOperator.at(i), queryFrame, argPos.at(i), distinct, false);
        }
        for (unsigned int i = 0; i < arity; i++) {
            if (DEBUG) {
                printf("recursion key[%d]: ", i);
                recursionKeyExpression.at(i).printForDebug("", "\n", true);
            }
            keyExpression.append(recursionKeyExpression.at(i));
        }
        if (DEBUG) {
            keyExpression.printForDebug("Resulting key: ", "\n", true);
            for (unsigned int i = 0; i < arity; i++) {
                printf("Recursion result [%u]:\n", i);
                printRecursionResult(recursionQueryResult.at(i));
            }
        }
    }

    // Process the returned (recursive) answers according to the type of logical
    // operation at this level.
    if (AndFlag) {
        if (DEBUG) printf("Start processing AND\n");
        logicOperator = OPERATOR_AND;
        for (unsigned int i = 0; i < arity; i++) {
            for (unsigned int j = 0; j < recursionForbiddenMappings.at(i).size(); j++) {
                if (DEBUG) {
                    printf("(AND) Adding forbidden mapping:\n");
                    printVarMapping(recursionQueryResult.at(i).at(j).second);
                }
                forbiddenMappings.push_back(recursionForbiddenMappings.at(i).at(j));
            }
        }
        vector<vector<ResultPair>> cleanRecursionQueryResult(arity);
        for (unsigned int i = 0; i < arity; i++) {
            if (DEBUG) printf("Branch: %u\n", i);
            for (unsigned int j = 0; j < recursionQueryResult.at(i).size(); j++) {
                if (! isForbiddenMapping(recursionQueryResult.at(i).at(j).second, forbiddenMappings)) {
                    cleanRecursionQueryResult.at(i).push_back(recursionQueryResult.at(i).at(j));
                    if (DEBUG) {
                        printf("Pushing:\n");
                        printTypeFrameSet(recursionQueryResult.at(i).at(j).first);
                    }
                } else {
                    if (DEBUG) {
                        printf("Forbidden:\n");
                        printTypeFrameSet(recursionQueryResult.at(i).at(j).first);
                    }
                }
            }
        }
        vector<ResultPair> aux[2];
        int src = 0;
        int tgt = 1;

        unsigned int baseBranch = 0;
        bool selectedFlag = false;
        for (unsigned int i = 0; i < arity; i++) {
            if (recursionHeadLogicOperator.at(i) != OPERATOR_NOT) {
                baseBranch = i;
                selectedFlag = true;
                for (unsigned int j = 0; j < cleanRecursionQueryResult.at(baseBranch).size(); j++) {
                    aux[tgt].push_back(cleanRecursionQueryResult.at(baseBranch).at(j));
                    if (DEBUG) {
                        printf("(AND) Adding solution to result:\n");
                        printTypeFrameSet(cleanRecursionQueryResult.at(baseBranch).at(j).first);
                        printVarMapping(cleanRecursionQueryResult.at(baseBranch).at(j).second);
                    }
                }
                break;
            }
        }
        // TODO: raise an exception if selectedFlag == false
        int switchCount = 1;
        for (unsigned int i = 0; i < arity; i++) {
            if (selectedFlag && (i != baseBranch) && (recursionHeadLogicOperator.at(i) != OPERATOR_NOT)) {
                src = switchCount++ % 2;
                tgt = 1 - src;
                aux[tgt].clear();
                TypeFrameSet newSet;
                for (unsigned int b = 0; b < cleanRecursionQueryResult.at(i).size(); b++) {
                    for (unsigned int a = 0; a < aux[src].size(); a++) {
                        if (compatibleVarMappings(aux[src].at(a).second, cleanRecursionQueryResult.at(i).at(b).second, distinct)) {
                            TypeFrameSet newSet;
                            VarMapping newMapping;
                            typeFrameSetUnion(newSet, aux[src].at(a).first, cleanRecursionQueryResult.at(i).at(b).first);
                            varMappingUnion(newMapping, aux[src].at(a).second, cleanRecursionQueryResult.at(i).at(b).second);
                            if (DEBUG) {
                                printf("(AND) Adding solution to result:\n");
                                printf("Base:\n");
                                printTypeFrameSet(aux[src].at(a).first);
                                printVarMapping(aux[src].at(a).second);
                                printf("Adding:\n");
                                printTypeFrameSet(cleanRecursionQueryResult.at(i).at(b).first);
                                printVarMapping(cleanRecursionQueryResult.at(i).at(b).second);
                                printf("Union:\n");
                                printTypeFrameSet(newSet);
                                printVarMapping(newMapping);
                            }
                            aux[tgt].emplace_back(newSet, newMapping);
                        } else {
                            if (DEBUG) {
                                printf("(AND) rejecting non-compatible var maps:\n");
                            }
                        }
                    }
                }
            }
        }
        for (unsigned int j = 0; j < aux[tgt].size(); j++) {
            if (noPermutations) {
                unfilteredAnswer.push_back(aux[tgt].at(j));
            } else {
                answer.push_back(aux[tgt].at(j));
            }
        }
    } else if (OrFlag) {
        if (DEBUG) printf("Start processing OR\n");
        logicOperator = OPERATOR_OR;
        for (unsigned int i = 0; i < arity; i++) {
            for (unsigned int j = 0; j < recursionQueryResult.at(i).size(); j++) {
                if (! isForbiddenMapping(recursionQueryResult.at(i).at(j).second, recursionForbiddenMappings.at(i))) {
                    if (DEBUG) {
                        printf("(OR) Adding solution to result:\n");
                        printTypeFrameSet(recursionQueryResult.at(i).at(j).first);
                        printVarMapping(recursionQueryResult.at(i).at(j).second);
                    }
                    if (noPermutations) {
                        unfilteredAnswer.push_back(recursionQueryResult.at(i).at(j));
                    } else {
                        answer.push_back(recursionQueryResult.at(i).at(j));
                    }
                } else {
                    if (DEBUG) {
                        printf("(OR) Rejecting solution:\n");
                        printTypeFrameSet(recursionQueryResult.at(i).at(j).first);
                        printVarMapping(recursionQueryResult.at(i).at(j).second);
                    }
                }
            }
        }
    } else if (NotFlag) {
        if (DEBUG) printf("Start processing NOT\n");
        logicOperator = OPERATOR_NOT;
        // arity is 1
        for (unsigned int j = 0; j < recursionQueryResult.at(0).size(); j++) {
            if (DEBUG) {
                printf("(NOT) Adding solution to result:\n");
                printTypeFrameSet(recursionQueryResult.at(0).at(j).first);
                printVarMapping(recursionQueryResult.at(0).at(j).second);
            }
            if (noPermutations) {
                unfilteredAnswer.push_back(recursionQueryResult.at(0).at(j));
            } else {
                answer.push_back(recursionQueryResult.at(0).at(j));
            }
            if (! isForbiddenMapping(recursionQueryResult.at(0).at(j).second, recursionForbiddenMappings.at(0))) {
                if (DEBUG) {
                    printf("(NOT) Adding forbidden mapping:\n");
                    printVarMapping(recursionQueryResult.at(0).at(j).second);
                }
                forbiddenMappings.push_back(recursionQueryResult.at(0).at(j).second);
            }
        }
    } else {
        logicOperator = OPERATOR_NOP;
        IntPairVector constraints;
        StringMap variableOccurrences;
        buildQueryTerm(keyExpression, variableOccurrences, queryFrame, cursor);
        buildConstraints(constraints, variableOccurrences);
        if (DEBUG) {
            keyExpression.printForDebug("Key: ", "\n", true);
            for (StringMap::iterator it1 = variableOccurrences.begin(); it1 != variableOccurrences.end(); ++it1) {
                printf("%s: ", (*it1).first.c_str());
                for (IntegerSet::iterator it2 = (*it1).second.begin(); it2 != (*it1).second.end(); ++it2) {
                    printf("%d ", (*it2));
                }
                printf("\n");
            }
            for (unsigned int i = 0; i < constraints.size(); i++) {
                printf("%d %d\n", constraints.at(i).first, constraints.at(i).second);
            }
        }
        PatternMap::const_iterator it1 = occurrenceSet.find(keyExpression);
        if (it1 != occurrenceSet.end()) {
            for (IntegerSet::iterator it2 = (*it1).second.begin(); it2 != (*it1).second.end(); ++it2) {
                vector<int> mapping;
                if (frames.at(*it2).match(mapping, keyExpression, constraints)) {
                    VarMapping varMap;
                    TypeFrameSet frameSet;
                    for (StringMap::iterator it = variableOccurrences.begin(); it != variableOccurrences.end(); ++it) {
                        varMap.insert(VarMapping::value_type((*it).first, frames.at(*it2).subFrameAt(*((*it).second.begin()))));
                    }
                    frameSet.insert(frames.at(*it2));
                    if (DEBUG) {
                        printf("(LEAF) Adding solution to result:\n");
                        printTypeFrameSet(frameSet);
                        printVarMapping(varMap);
                    }
                    if (noPermutations) {
                        unfilteredAnswer.emplace_back(frameSet, varMap);
                    } else {
                        answer.emplace_back(frameSet, varMap);
                    }
                }
            }
        }
    }

    if (noPermutations) {
        for (unsigned int i = 0; i < unfilteredAnswer.size(); i++) {
            bool flag = true;
            for (unsigned int j = 0; j < answer.size(); j++) {
                if (equivalentVarMappings(unfilteredAnswer.at(i).second, answer.at(j).second, distinct)) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                answer.push_back(unfilteredAnswer.at(i));
            }
        }
    }

    if (DEBUG) {
        printf("Answer:\n");
        printRecursionResult(answer);
    }
}

bool TypeFrameIndex::mapCover(const VarMapping &map1, const VarMapping &map2) const
{
    TypeFrameSet set;
    for (VarMapping::const_iterator it = map1.begin(); it != map1.end(); ++it) {
        set.insert((*it).second);
    }
    bool answer = true;
    for (VarMapping::const_iterator it = map2.begin(); it != map2.end(); ++it) {
        if (set.find((*it).second) == set.end()) {
            answer = false;
            break;
        }
    }

    return answer;
}

bool TypeFrameIndex::equivalentVarMappings(const VarMapping &map1,
                                           const VarMapping &map2,
                                           bool distinct) const
{
    return (distinct ? mapCover(map1, map2) : (mapCover(map1, map2) && mapCover(map2, map1)));
}

void TypeFrameIndex::printFrameVector(const vector<TypeFrame> &v) const
{
    for (unsigned int i = 0; i < v.size(); i++) {
        printf("v[%u] = ", i);
        v.at(i).printForDebug("", "\n");
    }
}

void TypeFrameIndex::printVarMapping(const VarMapping &map) const
{
    for (VarMapping::const_iterator it = map.begin(); it != map.end(); ++it) {
        printf("%s = ", (*it).first.c_str());
        (*it).second.printForDebug("", "\n", true);
    }
}

void TypeFrameIndex::printTypeFrameSet(const TypeFrameSet &set) const
{
    for (TypeFrameSet::const_iterator it = set.begin(); it != set.end(); ++it) {
        (*it).printForDebug("", "\n", true);
    }
}

void TypeFrameIndex::printRecursionResult(const vector<ResultPair> &v) const
{
    for (unsigned int i = 0; i < v.size(); i++) {
        printf("Solution %u\n" , i);
        printTypeFrameSet(v.at(i).first);
        printVarMapping(v.at(i).second);
    }
}

void TypeFrameIndex::printForDebug(bool showNodeNames) const
{
    PatternMap::const_iterator it1 = occurrenceSet.begin();
    while (it1 != occurrenceSet.end()) {
        (*it1).first.printForDebug("[", "]: ", showNodeNames);
        IntegerSet::iterator it2 = (*it1).second.begin();
        while (it2 != (*it1).second.end()) {
            printf("%d ", *it2);
            ++it2;
        }
        printf("\n");
        ++it1;
    }
}

