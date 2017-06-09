#include <chrono>
#include <limits>
#include <unistd.h>

#include "TypeFrameIndex.h"
#include "TypeFrame.h"
#include "CombinationGenerator.h"
#include "PartitionGenerator.h"
#include "CartesianProductGenerator.h"

using namespace opencog;

unsigned int TypeFrameIndex::LIMIT_FOR_SYMMETRIC_LINKS_PERMUTATION = 5;
bool TypeFrameIndex::PATTERN_COUNT_CACHE_ENABLED = false;
bool TypeFrameIndex::INDEPENDENT_SUBPATTERN_PROB_CACHE_ENABLED = false;
bool TypeFrameIndex::PATTERN_QUALITTY_CACHE_ENABLED = false;
unsigned int TypeFrameIndex::MINIMAL_FREQUENCY_TO_COMPUTE_SURPRISINGNESS = 5;
unsigned int TypeFrameIndex::NUMBER_OF_EVALUATION_THREADS = 4;
unsigned int TypeFrameIndex::MAX_SIZE_OF_COMPOUND_FRAMES_QUEUE = 5000000;

const int TypeFrameIndex::OPERATOR_NOP = 0;
const int TypeFrameIndex::OPERATOR_AND = 1;
const int TypeFrameIndex::OPERATOR_OR = 2;
const int TypeFrameIndex::OPERATOR_NOT = 3;
TypeFrameIndex::CoherenceFunction TypeFrameIndex::coherenceFunction = CONST_1;
TypeFrameIndex::CoherenceModulatorG TypeFrameIndex::coherenceModulatorG = ONE_OVER_COHERENCE;
TypeFrameIndex::CoherenceModulatorH TypeFrameIndex::coherenceModulatorH = COHERENCE;

TypeFrameIndex::TypeFrameIndex() 
{
    auxVar1.push_back(TypePair(classserver().getType("VariableNode"), 0));
    auxVar1.setNodeNameAt(0, "V1");
}

TypeFrameIndex::~TypeFrameIndex() 
{
}

float TypeFrameIndex::computeCoherence(const TypeFrame &frame) const
{
    switch (coherenceFunction) {
        case CONST_1: return 1;
        default: throw std::runtime_error("Unknown coherence function\n");
    }
}

float TypeFrameIndex::one_over_x(float x) const
{
    if (x < std::numeric_limits<float>::epsilon()) {
        return std::numeric_limits<float>::max();
    } else {
        return 1 / x;
    }
}

float TypeFrameIndex::gFunction(float x) const
{
    switch (coherenceModulatorG) {
        case ONE_OVER_COHERENCE: return one_over_x(x);
        default: throw std::runtime_error("Unknown coherence modulator G function\n");
    }
}

float TypeFrameIndex::hFunction(float x) const
{
    switch (coherenceModulatorH) {
        case COHERENCE: return x;
        default: throw std::runtime_error("Unknown coherence modulator H function\n");
    }
}

TypeFrame TypeFrameIndex::getFrameAt(int index)
{
    return frames.at(index);
}

void TypeFrameIndex::addPatterns(std::vector<TypeFrame> &answer, const TypeFrame &base) const
{
    //answer.push_back(base);
    TypeFrameSet nodes;
    base.buildNodesSet(nodes, true, true);
    CombinationGenerator selected(nodes.size(), true, false);

    if (DEBUG) {
        base.printForDebug("Compound frame: ", "\n", true);
        printf("Nodes: ");
        for (TypeFrameSet::iterator it = nodes.begin(); it != nodes.end(); it++) {
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
        for (TypeFrameSet::iterator it = nodes.begin(); it != nodes.end(); it++) {
            if (selected.at(count)) {
                std::string varName = "V" + std::to_string(count);
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

float TypeFrameIndex::computeQuality(const TypeFrame &pattern, RankingMetric metric)
{
    if (PATTERN_QUALITTY_CACHE_ENABLED) {
        PatternFloatMap::const_iterator it = patternQualityCache.find(pattern);
        if (it != patternQualityCache.end()) {
            return (*it).second;
        }
    }

    float answer = 0;
    switch (metric) {
        case I_SURPRISINGNESS: answer = computeISurprinsingness(pattern, false); break;
        case N_I_SURPRISINGNESS: answer = computeISurprinsingness(pattern, true); break;
        case II_SURPRISINGNESS: answer = computeIISurprinsingness(pattern, false); break;
        case N_II_SURPRISINGNESS: answer = computeIISurprinsingness(pattern, true); break;
        default: throw std::runtime_error("Unknown pattern ranking metric\n");
    }

    if (PATTERN_QUALITTY_CACHE_ENABLED) {
        patternQualityCache.insert(PatternFloatMap::value_type(pattern, answer));
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

    std::vector<TypeFrameIndex::ResultPair> queryAnswer;
    query(queryAnswer, pattern, true, false);
    unsigned int answer = queryAnswer.size();

    if (PATTERN_COUNT_CACHE_ENABLED) {
        patternCountCache.insert(PatternCountMap::value_type(pattern, answer));
    }

    return answer;
}

void TypeFrameIndex::addSupersetFrames(std::vector<TypeFrame> &answer, const TypeFrame &frame) const
{
    std::vector<TypeFrame> notUsed;
    addInferredSetFrames(answer, notUsed, frame, false);
}

void TypeFrameIndex::addSubsetFrames(std::vector<TypeFrame> &subset, std::vector<TypeFrame> &star, const TypeFrame &frame) const
{
    addInferredSetFrames(subset, star, frame, true);
}

void TypeFrameIndex::addInferredSetFrames(std::vector<TypeFrame> &subset, std::vector<TypeFrame> &star, const TypeFrame &frame, bool subSetFlag) const
{
    subset.clear();
    TypeFrameSet nodes;
    frame.buildNodesSet(nodes, false, true);
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
        for (TypeFrameSet::iterator it = nodes.begin(); it != nodes.end(); it++) {
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

std::pair<float,float> TypeFrameIndex::minMaxIndependentProb(const TypeFrame &pattern)
{
    if (INDEPENDENT_SUBPATTERN_PROB_CACHE_ENABLED) {
        SubPatternProbMap::const_iterator it = subPatternProbCache.find(pattern);
        if (it != subPatternProbCache.end()) {
            return (*it).second;
        }
    }

    float minP = 0;
    float maxP = 0;

    if (pattern.typeAtEqualsTo(0, "AndLink")) {
        if (DEBUG) printf("pattern is AndLink\n");
        minP = std::numeric_limits<float>::max();
        unsigned int n = pattern.at(0).second;
        std::vector<int> argPos = pattern.getArgumentsPosition(0);
        PartitionGenerator partitionGenerator(n);
        TypeFrame subPattern;
        TypeFrame aux;
        while (! partitionGenerator.depleted()) {
            if (DEBUG) partitionGenerator.printForDebug("Partition: ", "\n");
            PartitionGenerator::IntegerSetSet partition = partitionGenerator.getPartition();
            float prod = 1;
            for (PartitionGenerator::IntegerSetSet::const_iterator it1 = partition.begin(); it1 != partition.end(); it1++) {
                subPattern.clear();
                if ((*it1).size() > 1) {
                    subPattern.push_back(TypePair(classserver().getType("AndLink"), (*it1).size()));
                }
                for (PartitionGenerator::IntegerSet::const_iterator it2 = (*it1).begin(); it2 != (*it1).end(); it2++) {
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

    std::pair<float, float> answer = std::make_pair(minP, maxP);
    if (INDEPENDENT_SUBPATTERN_PROB_CACHE_ENABLED) {
        subPatternProbCache.insert(SubPatternProbMap::value_type(pattern, answer));
    }
    
    return answer;
}

std::pair<float,float> TypeFrameIndex::minMaxSupersetProb(const TypeFrame &pattern)
{
    float minP = 0;
    float maxP = 0;

    if (pattern.typeAtEqualsTo(0, "AndLink")) {
        if (DEBUG) printf("pattern is AndLink\n");
        unsigned int n = pattern.at(0).second;
        std::vector<int> argPos = pattern.getArgumentsPosition(0);
        TypeFrame aux;
        TypeFrame spotFrame;
        TypeFrame compoundFrame;
        for (unsigned int i = 0; i < n; i++) {
            spotFrame.clear();
            spotFrame = pattern.subFrameAt(argPos.at(i));
            std::vector<TypeFrame> supersetFrames;
            if (DEBUG) spotFrame.printForDebug("spotFrame: ", "\n");
            addSupersetFrames(supersetFrames, spotFrame);
            if (DEBUG) printFrameVector(supersetFrames);
            if (supersetFrames.size() > 0) {
                minP = std::numeric_limits<float>::max();
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

    return std::make_pair(minP, maxP);
}

std::pair<float,float> TypeFrameIndex::minMaxSubsetProb(const TypeFrame &pattern)
{
    float minP = 0;
    float maxP = 0;
    if (DEBUG) printf("frames.size() = %lu\n", frames.size());
    if (DEBUG) printf("floatUniverseCount = %f\n", floatUniverseCount);

    if (DEBUG) printf("minMaxSubsetProb()\n");
    if (pattern.typeAtEqualsTo(0, "AndLink")) {
        if (DEBUG) printf("pattern is AndLink\n");
        unsigned int n = pattern.at(0).second;
        std::vector<int> argPos = pattern.getArgumentsPosition(0);
        std::vector<std::vector<TypeFrame>> subsetVector;
        std::vector<std::vector<TypeFrame>> starVector;
        std::vector<TypeFrame> subsetFrames;
        std::vector<TypeFrame> starFrames;
        std::vector<unsigned int> sizes;
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
            // subsetFrames and starFrames have same size
            sizes.push_back(subsetFrames.size());
        }
        CartesianProductGenerator cartesianGenerator(sizes);
        if (! cartesianGenerator.depleted()) {
            minP = std::numeric_limits<float>::max();
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
            std::pair<float,float> ind = minMaxIndependentProb(compoundStar);
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

    return std::make_pair(minP, maxP);
}

float TypeFrameIndex::computeISurprinsingness(const TypeFrame &pattern, bool normalized)
{
    if (DEBUG) printf("computeISurprinsingness()\n");
    if (DEBUG) pattern.printForDebug("", "\n");

    float answer = 0;
    unsigned int count = countPattern(pattern);
    //float pFull = ((float) count) / frames.size();
    float pFull = ((float) count) / floatUniverseCount;

    if (DEBUG) printf("count (pFull) = %u\n", count);
    if (DEBUG) printf("pFull = %f\n", pFull);

    if (count < MINIMAL_FREQUENCY_TO_COMPUTE_SURPRISINGNESS) {
        return 0;
    }

    std::pair<float,float> pair = minMaxIndependentProb(pattern);

    float c1 = pFull - pair.second;
    float c2 = pair.first - pFull;
    answer = (c1 > c2 ? c1 : c2);

    if (normalized) {
        answer = answer / pFull;
    }

    return answer;
}

float TypeFrameIndex::computeIISurprinsingness(const TypeFrame &pattern, bool normalized)
{
    if (DEBUG) printf("computeIISurprinsingness()\n");
    if (DEBUG) pattern.printForDebug("", "\n");

    float answer = 0;
    unsigned int count = countPattern(pattern);
    float pFull = ((float) count) / floatUniverseCount;

    if (DEBUG) printf("count (pFull) = %u\n", count);
    if (DEBUG) printf("pFull = %f\n", pFull);

    if (count < MINIMAL_FREQUENCY_TO_COMPUTE_SURPRISINGNESS) {
        return 0;
    }

    std::pair<float,float> minMaxInd = minMaxIndependentProb(pattern);
    if (DEBUG) printf("minMaxInd = <%f,%f>\n", minMaxInd.first, minMaxInd.second);

    std::pair<float,float> minMaxSuper = minMaxSupersetProb(pattern);
    if (DEBUG) printf("minMaxSuper = <%f,%f>\n", minMaxSuper.first, minMaxSuper.second);

    std::pair<float,float> minMaxSub = minMaxSubsetProb(pattern);
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

/*
void TypeFrameIndex::minePatterns(std::vector<std::pair<float,TypeFrame>> &answer, unsigned int components, unsigned int maxAnswers, RankingMetric metric)
{
    answer.clear();

    floatUniverseCount = 1;
    unsigned int n = frames.size();
    for (unsigned int i = 0; i < components; i++) {
        floatUniverseCount = floatUniverseCount * ((float) (n - i));
        floatUniverseCount = floatUniverseCount / ((float) (i + 1));
    }
    //floatUniverseCount = (float) frames.size();

    EquivalentTypeFrameSet baseSet;

    if (LOCAL_DEBUG) printf("frames.size(): %lu\n", frames.size());
    for (unsigned int i = 0; i < frames.size(); i++) {
        if (frames.at(i).topLevelIsLink() && (! frames.at(i).typeAtEqualsTo(0, "ListLink"))) {
            baseSet.insert(frames.at(i));
        }
    }

    std::vector<TypeFrame> base;
    for (EquivalentTypeFrameSet::const_iterator it = baseSet.begin(); it != baseSet.end(); it++) {
        base.push_back(*it);
    }
    if (LOCAL_DEBUG) printf("base.size(): %lu\n", base.size());

    CartesianProductGenerator cartesianGenerator(components, base.size(), true, true);
    TypeFrame compoundFrame;
    std::vector<TypeFrame> patterns;
    this->patternCountCache.clear();
    PatternHeap heap;
    unsigned int debugCount = 0;
    while (! cartesianGenerator.depleted()) {
        if (DEBUG) cartesianGenerator.printForDebug("compound selection: ", "\n");
        compoundFrame.clear();
        compoundFrame.push_back(TypePair(classserver().getType("AndLink"), 1));
        compoundFrame.append(base.at(cartesianGenerator.at(components - 1)));
        bool flag = true;
        for (int c = ((int) components) - 2; c >= 0; c--) {
            //if ((compoundFrame.nonEmptyNodeIntersection(base.at(cartesianGenerator.at(c)))) &&
                //(! compoundFrame.contains(base.at(cartesianGenerator.at(c))))) {
            if (compoundFrame.nonEmptyNodeIntersection(base.at(cartesianGenerator.at(c)))) {
                compoundFrame.at(0).second++;
                compoundFrame.append(base.at(cartesianGenerator.at(c)));
            } else {
                cartesianGenerator.drop(c);
                flag = false;
                break;
            }
        }
        // AQUI Criar uma classe para servir como thread pool e processador de
        // compoundFrames. Passar o heap. Passar cada compound para ela
        // enfileirar. As threads vao consumir dessa fila de compounds
        if (flag) {
            if (LOCAL_DEBUG && (!(debugCount++ % 10000))) cartesianGenerator.printForDebug("Evaluating: ", "\n");
            patterns.clear();
            if (DEBUG) compoundFrame.printForDebug("compoundFrame: ", "\n");
            addPatterns(patterns, compoundFrame);
            if (DEBUG) printf("%lu PATTERNS\n", patterns.size());
            if (DEBUG) printFrameVector(patterns);
            for (unsigned int i = 0; i < patterns.size(); i++) {
                if (DEBUG) patterns.at(i).printForDebug("COMPUTE QUALITY FOR: ", "\n");
                float quality = computeQuality(patterns.at(i), metric);
                if (DEBUG) printf("%f: ", quality);
                if (DEBUG) patterns.at(i).printForDebug("", "\n");
                if (((heap.size() < maxAnswers) || (heap.top().first < quality)) && (! heapContainsEquivalent(heap, patterns.at(i))))  {
                    if (DEBUG) printf("Pushing to heap. Quality = %f ", quality);
                    if (DEBUG) patterns.at(i).printForDebug("", "\n");
                    heap.push(std::make_pair(quality, patterns.at(i)));
                    if (heap.size() > maxAnswers) {
                        if (DEBUG) printf("Removing pattern Quality = %f ", heap.top().first);
                        if (DEBUG) heap.top().second.printForDebug("", "\n");
                        heap.pop();
                    }
                }
            }
        }
        cartesianGenerator.generateNext();
    }

    if (DEBUG) printf("Finished mining. heap size = %lu\n", heap.size());

    while (heap.size() > 0) {
        if (DEBUG) printf("%f: ", heap.top().first);
        if (DEBUG) heap.top().second.printForDebug("", "\n");
        answer.push_back(heap.top());
        heap.pop();
    }
}
*/

bool TypeFrameIndex::enqueueCompoundFrame(const TypeFrame &compoundFrame)
{
    std::lock_guard<std::mutex> lock(compoundFrameMutex);
    static unsigned int queueDebugCount = 0;
    if (DEBUG) printf("enqueueCompoundFrame() BEGIN\n");
    compoundFrameQueue.push(compoundFrame);
    if (LOCAL_DEBUG && (!(queueDebugCount++ % 100000))) printf("Queue size: %ld\n",  compoundFrameQueue.size());
    if (DEBUG) printf("enqueueCompoundFrame() END\n");
    return (compoundFrameQueue.size() > MAX_SIZE_OF_COMPOUND_FRAMES_QUEUE);
}

bool TypeFrameIndex::dequeueCompoundFrame(TypeFrame &compoundFrame)
{
    std::lock_guard<std::mutex> lock(compoundFrameMutex);
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
    std::lock_guard<std::mutex> lock(compoundFrameMutex);
    if (DEBUG) printf("setCompoundFramesEnded() BEGIN\n");
    compoundFramesEnded = true;
    if (DEBUG) printf("setCompoundFramesEnded() END\n");
}

bool TypeFrameIndex::checkCompoundPatternsEnded()
{
    std::lock_guard<std::mutex> lock(compoundFrameMutex);
    if (DEBUG) printf("checkCompoundPatternsEnded() BEGIN\n");
    if (DEBUG) printf("checkCompoundPatternsEnded() END\n");
    return compoundFramesEnded;
}

void TypeFrameIndex::addMiningResult(float quality, const TypeFrame &frame)
{
    std::lock_guard<std::mutex> lock(miningResultsMutex);
    if (((miningResultsHeap.size() < maxResultsHeapSize) || (miningResultsHeap.top().first < quality)))  {
        miningResultsHeap.push(std::make_pair(quality, frame));
        if (miningResultsHeap.size() > maxResultsHeapSize) {
            miningResultsHeap.pop();
        }
    }
}

void TypeFrameIndex::evaluatePatterns()
{
    if (DEBUG) printf("evaluatePatterns() BEGIN\n");
    bool finished = false;
    TypeFrame compoundFrame;
    std::vector<TypeFrame> patterns;

    while (! finished) {
        if (dequeueCompoundFrame(compoundFrame)) {
            patterns.clear();
            addPatterns(patterns, compoundFrame);
            for (unsigned int i = 0; i < patterns.size(); i++) {
                float quality = computeQuality(patterns.at(i), miningRankingMetric);
                addMiningResult(quality, patterns.at(i));
            }
        } else {
            if (checkCompoundPatternsEnded()) {
                finished = true;
            } else {
                if (LOCAL_DEBUG) printf("Evaluation thread sleeping\n");
                usleep(100000); // 100 miliseconds
            }
        }
    }
    if (DEBUG) printf("evaluatePatterns() END\n");
}

void TypeFrameIndex::minePatterns(std::vector<std::pair<float,TypeFrame>> &answer, unsigned int components, unsigned int maxAnswers, RankingMetric metric)
{
    answer.clear();

    floatUniverseCount = 1;
    unsigned int n = frames.size();
    for (unsigned int i = 0; i < components; i++) {
        floatUniverseCount = floatUniverseCount * ((float) (n - i));
        floatUniverseCount = floatUniverseCount / ((float) (i + 1));
    }
    //floatUniverseCount = (float) frames.size();

    /*
    TypeFrame testFrame("(AndLink (SimilarityLink (VariableNode \"$var_1\") (ConceptNode \"chimp\")) (SimilarityLink (VariableNode \"$var_1\") (ConceptNode \"ent\")) (SimilarityLink (VariableNode \"$var_1\") (ConceptNode \"monkey\")))");
    testFrame.printForDebug("testFrame: ", "\n");
    float testQuality = computeQuality(testFrame, metric);
    printf("Quality: %f\n", testQuality);
    if (metric < 1000) return;
      
    TypeFrame testFrame2( "(AndLink (InheritanceLink (VariableNode \"V0\") (ConceptNode \"human\")) (InheritanceLink (VariableNode \"V0\") (ConceptNode \"ugly\")) (InheritanceLink (VariableNode \"V0\") (ConceptNode \"soda drinker\")) )");
    testFrame2.printForDebug("testFrame2: ", "\n");
    float testQuality2 = computeQuality(testFrame2, metric);
    printf("Quality2: %f\n", testQuality2);
    if (metric < 1000) return;
    */

    EquivalentTypeFrameSet baseSet;

    if (LOCAL_DEBUG) printf("frames.size(): %lu\n", frames.size());
    for (unsigned int i = 0; i < frames.size(); i++) {
        if (frames.at(i).topLevelIsLink() && (! frames.at(i).typeAtEqualsTo(0, "ListLink"))) {
            baseSet.insert(frames.at(i));
        }
    }

    std::vector<TypeFrame> base;
    for (EquivalentTypeFrameSet::const_iterator it = baseSet.begin(); it != baseSet.end(); it++) {
        base.push_back(*it);
    }
    if (LOCAL_DEBUG) printf("base.size(): %lu\n", base.size());

    CartesianProductGenerator cartesianGenerator(components, base.size(), true, true);
    TypeFrame compoundFrame;
    this->patternCountCache.clear();
    //miningResultsHeap.clear();
    maxResultsHeapSize = maxAnswers;
    miningRankingMetric = metric;
    compoundFramesEnded = false;
    evaluationThreads.clear();
    for (unsigned int i = 0; i < (NUMBER_OF_EVALUATION_THREADS - 1); i++) {
        evaluationThreads.push_back(new std::thread(&TypeFrameIndex::evaluatePatterns, this));
    }
    unsigned int debugCount = 0;
    while (! cartesianGenerator.depleted()) {
        if (DEBUG) cartesianGenerator.printForDebug("compound selection: ", "\n");
        compoundFrame.clear();
        compoundFrame.push_back(TypePair(classserver().getType("AndLink"), 1));
        compoundFrame.append(base.at(cartesianGenerator.at(components - 1)));
        bool flag = true;
        for (int c = ((int) components) - 2; c >= 0; c--) {
            if (compoundFrame.nonEmptyNodeIntersection(base.at(cartesianGenerator.at(c)))) {
                compoundFrame.at(0).second++;
                compoundFrame.append(base.at(cartesianGenerator.at(c)));
            } else {
                cartesianGenerator.drop(c);
                flag = false;
                break;
            }
        }
        if (flag) {
             if (LOCAL_DEBUG && (!(debugCount++ % 100000))) cartesianGenerator.printForDebug("Evaluating: ", "\n");
             if (enqueueCompoundFrame(compoundFrame)) {
                 usleep(1000000); // 1 second
             }
        }
        cartesianGenerator.generateNext();
    }
    setCompoundFramesEnded();
    if (LOCAL_DEBUG) printf("Finished creating compound patterns. Waiting for threads to clear evaluation queue.\n");
    evaluationThreads.push_back(new std::thread(&TypeFrameIndex::evaluatePatterns, this));

    for (unsigned int i = 0; i < evaluationThreads.size(); i++) {
        evaluationThreads.at(i)->join();
    }

    if (DEBUG) printf("Finished mining. heap size = %lu\n", miningResultsHeap.size());

    while (miningResultsHeap.size() > 0) {
        if (DEBUG) printf("%f: ", miningResultsHeap.top().first);
        if (DEBUG) miningResultsHeap.top().second.printForDebug("", "\n");
        answer.push_back(miningResultsHeap.top());
        miningResultsHeap.pop();
    }
}

void TypeFrameIndex::permutation(std::vector<std::vector<int>> &answer, int *array, int current, int size)
{
    if (current == size - 1) {
        std::vector<int> v;
        for (int i = 0; i < size; i++) {
            v.push_back(array[i]);
        }
        answer.push_back(v);
    } else {
        for (int i = current; i < size; i++) {
            int aux = array[current];
            array[current] = array[i];
            array[i] = aux;
            permutation(answer, array, current + 1, size);
            aux = array[current];
            array[current] = array[i];
            array[i] = aux;
        }
    }
}

void TypeFrameIndex::addPermutations(std::vector<std::vector<int>> &answer, std::vector<int> base)
{
    int array[base.size()];
    for (unsigned int i = 0; i < base.size(); i++) {
        array[i] = base.at(i);
    }
    permutation(answer, array, 0, base.size());
}

void TypeFrameIndex::addSymmetricPermutations(TypeFrameSet &answer, const TypeFrame &frame, unsigned int cursor)
{
    unsigned int arity = frame.at(cursor).second;
    std::vector<int> argPos = frame.getArgumentsPosition(cursor);
    if (frame.typeAtIsSymmetricLink(cursor)) {
        if (arity == 2) {
            // optimization of commom case
            answer.insert(frame.subFrameAt(cursor));
            TypeFrame permutation;
            for (unsigned int i = 0; i <= cursor; i++) {
                permutation.pickAndPushBack(frame, i);
            }
            TypeFrame subFrame0 = frame.subFrameAt(argPos.at(0));
            TypeFrame subFrame1 = frame.subFrameAt(argPos.at(1));
            permutation.append(subFrame1);
            permutation.append(subFrame0);
            answer.insert(permutation);
        } else {
            if (arity <= LIMIT_FOR_SYMMETRIC_LINKS_PERMUTATION) {
                std::vector<std::vector<int>> permutationVector;
                addPermutations(permutationVector, argPos);
                TypeFrame permutation;
                for (unsigned int k = 0; k < permutationVector.size(); k++) {
                    permutation.clear();
                    for (unsigned int i = 0; i <= cursor; i++) {
                        permutation.pickAndPushBack(frame, i);
                    }
                    TypeFrame subFrame;
                    for (unsigned int j = 0; j < arity; j++) {
                        subFrame.clear();
                        subFrame = frame.subFrameAt(permutationVector.at(k).at(j));
                        permutation.append(subFrame);
                    }
                    answer.insert(permutation);
                }
            }
        }
    } else {
        answer.insert(frame.subFrameAt(cursor));
    }
    for (unsigned int i = 0; i < arity; i++) {
        addSymmetrucPermutations(answer, frame, argPos.at(i));
    }
}

bool TypeFrameIndex::addFrame(TypeFrame &frame, int offset)
{
    bool exitStatus = true;
    if (frame.isValid()) {
        TypeFrameSet symmetricPermutations;
        addSymmetrucPermutations(symmetricPermutations, frame, 0);
        for (TypeFrameSet::iterator it = symmetricPermutations.begin(); it != symmetricPermutations.end(); it++) {
            frames.push_back(*it);
        }
        if (DEBUG) {
            printf("%d: ", offset);
            frame.printForDebug("", "\n", true);
        }
        exitStatus = false;
    } else {
        printf("DISCARDING INVALID FRAME (offset = %d)\n", offset);
    }

    return exitStatus;
}

bool TypeFrameIndex::addFromScheme(const std::string &txt, int offset)
{
    bool exitStatus = true;
    TypeFrame frame(txt);
    if (frame.isValid()) {
        exitStatus = addFrame(frame, offset);
    } else {
        printf("INVALID FRAME <%s>\n", txt.c_str());
    }
    return exitStatus;
}

void TypeFrameIndex::addArity2Patterns(std::vector<TypeFrame> &answer, std::vector<TypeFrame> &recurseResult1, std::vector<TypeFrame> &recurseResult2, TypeFrame &baseFrame, int cursor)
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

// TODO: This method should allocate answers in the heap instead of the stack to
// avoid copying data all the away in the recursive calls
// TODO: Break this method into smaller pieces
std::vector<TypeFrame> TypeFrameIndex::computeSubPatterns(TypeFrame &baseFrame, int cursor, int pos)
{
    std::vector<TypeFrame> answer;
    unsigned int headArity = baseFrame.at(cursor).second;
    bool symmetricHead = baseFrame.typeAtIsSymmetricLink(cursor);
    std::vector<int> argPos = baseFrame.getArgumentsPosition(cursor);

    if (headArity == 0) {

        // Node

        TypeFrame pattern1, pattern2;
        pattern1.push_back(baseFrame.at(cursor));    // discards node name
        pattern2.pickAndPushBack(baseFrame, cursor); // uses node name

        if (DEBUG) {
            printf("Adding node pattern\n");
            printf("Name: %s\n", baseFrame.nodeNameAt(cursor).c_str());
            pattern1.printForDebug("pattern1: ", "\n", true);
            pattern2.printForDebug("pattern2: ", "\n", true);
        }

        answer.push_back(pattern1); 
        answer.push_back(pattern2);

    } else if (headArity == 2) {

        // Arity 2 Link, computes all possible patterns

        std::vector<TypeFrame> recurseResult1, recurseResult2;

        recurseResult1 = computeSubPatterns(baseFrame, argPos.at(0), pos);
        recurseResult2 = computeSubPatterns(baseFrame, argPos.at(1), pos);
        if (DEBUG) {
            for (unsigned int i = 0; i < recurseResult1.size(); i++) {
                recurseResult1.at(i).printForDebug("recurseResult1: ", "\n", true);
            }
            for (unsigned int i = 0; i < recurseResult2.size(); i++) {
                recurseResult1.at(i).printForDebug("recurseResult2: ", "\n", true);
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

        std::vector<TypeFrame> recurseResult[headArity];
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
    }

    if (! TOPLEVEL_ONLY) {
        for (unsigned int i = 0; i < answer.size(); i++) {
            addPatternOccurrence(answer.at(i), pos);
        }
    }
    return answer;
}

void TypeFrameIndex::addPatternOccurrence(TypeFrame &pattern, int pos)
{
    occurrenceUniverse.insert(pos);
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
    for (unsigned int i = 0; i < frames.size(); i++) {
        TypeFrame currentFrame = frames.at(i);
        std::vector<TypeFrame> patterns = computeSubPatterns(currentFrame, 0, i);
        if (TOPLEVEL_ONLY) {
            addPatternOccurrence(currentFrame, i);
            for (unsigned int j = 0; j < patterns.size(); j++) {
                addPatternOccurrence(patterns.at(j), i);
            }
        }
    }
    if (DEBUG) printForDebug(true);
}

void TypeFrameIndex::query(std::vector<ResultPair> &result, const std::string &queryScm, bool distinct, bool noPermutations) const
{
    TypeFrame queryFrame(queryScm);
    if (DEBUG) queryFrame.printForDebug("NEW QUERY: ", "\n", true);
    query(result, queryFrame, distinct, noPermutations);
}

void TypeFrameIndex::selectCurrentElement(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor) const
{
    if (baseFrame.typeAtEqualsTo(cursor, "VariableNode")) {
        answer.push_back(TypeFrame::STAR_PATTERN);
        std::string key = baseFrame.nodeNameAt(cursor);
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


void TypeFrameIndex::buildConstraints(IntPairVector &constraints, StringMap &variableOccurrences) const
{
    constraints.clear();
    StringMap::iterator it = variableOccurrences.begin();
    while (it != variableOccurrences.end()) {
        std::vector<int> v((*it).second.begin(), (*it).second.end());
        for (unsigned int i = 0; i < v.size(); i++) {
            for (unsigned int j = i + 1; j < v.size(); j++) {
                constraints.push_back(std::make_pair(v.at(i), v.at(j)));
            }
        }
        it++;
    }
}

void TypeFrameIndex::buildQueryTerm(TypeFrame &answer, StringMap &variableOccurrences, const TypeFrame &baseFrame, int cursor) const
{
    selectCurrentElement(answer, variableOccurrences, baseFrame, cursor);
    int nargs = baseFrame.at(cursor).second;
    while (nargs > 0) {
        cursor++;
        selectCurrentElement(answer, variableOccurrences, baseFrame, cursor);
        nargs--;
        if (nargs > 0) {
            int skip = baseFrame.at(cursor).second;
            while (skip > 0) {
                cursor++;
                selectCurrentElement(answer, variableOccurrences, baseFrame, cursor);
                skip += baseFrame.at(cursor).second;
                skip--;
            }
        }
    }
}

void TypeFrameIndex::query(std::vector<ResultPair> &result, const TypeFrame &queryFrame, bool distinct, bool noPermutations) const
{
    TypeFrame keyExpression;
    std::vector<VarMapping> forbiddenMappings;
    int headLogicOperator;

    query(result, keyExpression, forbiddenMappings, headLogicOperator, queryFrame, 0, distinct, noPermutations);
}

bool TypeFrameIndex::compatibleVarMappings(const VarMapping &map1, const VarMapping &map2, bool distinct) const
{
    /*
    if (DEBUG) {
        printf("TypeFrameIndex::compatibleVarMappings()\n");
        printVarMapping(map1);
        printf("\n");
        printVarMapping(map2);
    }
    */

    for (VarMapping::const_iterator it1 = map1.begin(); it1 != map1.end(); it1++) {
        VarMapping::const_iterator it2 = map2.find((*it1).first);
        if ((it2 != map2.end()) && (! (*it1).second.equals((*it2).second))) {
            /*
            if (DEBUG) {
                printf("Failed at %s\n", (*it1).first.c_str());
            }
            */
            return false;
        }
    }
    if (distinct) {
        for (VarMapping::const_iterator it1 = map1.begin(); it1 != map1.end(); it1++) {
            for (VarMapping::const_iterator it2 = map2.begin(); it2 != map2.end(); it2++) {
                if (((*it1).first.compare((*it2).first) != 0) && ((*it1).second.equals((*it2).second))) {
                    return false;
                }
            }
        }
    }

    return true;
}

void TypeFrameIndex::typeFrameSetUnion(TypeFrameSet &answer, const TypeFrameSet &set1, const TypeFrameSet &set2) const
{
    answer.clear();
    for (TypeFrameSet::const_iterator it1 = set1.begin(); it1 != set1.end(); it1++) {
        answer.insert(*it1);
    }
    for (TypeFrameSet::const_iterator it2 = set2.begin(); it2 != set2.end(); it2++) {
        answer.insert(*it2);
    }
}

void TypeFrameIndex::varMappingUnion(VarMapping &answer, const VarMapping &map1, const VarMapping &map2) const
{
    for (VarMapping::const_iterator it = map1.begin(); it != map1.end(); it++) {
        answer.insert(*it);
    }
    for (VarMapping::const_iterator it = map2.begin(); it != map2.end(); it++) {
        answer.insert(*it);
    }
}

bool TypeFrameIndex::isForbiddenMapping(const VarMapping &mapping, const std::vector<VarMapping> &forbiddenVector) const
{
    bool answer = false;
    for (unsigned int i = 0; i < forbiddenVector.size(); i++) {
        bool match = true;
        for (VarMapping::const_iterator it1 = forbiddenVector.at(i).begin(); it1 != forbiddenVector.at(i).end(); it1++) {
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

void TypeFrameIndex::query(std::vector<ResultPair> &answer, TypeFrame &keyExpression, std::vector<VarMapping> &forbiddenMappings, int &logicOperator, const TypeFrame &queryFrame, int cursor, bool distinct, bool noPermutations) const
{
    if (DEBUG) queryFrame.printForDebug("Query frame: ", "\n", true);
    if (DEBUG) printf("Cursor: %d\n", cursor);
    std::vector<ResultPair> unfilteredAnswer;
    answer.clear();
    keyExpression.clear();
    forbiddenMappings.clear();
    unsigned int arity = queryFrame.at(cursor).second;
    std::vector<int> argPos = queryFrame.getArgumentsPosition(cursor);
    std::vector<std::vector<ResultPair>> recursionQueryResult(arity);
    std::vector<TypeFrame> recursionKeyExpression(arity);
    std::vector<std::vector<VarMapping>> recursionForbiddenMappings(arity);
    std::vector<int> recursionHeadLogicOperator(arity);
    bool AndFlag = queryFrame.typeAtEqualsTo(cursor, "AndLink");
    bool OrFlag = queryFrame.typeAtEqualsTo(cursor, "OrLink");
    bool NotFlag = queryFrame.typeAtEqualsTo(cursor, "NotLink");
    if (DEBUG) printf("Head is %s\n", (AndFlag ? "AND" : (OrFlag ? "OR" : (NotFlag ? "NOT" : "LEAF"))));

    if (AndFlag || OrFlag || NotFlag) {
        // Recursive call on each clause
        keyExpression.pickAndPushBack(queryFrame, cursor);
        for (unsigned int i = 0; i < arity; i++) {
            query(recursionQueryResult.at(i), recursionKeyExpression.at(i), recursionForbiddenMappings.at(i), recursionHeadLogicOperator.at(i), queryFrame, argPos.at(i), distinct, noPermutations);
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
        std::vector<std::vector<ResultPair>> cleanRecursionQueryResult(arity);
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
        std::vector<ResultPair> aux[2];
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
                            aux[tgt].push_back(std::make_pair(newSet, newMapping));
                        } else {
                            if (DEBUG) {
                                printf("(AND) rejecting non-compatible var maps:\n");
                                printVarMapping(aux[src].at(a).second);
                                printf("-\n");
                                printVarMapping(cleanRecursionQueryResult.at(i).at(b).second);
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
            for (StringMap::iterator it1 = variableOccurrences.begin(); it1 != variableOccurrences.end(); it1++) {
                printf("%s: ", (*it1).first.c_str());
                for (IntegerSet::iterator it2 = (*it1).second.begin(); it2 != (*it1).second.end(); it2++) {
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
            for (IntegerSet::iterator it2 = (*it1).second.begin(); it2 != (*it1).second.end(); it2++) {
                std::vector<int> mapping;
                if (frames.at(*it2).match(mapping, keyExpression, constraints)) {
                    VarMapping varMap;
                    TypeFrameSet frameSet;
                    for (StringMap::iterator it = variableOccurrences.begin(); it != variableOccurrences.end(); it++) {
                        varMap.insert(VarMapping::value_type((*it).first, frames.at(*it2).subFrameAt(*((*it).second.begin()))));
                    }
                    frameSet.insert(frames.at(*it2));
                    if (DEBUG) {
                        printf("(LEAF) Adding solution to result:\n");
                        printTypeFrameSet(frameSet);
                        printVarMapping(varMap);
                    }
                    if (noPermutations) {
                        unfilteredAnswer.push_back(std::make_pair(frameSet, varMap));
                    } else {
                        answer.push_back(std::make_pair(frameSet, varMap));
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
    for (VarMapping::const_iterator it = map1.begin(); it != map1.end(); it++) {
        set.insert((*it).second);
    }
    bool answer = true;
    for (VarMapping::const_iterator it = map2.begin(); it != map2.end(); it++) {
        if (set.find((*it).second) == set.end()) {
            answer = false;
            break;
        }
    }

    return answer;
}

bool TypeFrameIndex::equivalentVarMappings(const VarMapping &map1, const VarMapping &map2, bool distinct) const
{
    return (distinct ? mapCover(map1, map2) : (mapCover(map1, map2) && mapCover(map2, map1)));
}

void TypeFrameIndex::printFrameVector(const std::vector<TypeFrame> &v) const
{
    for (unsigned int i = 0; i < v.size(); i++) {
        printf("v[%u] = ", i);
        v.at(i).printForDebug("", "\n");
    }
}

void TypeFrameIndex::printVarMapping(const VarMapping &map) const
{
    for (VarMapping::const_iterator it = map.begin(); it != map.end(); it++) {
        printf("%s = ", (*it).first.c_str());
        (*it).second.printForDebug("", "\n", true);
    }
}

void TypeFrameIndex::printTypeFrameSet(const TypeFrameSet &set) const
{
    for (TypeFrameSet::const_iterator it = set.begin(); it != set.end(); it++) {
        (*it).printForDebug("", "\n", true);
    }
}

void TypeFrameIndex::printRecursionResult(const std::vector<ResultPair> &v) const
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
            it2++;
        }
        printf("\n");
        it1++;
    }
}

