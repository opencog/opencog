#include "PatternIndexAPI.h"

#include <algorithm> 

#include <opencog/guile/SchemePrimitive.h>
#include <opencog/util/Config.h>
#include "SCMLoader.h"
#include "TypeFrameIndexBuilder.h"

using namespace opencog;

PatternIndexAPI::PatternIndexAPI()
{
    atomSpace = SchemeSmob::ss_get_env_as("PatternIndex");
    schemeEval = new SchemeEval(atomSpace);
    lastUsedTicket = 0;
}

PatternIndexAPI::~PatternIndexAPI()
{
    for (IndexMap::iterator it = indexes.begin(); it != indexes.end(); it++) {
        delete (*it).second.first;
    }
}

std::string PatternIndexAPI::getStringProperty(StringMap &map, const std::string key)
{
    StringMap::iterator it = map.find(key);
    if (it == map.end()) {
        throw std::runtime_error("Invalid property key: " + key);
    } else {
        return (*it).second;
    }
}

int PatternIndexAPI::getIntProperty(StringMap &map, const std::string key, int min, int max)
{
    int answer;

    StringMap::iterator it = map.find(key);
    if (it == map.end()) {
        throw std::runtime_error("Invalid property key: " + key);
    } else {
        answer = std::stoi((*it).second);
        if ((answer < min) || (answer > max)) {
            throw std::runtime_error("Invalid value for " + key + ": " + std::to_string(answer));
        }
    }

    return answer;
}

bool PatternIndexAPI::getBoolProperty(StringMap &map, const std::string key)
{
    bool answer;

    StringMap::iterator it = map.find(key);
    if (it == map.end()) {
        throw std::runtime_error("Invalid property key: " + key);
    } else {
        std::string value = (*it).second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        if (value.compare("true") == 0) {
            answer = true;
        } else if (value.compare("false") == 0) {
            answer = false;
        } else {
            throw std::runtime_error("Invalid value for " + key + ": " + (*it).second);
        }
    }

    return answer;
}

// Default properties are set using the config file
void PatternIndexAPI::setDefaultProperty(StringMap &map, const std::string key)
{
    std::string value = config().get("PatternIndex::" + key);
    map.insert(StringMap::value_type(key, value));
}

void PatternIndexAPI::setDefaultProperties(StringMap &properties)
{
    // -----------------------------------------------------------------------
    // Relevant for querying and mining

    // Enables the use of a cache to count pattern occurrences.
    // If this flag is set "true", queries and mining tend to be faster
    // at the cost of memory.
    //
    // Tipically, you will want to set this "true" for small/medium datasets
    // and "false" for large (> 5K/10K links) ones. The actual limits will (of
    // course) depend on the dataset and the amount of available memory.
    // 
    // If the number of evaluation threads (see below) is set to n > 1
    // then this parameter is bypassed and the cache is automatically disabled.
    setDefaultProperty(properties, "PatternCountCacheEnabled");

    // -----------------------------------------------------------------------
    // Relevant for querying

    // In queries with more than one VariableNode, setting this parameter "true"
    // will ensure that different VariableNodes are assigned to different Atoms.
    //
    // Example:
    //
    // Suppose we have the following knowledge base
    //
    // (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
    // (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
    // (SimilarityLink (ConceptNode "chimp") (ConceptNode "monkey"))
    //
    // Consider the following query
    //
    // (AndLink 
    //     (SimilarityLink (VariableNode V1) (VariableNode V2))
    //     (SimilarityLink (VariableNode V2) (VariableNode V3))
    // )
    //
    // If "DifferentAssignmentForDifferentVars" == "true" the following result
    // will NOT be returned
    //
    // V1 = (ConceptNode "monkey")
    // V2 = (ConceptNode "human")
    // V3 = (ConceptNode "monkey")
    //
    // otherwise, the above result may be returned (depending on the other
    // settings).
    //
    setDefaultProperty(properties, "DifferentAssignmentForDifferentVars");

    // In queries with more than one VariableNode, if this parameter is set
    // "true" then permutation of variable assignments are considered
    // equivalent (thus they are not returned as query result).
    //
    // Example:
    //
    // Suppose we have the following knowledge base
    //
    // (SimilarityLink (ConceptNode "human") (ConceptNode "monkey"))
    // (SimilarityLink (ConceptNode "human") (ConceptNode "chimp"))
    // (SimilarityLink (ConceptNode "chimp") (ConceptNode "monkey"))
    //
    // Consider the following query
    //
    // (AndLink 
    //     (SimilarityLink (VariableNode V1) (VariableNode V2))
    //     (SimilarityLink (VariableNode V2) (VariableNode V3))
    // )
    //
    // If "PermutationsOfVarsConsideredEquivalent" == "false" all the following results
    // will be returned
    //
    // V1 = (ConceptNode "monkey")
    // V2 = (ConceptNode "human")
    // V3 = (ConceptNode "chimp")
    //
    // V1 = (ConceptNode "monkey")
    // V2 = (ConceptNode "chimp")
    // V3 = (ConceptNode "human")
    //
    // V1 = (ConceptNode "human")
    // V2 = (ConceptNode "monkey")
    // V3 = (ConceptNode "chimp")
    //
    // ... Etc ...
    //
    // setting this parameter "true" will avoid this permutation in query result.
    //
    setDefaultProperty(properties, "PermutationsOfVarsConsideredEquivalent");

    // -----------------------------------------------------------------------
    // Relevant for mining

    // Number of auxiliary threads used to compute patterns quality. 
    // Need to be >= 1.
    // Usually this is set to the number of availables processors. There is no
    // point in using a larger value.
    setDefaultProperty(properties, "NumberOfEvaluationThreads");

    // Used to mine patterns. Patterns that have a total count below this
    // threashold are discarded without being evaluated.
    //
    // There is no default that makes sense for all applications. So you may
    // want to play with this parameter to see how good (or bad) are the results
    // of the mining algorithm.
    //
    // This parameter have a SEVERE effect in time performance of the mining
    // algorithm. Larger values will make a lot of patterns to be discarded
    // before evaluation of the quality metric (surprinsingness, etc) so the
    // algorithm executes faster (the drawback is the possibility of discarding
    // a good - i.e high quality measure - pattern).
    setDefaultProperty(properties, "MinimalFrequencyToComputeQualityMetric");

    // The size of an auxiliary queue used in the mining algorithm.
    // This parameter is here because it will determine the amount of memory
    // used by the mining algorithm.
    //
    // Again, there is no default that makes sense for every application.
    //
    // In general, the mining algorithm will run slightly faster with larger
    // values. 
    setDefaultProperty(properties, "MaximumSizeOfCompoundFramesQueue");

    // Only patterns of specified GRAM will be evaluated.
    // (see the PatternMiner documentation in Wiki to understand the meaning of a pattern's GRAM)
    //
    // This parameter have a HUGE effect in time performance and pattern overall
    // "quality". Usually "PatternsGram" < 3 will lead to useless patterns.
    // For large knowledge bases, searching for > 3 gram patterns is not viable.
    setDefaultProperty(properties, "PatternsGram");

    // The pattern searching algorithm will always keep the best
    // "MaximumNumberOfMiningResults" patterns found so far. When the search
    // ends, these patterns are returned as the mining results.
    //
    // This parameter have a small effect in memory usage and execution time.
    // Internally, the algorithm keeps a heap of results. Thus although using
    // large (> 1000000) values here are possible, this may slow down the
    // search.
    setDefaultProperty(properties, "MaximumNumberOfMiningResults");

    // Patterns are evaluated according to this metric.
    // (see the PatternMiner documentation in Wiki to understand the metric
    // algorithms)
    //
    // Available metrics are:
    //
    // - isurprisingness
    // - nisurprisingness
    // - iisurprisingness
    // - niisurprisingness
    //
    // IISurprisingness is a lot more time expensive than ISurprisingness but
    // usually will lead to better (higher pattern surprisingness) results IFF the knowledge base have a
    // sensible set of InheritanceLinks. 
    //
    // Normalized versions have the same (time) cost of the non-normalized
    // counterpart.
    setDefaultProperty(properties, "PatternRankingMetric");

    // The function used to compute coherence(x: atom). It is used by the mining
    // algorithm to compute surprinsigness (a metric for a pattern's quality).
    //
    // Actually (currently) there's no alternative to the default "const1" which is
    // coherence(x) = 1 for all x.
    setDefaultProperty(properties, "CoherenceFunction");

    // This is the function g(x) mentioned in the document that describes how to
    // compute surprisingness of a pattern. It is used by the mining
    // algorithm to compute surprinsigness (a metric for a pattern's quality).
    //
    // Actually (currently) there's no alternative to the default
    // "oneOverCoherence" which is g(x) = 1 / coherence(x).
    setDefaultProperty(properties, "CoherenceModulatorG");

    // This is the function h(x) mentioned in the document that describes how to
    // compute surprisingness of a pattern. It is used by the mining
    // algorithm to compute surprinsigness (a metric for a pattern's quality).
    //
    // Actually (currently) there's no alternative to the default
    // "coherence" which is h(x) = coherence(x).
    setDefaultProperty(properties, "CoherenceModulatorH");
}

Handle PatternIndexAPI::createIndex(const std::string &scmPath)
{

    int ticket = -1;

    TypeFrameIndex *index = new TypeFrameIndex();
    TypeFrameIndexBuilder builder(index);

    if (SCMLoader::load(scmPath, *atomSpace, &builder)) {
        throw std::runtime_error("Error creating PatternIndex. SCM file is invalid.\n");
    } else {
        index->buildSubPatternsIndex();
        ticket = ++lastUsedTicket;
        StringMap properties;
        setDefaultProperties(properties);
        indexes.insert(IndexMap::value_type(ticket, std::make_pair(index, properties)));
    }

    return atomSpace->add_node(ANCHOR_NODE, std::to_string(ticket));
}

Handle PatternIndexAPI::createIndex(const HandleSeq &handles)
{

    int ticket = -1;

    TypeFrameIndex *index = new TypeFrameIndex();

    for (HandleSeq::const_iterator it = handles.begin(); it != handles.end(); it++) {
        index->add(*it, 0);
    }

    index->buildSubPatternsIndex();
    ticket = ++lastUsedTicket;
    StringMap properties;
    setDefaultProperties(properties);
    indexes.insert(IndexMap::value_type(ticket, std::make_pair(index, properties)));

    return atomSpace->add_node(ANCHOR_NODE, std::to_string(ticket));
}

void PatternIndexAPI::deleteIndex(Handle key)
{
    IndexMap::iterator it = indexes.find(std::stoi(key->getName()));
    if (it == indexes.end()) {
        throw std::runtime_error("Invalid index key: " + key->toString());
    }

    delete (*it).second.first;
    indexes.erase(it);
}

void PatternIndexAPI::applyProperties(Handle key)
{
    IndexMap::iterator it = indexes.find(std::stoi(key->getName()));
    if (it == indexes.end()) {
        throw std::runtime_error("Invalid index key: " + key->toString());
    }

    TypeFrameIndex *index = (*it).second.first;

    index->PATTERN_COUNT_CACHE_ENABLED = getBoolProperty((*it).second.second, "PatternCountCacheEnabled");
    int n = getIntProperty((*it).second.second, "NumberOfEvaluationThreads", 1);
    index->NUMBER_OF_EVALUATION_THREADS = n;
    if (n > 1) {
        index->PATTERN_COUNT_CACHE_ENABLED = false;
    }
    index->MINIMAL_FREQUENCY_TO_COMPUTE_QUALITY_METRIC = getIntProperty((*it).second.second, "MinimalFrequencyToComputeQualityMetric", 1);
    index->MAX_SIZE_OF_COMPOUND_FRAMES_QUEUE = getIntProperty((*it).second.second, "MaximumSizeOfCompoundFramesQueue", 1);

    std::string s = getStringProperty((*it).second.second, "CoherenceFunction");
    if (s.compare("const1") == 0) {
        index->COHERENCE_FUNCTION = TypeFrameIndex::CONST_1;
    } else {
        throw std::runtime_error("Invalid value for CoherenceFunction: " + s);
    }

    s = getStringProperty((*it).second.second, "CoherenceModulatorG");
    if (s.compare("oneOverCoherence") == 0) {
        index->COHERENCE_MODULATOR_G = TypeFrameIndex::ONE_OVER_COHERENCE;
    } else {
        throw std::runtime_error("Invalid value for CoherenceModulatorG: " + s);
    }

    s = getStringProperty((*it).second.second, "CoherenceModulatorH");
    if (s.compare("coherence") == 0) {
        index->COHERENCE_MODULATOR_H = TypeFrameIndex::COHERENCE;
    } else {
        throw std::runtime_error("Invalid value for CoherenceModulatorH: " + s);
    }

    index->DIFFERENT_ASSIGNMENT_FOR_DIFFERENT_VARS = getBoolProperty((*it).second.second, "DifferentAssignmentForDifferentVars");
    index->PERMUTATIONS_OF_VARS_CONSIDERED_EQUIVALENT = getBoolProperty((*it).second.second, "PermutationsOfVarsConsideredEquivalent");
    index->PATTERNS_GRAM = getIntProperty((*it).second.second, "PatternsGram", 1);
    index->MAXIMUM_NUMBER_OF_MINING_RESULTS = getIntProperty((*it).second.second, "MaximumNumberOfMiningResults", 1);

    s = getStringProperty((*it).second.second, "PatternRankingMetric");
    if (s.compare("isurprisingness") == 0) {
        index->PATTERN_RANKING_METRIC = TypeFrameIndex::I_SURPRISINGNESS;
    } else if (s.compare("nisurprisingness") == 0) {
        index->PATTERN_RANKING_METRIC = TypeFrameIndex::N_I_SURPRISINGNESS;
    } else if (s.compare("iisurprisingness") == 0) {
        index->PATTERN_RANKING_METRIC = TypeFrameIndex::II_SURPRISINGNESS;
    } else if (s.compare("niisurprisingness") == 0) {
        index->PATTERN_RANKING_METRIC = TypeFrameIndex::N_II_SURPRISINGNESS;
    } else {
        throw std::runtime_error("Invalid value for PatternRankingMetric: " + s);
    }
}

void PatternIndexAPI::setProperty(Handle key, const std::string &propertyName, const std::string &propertyValue)
{
    IndexMap::iterator it1 = indexes.find(std::stoi(key->getName()));
    if (it1 == indexes.end()) {
        throw std::runtime_error("Invalid index key: " + key->toString());
    }
    StringMap::iterator it2 = (*it1).second.second.find(propertyName);
    if (it2 == (*it1).second.second.end()) {
        throw std::runtime_error("Invalid property name: " + propertyName);
    } else {
        (*it2).second = propertyValue;
    }
}

void PatternIndexAPI::query(std::vector<QueryResult> &answer, Handle key, const TypeFrame &query)
{
    applyProperties(key);
    std::vector<TypeFrameIndex::ResultPair> queryResult;
    indexes.find(std::stoi(key->getName()))->second.first->query(queryResult, query);
    for (unsigned int i = 0; i < queryResult.size(); i++) {
        HandleSeq atoms;
        VariableMapping mapping;
        for (TypeFrameIndex::TypeFrameSet::const_iterator it = queryResult.at(i).first.begin(); it != queryResult.at(i).first.end(); it++) {
            atoms.push_back(schemeEval->eval_h((*it).toSCMString()));
        }
        for (TypeFrameIndex::VarMapping::const_iterator it = queryResult.at(i).second.begin(); it != queryResult.at(i).second.end(); it++) {
            Handle var = schemeEval->eval_h("(VariableNode \"" + (*it).first + "\")");
            Handle value = schemeEval->eval_h((*it).second.toSCMString());
            mapping.push_back(std::make_pair(var, value));
        }
        answer.push_back(std::make_pair(atoms, mapping));
    }
}

void PatternIndexAPI::query(std::vector<QueryResult> &answer, Handle key, const std::string &queryStr)
{
    TypeFrame queryFrame(queryStr);
    query(answer, key, queryFrame);
}

Handle PatternIndexAPI::query(Handle key, Handle queryLink)
{
    TypeFrame queryFrame(queryLink);

    std::vector<QueryResult> queryResult;
    query(queryResult, key, queryFrame);
    HandleSeq resultVector;
    HandleSeq variableMapping;
    for (unsigned int i = 0; i < queryResult.size(); i++) {
        variableMapping.clear();
        for (unsigned int j = 0; j < queryResult.at(i).second.size(); j++) {
            variableMapping.push_back(atomSpace->add_link(LIST_LINK, queryResult.at(i).second.at(j).first, queryResult.at(i).second.at(j).second));
        }
        Handle satisfyingSubgraph = atomSpace->add_link(LIST_LINK, queryResult.at(i).first);
        Handle varMapping = atomSpace->add_link(LIST_LINK, variableMapping);
        resultVector.push_back(atomSpace->add_link(LIST_LINK, satisfyingSubgraph, varMapping));
    }

    return atomSpace->add_link(LIST_LINK, resultVector);
}

void PatternIndexAPI::minePatterns(std::vector<MiningResult> &answer, Handle key)
{
    applyProperties(key);

    std::vector<std::pair<float,TypeFrame>> patterns;
    indexes.find(std::stoi(key->getName()))->second.first->minePatterns(patterns);
    for (unsigned int i = 0; i < patterns.size(); i++) {
        answer.push_back(std::make_pair(patterns.at(i).first, schemeEval->eval_h(patterns.at(i).second.toSCMString())));
    }
}

Handle PatternIndexAPI::minePatterns(Handle key)
{
    std::vector<MiningResult> miningResults;
    minePatterns(miningResults, key);
    HandleSeq resultVector;
    for (unsigned int i = 0; i < miningResults.size(); i++) {
        Handle quality = atomSpace->add_node(NUMBER_NODE, std::to_string(miningResults.at(i).first));
        resultVector.push_back(atomSpace->add_link(LIST_LINK, quality, miningResults.at(i).second));
    }

    return atomSpace->add_link(LIST_LINK, resultVector);
}

// Compliance with OpenCog's style of singleton access.
// using patternindex() or PatternIndexAPI::getInstance() is equivalent
PatternIndexAPI &opencog::patternindex()
{
    return PatternIndexAPI::getInstance();
}

