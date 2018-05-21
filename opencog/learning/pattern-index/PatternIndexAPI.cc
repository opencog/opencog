#include "PatternIndexAPI.h"

#include <algorithm> 
#include <sstream>
#include <iostream>

#include <opencog/atoms/proto/NameServer.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/util/Config.h>
#include "SCMLoader.h"
#include "TypeFrameIndexBuilder.h"

using namespace opencog;
using namespace std;

PatternIndexAPI::PatternIndexAPI()
{
    atomSpace = SchemeSmob::ss_get_env_as("PatternIndex");
    schemeEval = new SchemeEval(atomSpace);
    lastUsedTicket = 0;
}

PatternIndexAPI::~PatternIndexAPI()
{
	for (const IndexMap::value_type& index : indexes) {
        delete index.second.first;
    }
}

string PatternIndexAPI::getStringProperty(StringMap &map, const string key)
{
    StringMap::iterator it = map.find(key);
    if (it == map.end()) {
        throw runtime_error("Invalid property key: " + key);
    } else {
        return it->second;
    }
}

int PatternIndexAPI::getIntProperty(StringMap &map, 
                                    const string key, 
                                    int min, 
                                    int max)
{
    int answer;

    StringMap::iterator it = map.find(key);
    if (it == map.end()) {
        throw runtime_error("Invalid property key: " + key);
    } else {
        answer = stoi(it->second);
        if ((answer < min) || (answer > max)) {
            string m = "Invalid value for " + 
                            key + ": " + to_string(answer);
            throw runtime_error(m);
        }
    }

    return answer;
}

bool PatternIndexAPI::getBoolProperty(StringMap &map, const string key)
{
    bool answer;

    StringMap::iterator it = map.find(key);
    if (it == map.end()) {
        throw runtime_error("Invalid property key: " + key);
    } else {
        string value = it->second;
        transform(value.begin(), value.end(), value.begin(), ::tolower);
        if (value.compare("true") == 0) {
            answer = true;
        } else if (value.compare("false") == 0) {
            answer = false;
        } else {
            string m = "Invalid value for " + 
                            key + ": " + it->second;
            throw runtime_error(m);
        }
    }

    return answer;
}

// Default properties are set using the config file
void PatternIndexAPI::setDefaultProperty(StringMap &map, const string key)
{
    string value = config().get("PatternIndex::" + key);
    map.insert(StringMap::value_type(key, value));
}

void PatternIndexAPI::setDefaultProperties(StringMap &properties)
{
    // -----------------------------------------------------------------------
    // Relevant for querying and mining
    // -----------------------------------------------------------------------

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
    // -----------------------------------------------------------------------

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
    // If "PermutationsOfVarsConsideredEquivalent" == "false" all the following
    // results will be returned
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
    // setting this parameter "true" will avoid this permutation in query
    // result.
    //
    setDefaultProperty(properties, "PermutationsOfVarsConsideredEquivalent");

    // -----------------------------------------------------------------------
    // Relevant for mining
    // -----------------------------------------------------------------------

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
    // (see the PatternMiner documentation in Wiki to understand the meaning 
    // of a pattern's GRAM)
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
    // usually will lead to better (higher pattern surprisingness) results IFF
    // the knowledge base have a sensible set of InheritanceLinks. 
    //
    // Normalized versions have the same (time) cost of the non-normalized
    // counterpart.
    setDefaultProperty(properties, "PatternRankingMetric");

    // The function used to compute coherence(x: atom). It is used by the mining
    // algorithm to compute surprinsigness (a metric for a pattern's quality).
    //
    // Actually (currently) there's no alternative to the default "const1" which
    // is coherence(x) = 1 for all x.
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

    // A list of OpenCog's atom types that should be used as "root"
    // subcomponents to build patterns.
    //
    // E.g. if a gram-2 pattern like this:
    //
    // (AndLink
    //   (TypeA ... )
    //   (TypeB ... )
    // )
    //
    // is supposed to be considered in the pattern mining, so TypeA and TypeB
    // should be listed here.
    setDefaultProperty(properties, "RootTypesUsedToBuildPatterns");

    // A list of OpenCog's atom types that may become variables in patterns.
    //
    // (see the PatternMiner documentation in Wiki to understand how patterns
    // are built from AtomSpace subgraphs)
    setDefaultProperty(properties, "TypesAllowedToBecomeVariables");
}

Handle PatternIndexAPI::createIndex(const string &scmPath)
{

    int tkt = -1;
    TypeFrameIndex *index = new TypeFrameIndex();
    TypeFrameIndexBuilder builder(index);

    if (SCMLoader::load(scmPath, *atomSpace, &builder)) {
        string m ="Error creating PatternIndex. SCM file is invalid.\n";
        throw runtime_error(m);
    } else {
        index->buildSubPatternsIndex();
        tkt = ++lastUsedTicket;
        StringMap props;
        setDefaultProperties(props);
        indexes.insert(IndexMap::value_type(tkt, make_pair(index, props)));
    }

    return atomSpace->add_node(ANCHOR_NODE, to_string(tkt));
}

Handle PatternIndexAPI::createIndex(const HandleSeq &handles)
{

    int tkt = -1;
    TypeFrameIndex *index = new TypeFrameIndex();

    for (const Handle h : handles) {
        index->add(h, 0);
    }

    index->buildSubPatternsIndex();
    tkt = ++lastUsedTicket;
    StringMap props;
    setDefaultProperties(props);
    indexes.emplace(tkt, make_pair(index, props));

    return atomSpace->add_node(ANCHOR_NODE, to_string(tkt));
}

void PatternIndexAPI::deleteIndex(Handle key)
{
    IndexMap::iterator it = indexes.find(stoi(key->get_name()));
    if (it == indexes.end()) {
        throw runtime_error("Invalid index key: " + key->to_string());
    }

    delete it->second.first;
    indexes.erase(it);
}

void PatternIndexAPI::applyProperties(Handle key)
{
    IndexMap::iterator it = indexes.find(stoi(key->get_name()));
    if (it == indexes.end()) {
        throw runtime_error("Invalid index key: " + key->to_string());
    }

    TypeFrameIndex *index = it->second.first;

    index->PATTERN_COUNT_CACHE_ENABLED = getBoolProperty(it->second.second, "PatternCountCacheEnabled");
    int n = getIntProperty(it->second.second, "NumberOfEvaluationThreads", 1);
    index->NUMBER_OF_EVALUATION_THREADS = n;
    if (n > 1) {
        index->PATTERN_COUNT_CACHE_ENABLED = false;
    }
    index->MINIMAL_FREQUENCY_TO_COMPUTE_QUALITY_METRIC = getIntProperty(it->second.second, "MinimalFrequencyToComputeQualityMetric", 1);
    index->MAX_SIZE_OF_COMPOUND_FRAMES_QUEUE = getIntProperty(it->second.second, "MaximumSizeOfCompoundFramesQueue", 1);

    string s = getStringProperty(it->second.second, "CoherenceFunction");
    if (s.compare("const1") == 0) {
        index->COHERENCE_FUNCTION = TypeFrameIndex::CONST_1;
    } else {
        throw runtime_error("Invalid value for CoherenceFunction: " + s);
    }

    s = getStringProperty(it->second.second, "CoherenceModulatorG");
    if (s.compare("oneOverCoherence") == 0) {
        index->COHERENCE_MODULATOR_G = TypeFrameIndex::ONE_OVER_COHERENCE;
    } else {
        throw runtime_error("Invalid value for CoherenceModulatorG: " + s);
    }

    s = getStringProperty(it->second.second, "CoherenceModulatorH");
    if (s.compare("coherence") == 0) {
        index->COHERENCE_MODULATOR_H = TypeFrameIndex::COHERENCE;
    } else {
        throw runtime_error("Invalid value for CoherenceModulatorH: " + s);
    }

    index->DIFFERENT_ASSIGNMENT_FOR_DIFFERENT_VARS = getBoolProperty(it->second.second, "DifferentAssignmentForDifferentVars");
    index->PERMUTATIONS_OF_VARS_CONSIDERED_EQUIVALENT = getBoolProperty(it->second.second, "PermutationsOfVarsConsideredEquivalent");
    index->PATTERNS_GRAM = getIntProperty(it->second.second, "PatternsGram", 1);
    index->MAXIMUM_NUMBER_OF_MINING_RESULTS = getIntProperty(it->second.second, "MaximumNumberOfMiningResults", 1);

    s = getStringProperty(it->second.second, "PatternRankingMetric");
    if (s.compare("isurprisingness") == 0) {
        index->PATTERN_RANKING_METRIC = TypeFrameIndex::I_SURPRISINGNESS;
    } else if (s.compare("nisurprisingness") == 0) {
        index->PATTERN_RANKING_METRIC = TypeFrameIndex::N_I_SURPRISINGNESS;
    } else if (s.compare("iisurprisingness") == 0) {
        index->PATTERN_RANKING_METRIC = TypeFrameIndex::II_SURPRISINGNESS;
    } else if (s.compare("niisurprisingness") == 0) {
        index->PATTERN_RANKING_METRIC = TypeFrameIndex::N_II_SURPRISINGNESS;
    } else {
        throw runtime_error("Invalid value for PatternRankingMetric: " + s);
    }

    s = getStringProperty(it->second.second, "RootTypesUsedToBuildPatterns");
    istringstream iss1(s);
    string stype;    
    while (getline(iss1, stype, ',')) {
        if (nameserver().isDefined(stype)) {
            index->ALLOWED_TOP_LEVEL_TYPES.insert(nameserver().getType(stype));
        } else {
            printf("type: %u\n", nameserver().getType(stype));
            throw runtime_error("Invalid value for RootTypesUsedToBuildPatterns. Unknown type: " + stype);
        }
    }

    s = getStringProperty(it->second.second, "TypesAllowedToBecomeVariables");
    istringstream iss2(s);
    while (getline(iss2, stype, ',')) {
        if (nameserver().isDefined(stype)) {
            index->ALLOWED_VAR_SUBSTITUTION.insert(nameserver().getType(stype));
        } else {
            printf("type: %u\n", nameserver().getType(stype));
            throw runtime_error("Invalid value for TypesAllowedToBecomeVariables. Unknown type: " + stype);
        }
    }
}

void PatternIndexAPI::setProperty(Handle key, 
                                  const string &propertyName, 
                                  const string &propertyValue)
{
    IndexMap::iterator it1 = indexes.find(stoi(key->get_name()));
    if (it1 == indexes.end()) {
        throw runtime_error("Invalid index key: " + key->to_string());
    }
    StringMap::iterator it2 = it1->second.second.find(propertyName);
    if (it2 == it1->second.second.end()) {
        throw runtime_error("Invalid property name: " + propertyName);
    } else {
        it2->second = propertyValue;
    }
}

void PatternIndexAPI::query(vector<QueryResult> &answer, 
                            Handle key, 
                            const TypeFrame &query)
{
    applyProperties(key);
    vector<TypeFrameIndex::ResultPair> queryResults;
    indexes.find(stoi(key->get_name()))->second.first->query(queryResults, query);
    for (const TypeFrameIndex::ResultPair& result : queryResults) {
        HandleSeq atoms;
        VariableMapping mapping;
        for (const TypeFrame& frame : result.first) {
            atoms.push_back(schemeEval->eval_h(frame.toSCMString()));
        }
        for (const auto& varframe : result.second) {
            Handle var = schemeEval->eval_h("(VariableNode \"" + varframe.first + "\")");
            Handle value = schemeEval->eval_h(varframe.second.toSCMString());
            mapping.emplace_back(var, value);
        }
        answer.emplace_back(atoms, mapping);
    }
}

void PatternIndexAPI::query(vector<QueryResult> &answer, 
                            Handle key, 
                            const string &queryStr)
{
    TypeFrame queryFrame(queryStr);
    query(answer, key, queryFrame);
}

Handle PatternIndexAPI::query(Handle key, Handle queryLink)
{
    TypeFrame queryFrame(queryLink);

    vector<QueryResult> queryResults;
    query(queryResults, key, queryFrame);
    HandleSeq resultVector;
    HandleSeq variableMapping;
    for (const QueryResult& result : queryResults) {
        variableMapping.clear();
        for (const HandlePair& vargnd : result.second) {
            Handle link = atomSpace->add_link(LIST_LINK,
                                              vargnd.first,
                                              vargnd.second);
            variableMapping.push_back(link);
        }
        Handle satisfyingSubgraph = atomSpace->add_link(LIST_LINK, result.first);
        Handle varMapping = atomSpace->add_link(LIST_LINK, variableMapping);
        Handle rLink = atomSpace->add_link(LIST_LINK,
                                           satisfyingSubgraph,
                                           varMapping);
        resultVector.push_back(rLink);
    }

    return atomSpace->add_link(LIST_LINK, resultVector);
}

void PatternIndexAPI::minePatterns(vector<MiningResult> &answer, 
                                   Handle key)
{
    applyProperties(key);

    vector<pair<float,TypeFrame>> patterns;
    indexes.find(stoi(key->get_name()))->second.first->minePatterns(patterns);
    for (const auto& pattern : patterns) {
        answer.emplace_back(pattern.first,
                            schemeEval->eval_h(pattern.second.toSCMString()));
    }
}

Handle PatternIndexAPI::minePatterns(Handle key)
{
    vector<MiningResult> miningResults;
    minePatterns(miningResults, key);
    HandleSeq resultVector;
    for (const MiningResult& result : miningResults) {
        Handle quality = atomSpace->add_node(NUMBER_NODE,
                                             to_string(result.first));
        Handle link = atomSpace->add_link(LIST_LINK,
                                          quality,
                                          result.second);
        resultVector.push_back(link);
    }

    return atomSpace->add_link(LIST_LINK, resultVector);
}

// Compliance with OpenCog's style of singleton access.
// using patternindex() or PatternIndexAPI::getInstance() is equivalent
PatternIndexAPI &opencog::patternindex()
{
    return PatternIndexAPI::getInstance();
}

