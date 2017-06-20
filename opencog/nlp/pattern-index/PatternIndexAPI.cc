#include "PatternIndexAPI.h"

#include <opencog/guile/SchemePrimitive.h>
#include "SCMLoader.h"
#include "TypeFrameIndexBuilder.h"

using namespace opencog;

PatternIndexAPI::PatternIndexAPI()
{
    atomSpace = SchemeSmob::ss_get_env_as("PatternIndex");
    schemeEval = new SchemeEval(atomSpace);
}

PatternIndexAPI::~PatternIndexAPI()
{
    for (unsigned int i = 0; i < indexVector.size(); i++) {
        delete indexVector.at(i);
    }
}

void PatternIndexAPI::setDefaultProperties(PropertyMap &properties)
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
    properties.insert(PropertyMap::value_type("PatternCountCacheEnabled", "true"));

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
    properties.insert(PropertyMap::value_type("DifferentAssignmentForDifferentVars", "true"));

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
    properties.insert(PropertyMap::value_type("PermutationsOfVarsConsideredEquivalent", "true"));

    // -----------------------------------------------------------------------
    // Relevant for mining

    // Number of auxiliary threads used to compute patterns quality. 
    // Need to be >= 1.
    // Usually this is set to the number of availables processors. There is no
    // point in using a larger value.
    properties.insert(PropertyMap::value_type("NumberOfEvaluationThreads", "4"));

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
    properties.insert(PropertyMap::value_type("MinimalFrequencyToComputeQualityMetric", "5"));

    // The size of an auxiliary queue used in the mining algorithm.
    // This parameter is here because it will determine the amount of memory
    // used by the mining algorithm.
    //
    // Again, there is no default that makes sense for every application. In my
    // tests, setting this to 5000000 will cause the mining algorithm to use
    // +- 8G of RAM. But this will change for different datasets so you may need
    // to adjust it to fit in your use case.
    //
    // In general, the mining algorithm will run slightly faster with larger
    // values. 
    properties.insert(PropertyMap::value_type("MaximumSizeOfCompoundFramesQueue", "5000000"));

    // Only patterns of specified GRAM will be evaluated.
    // (see the PatternMiner documentation in Wiki to understand the meaning of a pattern's GRAM)
    //
    // This parameter have a HUGE effect in time performance and pattern overall
    // "quality". Usually "PatternsGram" < 3 will lead to useless patterns.
    // For large knowledge bases, searching for > 3 gram patterns is not viable.
    properties.insert(PropertyMap::value_type("PatternsGram", "3"));

    // The pattern searching algorithm will always keep the best
    // "MaximumNumberOfMiningResults" patterns found so far. When the search
    // ends, these patterns are returned as the mining results.
    //
    // This parameter have a small effect in memory usage and execution time.
    // Internally, the algorithm keeps a heap of results. Thus although using
    // large (> 1000000) values here are possible, this may slow down the
    // search.
    properties.insert(PropertyMap::value_type("MaximumNumberOfMiningResults", "10"));

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
    properties.insert(PropertyMap::value_type("PatternRankingMetric", "nisurprisingness"));

    // The function used to compute coherence(x: atom). It is used by the mining
    // algorithm to compute surprinsigness (a metric for a pattern's quality).
    //
    // Actually (currently) there's no alternative to the default "const1" which is
    // coherence(x) = 1 for all x.
    properties.insert(PropertyMap::value_type("CoherenceFunction", "const1"));

    // This is the function g(x) mentioned in the document that describes how to
    // compute surprisingness of a pattern. It is used by the mining
    // algorithm to compute surprinsigness (a metric for a pattern's quality).
    //
    // Actually (currently) there's no alternative to the default
    // "oneOverCoherence" which is g(x) = 1 / coherence(x).
    properties.insert(PropertyMap::value_type("CoherenceModulatorG", "oneOverCoherence"));

    // This is the function h(x) mentioned in the document that describes how to
    // compute surprisingness of a pattern. It is used by the mining
    // algorithm to compute surprinsigness (a metric for a pattern's quality).
    //
    // Actually (currently) there's no alternative to the default
    // "coherence" which is h(x) = coherence(x).
    properties.insert(PropertyMap::value_type("CoherenceModulatorH", "coherence"));
}

int PatternIndexAPI::createNewIndex(const std::string &scmPath)
{

    int ticket = -1;

    TypeFrameIndex *index = new TypeFrameIndex();
    TypeFrameIndexBuilder builder(index);

    if (SCMLoader::load(scmPath, *atomSpace, &builder)) {
        throw std::runtime_error("Error creating PatternIndex. SCM file is invalid.\n");
    } else {
        index->buildSubPatternsIndex();
        ticket = indexVector.size();
        indexVector.push_back(index);
        PropertyMap prop;
        setDefaultProperties(prop);
        properties.push_back(prop);
    }

    return ticket;
}

Handle PatternIndexAPI::createNewIndex_h(const std::string &scmPath)
{
    return atomSpace->add_node(ANCHOR_NODE, std::to_string(createNewIndex(scmPath)));
}

void PatternIndexAPI::checkKey(int key)
{
    if ((key < 0) || (key >= (int) indexVector.size())) {
        throw std::runtime_error("Invalid index key");
    }
}

void PatternIndexAPI::applyProperties(int key)
{
    checkKey(key);
    PropertyMap::iterator it;

    it = properties.at(key).find("PatternCountCacheEnabled");
    if ((*it).second.compare("true") == 0) {
        indexVector.at(key)->PATTERN_COUNT_CACHE_ENABLED = true;
    } else if ((*it).second.compare("false") == 0) {
        indexVector.at(key)->PATTERN_COUNT_CACHE_ENABLED = false;
    } else {
        throw std::runtime_error("Invalid value for PatternCountCacheEnabled");
    }

    it = properties.at(key).find("NumberOfEvaluationThreads");
    int n = std::stoi((*it).second);
    if (n > 0) {
        indexVector.at(key)->NUMBER_OF_EVALUATION_THREADS = n;
        if (n > 1) {
            indexVector.at(key)->PATTERN_COUNT_CACHE_ENABLED = false;
        }
    } else {
        throw std::runtime_error("Invalid value for NumberOfEvaluationThreads");
    }

    it = properties.at(key).find("MinimalFrequencyToComputeQualityMetric");
    n = std::stoi((*it).second);
    if (n > 0) {
        indexVector.at(key)->MINIMAL_FREQUENCY_TO_COMPUTE_QUALITY_METRIC = n;
    } else {
        throw std::runtime_error("Invalid value for MinimalFrequencyToComputeQualityMetric");
    }

    it = properties.at(key).find("MaximumSizeOfCompoundFramesQueue");
    n = std::stoi((*it).second);
    if (n > 0) {
        indexVector.at(key)->MAX_SIZE_OF_COMPOUND_FRAMES_QUEUE = n;
    } else {
        throw std::runtime_error("Invalid value for MaximumSizeOfCompoundFramesQueue");
    }

    //TODO: Change the way these functions are set
    /*
    it = properties.at(key).find("CoherenceFunction");
    if ((*it).second.compare("const1") == 0) {
        indexVector.at(key)->COHERENCE_FUNCTION = TypeFrameIndex::CONST1;
    } else {
        throw std::runtime_error("Invalid value for CoherenceFunction");
    }

    it = properties.at(key).find("CoherenceModulatorG");
    if ((*it).second.compare("oneOverCoherence") == 0) {
        indexVector.at(key)->COHERENCE_FUNCTION = TypeFrameIndex::ONE_OVER_COHERENCE;
    } else {
        throw std::runtime_error("Invalid value for CoherenceModulatorG");
    }

    it = properties.at(key).find("CoherenceModulatorH");
    if ((*it).second.compare("coherence") == 0) {
        indexVector.at(key)->COHERENCE_FUNCTION = TypeFrameIndex::COHERENCE;
    } else {
        throw std::runtime_error("Invalid value for CoherenceModulatorH");
    }
    */

    it = properties.at(key).find("DifferentAssignmentForDifferentVars");
    if ((*it).second.compare("true") == 0) {
        indexVector.at(key)->DIFFERENT_ASSIGNMENT_FOR_DIFFERENT_VARS = true;
    } else if ((*it).second.compare("false") == 0) {
        indexVector.at(key)->DIFFERENT_ASSIGNMENT_FOR_DIFFERENT_VARS = false;
    } else {
        throw std::runtime_error("Invalid value for DifferentAssignmentForDifferentVars");
    }

    it = properties.at(key).find("PermutationsOfVarsConsideredEquivalent");
    if ((*it).second.compare("true") == 0) {
        indexVector.at(key)->PERMUTATIONS_OF_VARS_CONSIDERED_EQUIVALENT = true;
    } else if ((*it).second.compare("false") == 0) {
        indexVector.at(key)->PERMUTATIONS_OF_VARS_CONSIDERED_EQUIVALENT = false;
    } else {
        throw std::runtime_error("Invalid value for PermutationsOfVarsConsideredEquivalent");
    }

    it = properties.at(key).find("PatternsGram");
    n = std::stoi((*it).second);
    if (n > 0) {
        indexVector.at(key)->PATTERNS_GRAM = n;
    } else {
        throw std::runtime_error("Invalid value for PatternsGram");
    }

    it = properties.at(key).find("MaximumNumberOfMiningResults");
    n = std::stoi((*it).second);
    if (n > 0) {
        indexVector.at(key)->MAXIMUM_NUMBER_OF_MINING_RESULTS = n;
    } else {
        throw std::runtime_error("Invalid value for MaximumNumberOfMiningResults");
    }

    it = properties.at(key).find("PatternRankingMetric");
    if ((*it).second.compare("isurprisingness") == 0) {
        indexVector.at(key)->PATTERN_RANKING_METRIC = TypeFrameIndex::I_SURPRISINGNESS;
    } else if ((*it).second.compare("nisurprisingness") == 0) {
        indexVector.at(key)->PATTERN_RANKING_METRIC = TypeFrameIndex::N_I_SURPRISINGNESS;
    } else if ((*it).second.compare("iisurprisingness") == 0) {
        indexVector.at(key)->PATTERN_RANKING_METRIC = TypeFrameIndex::II_SURPRISINGNESS;
    } else if ((*it).second.compare("niisurprisingness") == 0) {
        indexVector.at(key)->PATTERN_RANKING_METRIC = TypeFrameIndex::N_II_SURPRISINGNESS;
    } else {
        throw std::runtime_error("Invalid value for PatternRankingMetric");
    }
}

void PatternIndexAPI::setProperty(Handle key, const std::string &propertyName, const std::string &propertyValue)
{
    setProperty(std::stoi(key->getName()), propertyName, propertyValue);
}

void PatternIndexAPI::setProperty(int key, const std::string &propertyName, const std::string &propertyValue)
{
    checkKey(key);
    PropertyMap::iterator it = properties.at(key).find(propertyName);
    if (it == properties.at(key).end()) {
        throw std::runtime_error("Invalid property name");
    } else {
        (*it).second = propertyValue;
    }
}

void PatternIndexAPI::query(std::vector<QueryResult> &answer, int key, const TypeFrame &query)
{
    applyProperties(key);

    std::vector<TypeFrameIndex::ResultPair> queryResult;
    indexVector.at(key)->query(queryResult, query);
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

void PatternIndexAPI::query(std::vector<QueryResult> &answer, int key, const std::string &queryStr)
{
    TypeFrame queryFrame(queryStr);
    query(answer, key, queryFrame);
}

Handle PatternIndexAPI::query(Handle keyNode, Handle queryLink)
{
    int key = std::stoi(keyNode->getName());
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

void PatternIndexAPI::minePatterns(std::vector<MiningResult> &answer, int key)
{
    applyProperties(key);

    std::vector<std::pair<float,TypeFrame>> patterns;
    indexVector.at(key)->minePatterns(patterns);
    for (unsigned int i = 0; i < patterns.size(); i++) {
        answer.push_back(std::make_pair(patterns.at(i).first, schemeEval->eval_h(patterns.at(i).second.toSCMString())));
    }
}

Handle PatternIndexAPI::minePatterns(Handle keyNode)
{
    int key = std::stoi(keyNode->getName());
    std::vector<MiningResult> miningResults;
    minePatterns(miningResults, key);
    HandleSeq resultVector;
    for (unsigned int i = 0; i < miningResults.size(); i++) {
        Handle quality = atomSpace->add_node(NUMBER_NODE, std::to_string(miningResults.at(i).first));
        resultVector.push_back(atomSpace->add_link(LIST_LINK, quality, miningResults.at(i).second));
    }

    return atomSpace->add_link(LIST_LINK, resultVector);
}
