#include "PatternIndexAPI.h"

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
    // Enables the use of a cache to count pattern occurrences.
    // If this flag is set "true", queries and mining tend to be faster
    // at the cost of memory.
    //
    // Tipically, you will want to set this "true" for small/medium datasets
    // and "false" for large (> 5K/10K links) ones. The actual limits will (of
    // course) depend on the dataset and the amount of available memory.
    // 
    // If the number of evaluation threads (see below) is set to n > 1
    // then this parameter is automatically set to "false".
    properties.insert(PropertyMap::value_type("PatternCountCacheEnabled", "true"));

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
        applyProperties(ticket);
    }

    return ticket;
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
}

void PatternIndexAPI::setProperty(int key, const std::string &propertyName, const std::string &propertyValue)
{
    checkKey(key);
    PropertyMap::iterator it = properties.at(key).find(propertyName);
    if (it == properties.at(key).end()) {
        throw std::runtime_error("Invalid property name");
    } else {
        (*it).second = propertyValue;
        applyProperties(key);
    }
}

TypeFrameIndex::RankingMetric PatternIndexAPI::selectMetric(MinedPatternsRankingMetric src)
{
    switch (src) {
        case I_SURPRISINGNESS: return TypeFrameIndex::I_SURPRISINGNESS;
        case N_I_SURPRISINGNESS: return TypeFrameIndex::N_I_SURPRISINGNESS;
        case II_SURPRISINGNESS: return TypeFrameIndex::II_SURPRISINGNESS;
        case N_II_SURPRISINGNESS: return TypeFrameIndex::N_II_SURPRISINGNESS;
        default: throw std::runtime_error("Unknown pattern ranking metric\n");
    }
}

void PatternIndexAPI::query(std::vector<QueryResult> &answer, int key, const std::string &queryStr, bool distinct, bool noPermutations)
{
    std::vector<TypeFrameIndex::ResultPair> queryResult;
    indexVector.at(key)->query(queryResult, queryStr, distinct, noPermutations);
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

void PatternIndexAPI::minePatterns(std::vector<MiningResult> &answer, int key, unsigned int components, unsigned int maxResults, MinedPatternsRankingMetric metric)
{
    std::vector<std::pair<float,TypeFrame>> patterns;
    indexVector.at(key)->minePatterns(patterns, components, maxResults, selectMetric(metric));
    for (unsigned int i = 0; i < patterns.size(); i++) {
        answer.push_back(std::make_pair(patterns.at(i).first, schemeEval->eval_h(patterns.at(i).second.toSCMString())));
    }
}

