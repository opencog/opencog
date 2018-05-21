/*
 * opencog/learning/PatternMiner/PatternMiner.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com> in 2014
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

#ifndef _OPENCOG_PATTERNMINER_PATTERNMINER_H
#define _OPENCOG_PATTERNMINER_PATTERNMINER_H
#include <cstdio>
#include <map>
#include <mutex>
#include <thread>
#include <vector>
#include <fstream>

#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/atomspace/AtomSpace.h>

#include <opencog/learning/PatternMiner/types/atom_types.h>

#include "HTree.h"
#include "Parameters.h"

using namespace std;

class PatternMinerUTest;

namespace opencog
{

class PatternMinerSCM;

namespace PatternMining
{
#define FLOAT_MIN_DIFF 0.00001
#define FREQUENCY_TOP_THRESHOLD 0.02
#define FREQUENCY_BOTTOM_THRESHOLD 0.95
#define SURPRISINGNESS_I_TOP_THRESHOLD 0.20
#define SURPRISINGNESS_II_TOP_THRESHOLD 0.40
#define OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE 1
#define GENERATE_TMP_PATTERNS 0
#define USE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE 0
#define USE_QUERY_ALL_ENTITY_COUNT 0
#define USE_ABS_SURPRISINGNESS 1
#define LINE_INDENTATION "  "
#define CALCULATE_TYPE_B_SURPRISINGNESS 1
#define GENERATE_TYPE_B_RELATION_WHEN_CALCULATE_SURPRISINGNESS 0

#define PATTERN_VARIABLENODE_TYPE PATTERN_VARIABLE_NODE

struct _non_ordered_pattern
{
    Handle link;
    vector<vector<std::pair<int, std::size_t>>> indexesOfSharedVars;

    bool operator<(const _non_ordered_pattern& other) const
    {
        for (unsigned int i = 0; i < indexesOfSharedVars.size(); ++ i)
        {
            if (indexesOfSharedVars[i].size() < other.indexesOfSharedVars[i].size())
                return true;
            else if (indexesOfSharedVars[i].size() > other.indexesOfSharedVars[i].size())
                return false;

            for (unsigned int j = 0; j < indexesOfSharedVars[i].size(); ++ j)
            {
                if ((indexesOfSharedVars[i][j]).first < (other.indexesOfSharedVars[i][j]).first)
                    return true;
                else if ((indexesOfSharedVars[i][j]).first > (other.indexesOfSharedVars[i][j]).first)
                    return false;

                if ((indexesOfSharedVars[i][j]).second < (other.indexesOfSharedVars[i][j]).second)
                    return true;
                else if ((indexesOfSharedVars[i][j]).second > (other.indexesOfSharedVars[i][j]).second)
                    return false;
            }
        }

        // if all above criteria cannot figure out the order of these two patterns, just return true and output a warning
        cout << "\n warning: _non_ordered_pattern: Fail to figure out the order of two patterns!\n";
        // TODO if there are equal, then it should be false.
        return true;
    }
};

// TODO this seems identical to _non_ordered_pattern
struct advanced_non_ordered_pattern // only when complex patterns like pln patterns, used when enable_unify_unordered_links = true
{
    Handle link;
    vector<vector<std::pair<int, std::size_t>>> indexesOfSharedVars; // the shared vars indexes appear in the already sorted links

    bool operator<(const _non_ordered_pattern& other) const
    {
        // first, use indexesOfSharedVars
        for (unsigned int i = 0; i < indexesOfSharedVars.size(); ++ i)
        {
            if (indexesOfSharedVars[i].size() < other.indexesOfSharedVars[i].size())
                return true;
            else if (indexesOfSharedVars[i].size() > other.indexesOfSharedVars[i].size())
                return false;

            for (unsigned int j = 0; j < indexesOfSharedVars[i].size(); ++ j)
            {
                if ((indexesOfSharedVars[i][j]).first < (other.indexesOfSharedVars[i][j]).first)
                    return true;
                else if ((indexesOfSharedVars[i][j]).first > (other.indexesOfSharedVars[i][j]).first)
                    return false;

                if ((indexesOfSharedVars[i][j]).second < (other.indexesOfSharedVars[i][j]).second)
                    return true;
                else if ((indexesOfSharedVars[i][j]).second > (other.indexesOfSharedVars[i][j]).second)
                    return false;
            }
        }

        // if indexesOfSharedVars cannot decide the result, use

        // if all above criteria cannot figure out the order of these two patterns, just return true and output a warning
        cout << "\n warning: _non_ordered_pattern: Fail to figure out the order of two patterns!\n";
        return true;
    }
};

//! Returns a string from the given argument by using the << operator
template <typename T>
std::string toString(T data)
{
    std::ostringstream oss;
    oss << data;
    return oss.str();
}

struct MinedPatternInfo
{
    string curPatternKeyStr;
    string parentKeyString;
    unsigned int extendedLinkIndex;
    bool notOutPutPattern;
};

class PatternMiner
{
    friend class ::PatternMinerUTest;
	friend class opencog::PatternMinerSCM;

protected:

    HTree* htree;
    AtomSpace* as;              // AtomSpace holding the patterns
    AtomSpace& original_as;
    AtomSpace* observing_as;

    HandleSeq allLinks;// all links in the orginal atomspace

    // Every pattern is reprented as a unique string as the key in this map, mapping to its cooresponding HTreeNode
    map<string, HTreeNode*> keyStrToHTreeNodeMap;

    vector<vector<HTreeNode*>> patternsForGram;
    vector<vector<HTreeNode*>> finalPatternsForGram;

    // temp patterns generated only for calcuate the interestingness of its superpatterns, e.g. patterns with too many variables
    vector<vector<HTreeNode*>> tmpPatternsForGram;

    map<string, unsigned int> allEntityNumMap;

    std::thread *threads;

	Parameters param;

    bool is_distributed;

    unsigned int cur_gram;

    int cur_index;

    int allLinkNumber;

    unsigned int linksPerThread;

    float last_gram_total_float;

    unsigned int num_of_patterns_without_superpattern_cur_gram;
    unsigned int *num_of_patterns_with_1_frequency;

    std::mutex uniqueKeyLock, patternForLastGramLock, removeAtomLock,
        patternMatcherLock, addNewPatternLock, calculateIILock,
        readNextLinkLock,actualProcessedLinkLock, curDFExtractedLinksLock,
        readNextPatternLock, threadExtractedLinksLock;

    HandleSet black_keyword_Handles; // only use when keyword_black_logic_is_contain = false

//    Handle FrequencyHandle;
//    Handle InteractionInformationHandle;
//    Handle SurprisingnessIHandle;
//    Handle SurprisingnessIIHandle;
    Handle PatternValuesHandle;

    float atomspaceSizeFloat;

    float surprisingness_II_threshold;

    //debug
    unsigned int processedLinkNum; // include those links of ignored types
    unsigned int actualProcessedLinkNum;

    vector<vector<vector<unsigned int>>> components_ngram[3];

    HandleSeq allLinksContainWhiteKeywords;
    HandleSet havenotProcessedWhiteKeywordLinks;

    // [gram], this to avoid different threads happen to work on the same links.
    // each string is composed the handles of a group of fact links in the observing_as in the default hash order using std set
    // queue<string>[thread_num][max_gram]
    list<string>** thread_DF_ExtractedLinks;
    // set<string>[max_gram]
    set<string>* all_thread_ExtractedLinks_pergram;

    HandleSeq allDBpediaKeyNodes;

    std::ofstream surpringnessIICalfile;

    /**
     * Re-order the inputPattern so that, after abstracting away the
     * variable names, the unique sub-patterns come first (according
     * to some fixed order defined by their content -- currently using
     * their string representations). Then the sub-patterns grouped by
     * structures (after abstracting away the variable names). Groups
     * are ordered according to their structures (using again their
     * string representations). And sub-patterns within groups are
     * ordered according to how their mapping to their structures
     * after abstracting names away.
     *
     * The value unifiedLastLinkIndex represents the index of the last
     * sub-pattern in the new order.
     *
     * For instance, if inputPattern is
     *
     *    (InheritanceLink
     *       (VariableNode "$1")
     *       (ConceptNode "Animal")
     *
     *    (InheritanceLink
     *       (VariableNode "$2")
     *       (VariableNode "$1")
     *
     *    (InheritanceLink
     *       (VariableNode "$3")
     *       (VariableNode "$2")
     *
     *    (EvaluationLink (stv 1 1)
     *       (PredicateNode "like_food")
     *       (ListLink
     *          (VariableNode "$3")
     *          (ConceptNode "meat")
     *       )
     *    )
     *
     * The output may look like
     *
     *
     *    (InheritanceLink
     *       (VariableNode "$1")
     *       (ConceptNode "Animal")
     *
     *    (EvaluationLink (stv 1 1)
     *       (PredicateNode "like_food")
     *       (ListLink
     *          (VariableNode "$3")
     *          (ConceptNode "meat")
     *       )
     *    )
     *
     *    (InheritanceLink
     *       (VariableNode "$2")
     *       (VariableNode "$1")
     *
     *    (InheritanceLink
     *       (VariableNode "$3")
     *       (VariableNode "$2")
     *
     * and unifiedLastLinkIndex = 1
     *
     * TODO can be simplified by generalizing to atoms instead of
     * links.
     */
    HandleSeq _UnifyPatternOrder(const HandleSeq& inputPattern, unsigned int& unifiedLastLinkIndex);

    /**
     * TODO write comment
     */
    HandleSeq UnifyPatternOrder(const HandleSeq& inputPattern, unsigned int& unifiedLastLinkIndex, HandleMap& orderedVarNameMap);

    /**
     * Replace all unordered links within `link` by a ListLink, where
     * all outgoings follow a determined ordered (defined by
     * _UnifyPatternOrder). In addition, fill orderedTmpLinkToType
     * with associating replacement links by their original link type.
     *
     * TODO can be simplified by generalizing to atoms instead of
     * links.
     */
    Handle UnifyOneLinkForUnorderedLink(const Handle& link,std::map<Handle,Type>& orderedTmpLinkToType);

    Handle rebindLinkTypeRecursively(const Handle& inputLink, std::map<Handle,Type>& orderedTmpLinkToType);

    /**
     * Traverses link, if encouter a pattern variable not in
     * varNameMap, create a name for it and insert a pair {old
     * variable, new variable} in varNameMap. Produce a new outgoing
     * set of link with all new variables and return it.
     *
     * TODO orderedTmpLinkToType isn't used at the moment.
     */
    HandleSeq findAndRenameVariables(const Handle& link, HandleMap& varNameMap,
                                     const std::map<Handle,Type>& orderedTmpLinkToType);

    // Rename the variable names in a ordered pattern according to the orders of the variables appear in the orderedPattern
    HandleSeq RebindVariableNames(const HandleSeq& orderedPattern, HandleMap& orderedVarNameMap, const std::map<Handle,Type>& orderedTmpLinkToType);

    void ReplaceConstNodeWithVariableForOneLink(Handle link, Handle constNode, Handle newVariableNode, HandleSeq& renameOutgoingLinks);

    HandleSeq ReplaceConstNodeWithVariableForAPattern(const HandleSeq& pattern, Handle constNode, Handle newVariableNode);

    /**
     * For each pattern variable of `link` (presumably duplicates as
     * well), pushes onto `indexes` a vector of pairs such that the
     * first element of each pair is an index within orderedHandles,
     * and the second element is the location of the variable name
     * within the string representation of the handle at that index,
     * in case the variable appears in that handle at all.
     */
    void generateIndexesOfSharedVars(const Handle& link, const HandleSeq& orderedHandles, vector<vector<std::pair<int, size_t>>> &indexes);

	/**
	 * Given a mapping between handles, substitute all encountered
	 * handle keys by their values in h. The produced handle is added
	 * to the pattern miner atomspace and returned.
	 */
	Handle substitute(const Handle& h, const HandleMap& h2h);

    /**
     * Retrieve all nodes from a given link and its descendents. For
     * each node, associate a pattern variable with a generated name,
     * and insert that association to nodeToVar.
     */
    void associateNodesToVars(const Handle& link, HandleMap& nodesToVars);

    /**
     * Retrieve all nodes from the given link and its descendents,
     * fill nodes with them.
     */
    void extractNodes(const Handle& link, HandleSet& nodes);

    /**
     * Retrieve all pattern variable nodes from the given link and its
     * descendents, fill varNodes with them.
     */
    void extractVarNodes(const Handle& link, HandleSet& varNodes);

    /**
     * Retrieve all non pattern variable nodes from the given links
     * and its descendents, fill constNodes with them. Note that
     * regular variables are included as well, since they are
     * different then pattern variables.
     */
    void extractConstNodes(const Handle& link, HandleSet& constNodes);

	/**
	 * Return true iff an atom contains only pattern variable nodes,
	 * no const nodes.
	 */
    bool containOnlyVariables(Handle h);

	/**
	 * Return true iff an atom contains some pattern variable nodes.
	 */
    bool containSomeVariables(Handle link);

    void extractAllPossiblePatternsFromInputLinksBF(const HandleSeq& inputLinks, HTreeNode* parentNode, HandleSet& sharedNodes, unsigned int gram);

	/**
     * Copy the outgoings of `link` to `to_as` and return the copies
     *
     * Pattern variables are turned into regular variables while being
     * copied, filling `variables`.
     */
    HandleSeq copyOutgoings(AtomSpace& to_as, const Handle& link,
                            HandleSeq& variables);

    /**
     * Copy `h` to `to_as` and return the copy.
     *
     * Pattern variables are turned into regular variables while being
     * copied, filling `variables`.
     */
    Handle copyAtom(AtomSpace& to_as, const Handle& link,
                    HandleSeq& variables);

	/**
     * Copy `links` to `to_as` and return the copies.
     *
     * Pattern variables are turned into regular variables while being
     * copied, filling `variables`.
     *
     * TODO: can be simplified by generalizing to atoms rather than
     * links.
     */
    HandleSeq copyLinks(AtomSpace& to_as, const HandleSeq& links,
                        HandleSeq &variables);

    void swapOneLinkBetweenTwoAtomSpaceForBindLink(AtomSpace& to_as, const Handle& fromLink, HandleSeq& outgoings,
                                          HandleSeq &outVariableNodes, HandleSeq& linksWillBeDel, bool& containVar);

    HandleSeq swapLinksBetweenTwoAtomSpaceForBindLink(AtomSpace& to_as, const HandleSeq& fromLinks, HandleSeq& outVariableNodes, HandleSeq& linksWillBeDel);

    void extractAllVariableNodesInAnInstanceLink(const Handle& instanceLink, const Handle& patternLink, HandleSet& allVarNodes);

    void extendAllPossiblePatternsForOneMoreGramBF(const HandleSeq& instance, HTreeNode* curHTreeNode, unsigned int gram);

    void addThreadExtractedLinks(unsigned int _gram, unsigned int cur_thread_index, string _extractedLinkUIDs);

    bool existInAllThreadExtractedLinks(unsigned int _gram, string _extractedLinkUIDs);

    bool existInOneThreadExtractedLinks(unsigned int _gram, unsigned int cur_thread_index, string _extractedLinkUIDs);

    void extendAPatternForOneMoreGramRecursively(const Handle& extendedLink, AtomSpace& from_as, const Handle& extendedNode, const HandleSeq& lastGramLinks,
                                                 HTreeNode* parentNode, const HandleMap& lastGramValueToVarMap, const HandleMap& lastGramPatternVarMap,
                                                 bool isExtendedFromVar, set<string>& allNewMinedPatternsCurTask, vector<HTreeNode*>& allHTreeNodesCurTask,
                                                 vector<MinedPatternInfo>& allNewMinedPatternInfo, unsigned int thread_index, bool startFromLinkContainWhiteKeyword);

    /**
     * Return iff some sub-pattern contains only variables.
     *
     * For instance, in the following $var_3 isn't really a variable because...
     *
     * To exclude this kind of patterns:
     * $var_3 doesn't really can be a variable, all the links contains it doesn't contains any const nodes, so actually $var_3 is a leaf
     *
     * we call variable nodes like $var_3 as "loop variable"
     * (InheritanceLink
     *  (ConceptNode Broccoli)
     *  (VariableNode $var_1))
     *
     * (InheritanceLink
     *  (ConceptNode dragonfruit)
     *  (VariableNode $var_2))
     *
     * (InheritanceLink
     *  (VariableNode $var_2)
     *  (VariableNode $var_3))
     *
     * (InheritanceLink
     *  (VariableNode $var_1)
     *  (VariableNode $var_3))
     */
    bool containLoopVariable(const HandleSeq& pattern);

    void quoteAPattern(HTreeNode* hTreeNode);

    void quoteAllThePatternSForGram(unsigned int gram);

    HTreeNode* extractAPatternFromGivenVarCombination(HandleSeq &inputLinks, HandleMap &patternVarMap, HandleMap& orderedVarNameMap,HandleSeqSeq &oneOfEachSeqShouldBeVars, HandleSeq &leaves,HandleSeq &shouldNotBeVars, HandleSeq &shouldBeVars,
                                                      unsigned int &extendedLinkIndex, set<string>& allNewMinedPatternsCurTask, bool &notOutPutPattern, bool &patternAlreadyExtractedInCurTask,bool startFromLinkContainWhiteKeyword);

    void findAllInstancesForGivenPatternInNestedAtomSpace(HTreeNode* HNode);

    void findAllInstancesForGivenPatternBF(HTreeNode* HNode);

    void growTheFirstGramPatternsTaskBF();

    void ConstructTheFirstGramPatternsBF();

    void growPatternsTaskBF();

    void GrowAllPatternsBF();

    void growPatternsDepthFirstTask(unsigned int thread_index);

    void evaluateInterestingnessTask();

	/**
	 * TODO Add comment + utest
	 */
    void generateNextCombinationGroup(bool* &indexes, int n_max);

    bool isLastNElementsAllTrue(bool* array, int size, int n);

    bool isInHandleSeq(const Handle& handle, const HandleSeq& handles);

    bool isInHandleSeqSeq(const Handle& handle, const HandleSeqSeq& handleSeqs);

    bool containDuplicates(const HandleSeq& handles);

    Handle getFirstNonIgnoredIncomingLink(AtomSpace& atomspace, const Handle& handle);

    bool isIgnoredType(Type type);

    bool isTypeInList(Type type, const vector<Type> &typeList);

    // if atomspace = nullptr, it will use the pattern mining Atomspace
    std::string Link2keyString(const Handle& link, const string& indent=string());

    HandleSet _getAllNonIgnoredLinksForGivenNode(Handle keywordNode, const HandleSet& allSubsetLinks);

    HandleSet _extendOneLinkForSubsetCorpus(const HandleSet& allNewLinksLastGram, HandleSet& allSubsetLinks, HandleSet& extractedNodes);

    // will write the subset to a scm file
    void _selectSubsetFromCorpus(const set<string>& subsetKeywords, unsigned int max_connection, bool logic_contain=true);

    void findAllLinksContainKeyWords(const set<string>& subsetKeywords, unsigned int max_connection, bool logic_contain, HandleSet& foundLinks);

    bool isIgnoredContent(const string& keyword);

    bool containIgnoredContent(Handle link);

    bool doesLinkContainNodesInKeyWordNodes(const Handle& link, const HandleSet& keywordNodes);

    /**
     * Partition links into strongly connected components, where each
     * link is connected to the other w.r.t. whether they shares some
     * pattern variables.
     *
     * Write the results into connectedGroups. Return true iff there
     * more than 1 component.
     */
    bool partitionBySharedVariables(const HandleSeq& links, HandleSeqSeq& connectedGroups);

    double calculateEntropyOfASubConnectedPattern(string& connectedSubPatternKey, HandleSeq& connectedSubPattern);

    void calculateInteractionInformation(HTreeNode* HNode);

    void generateComponentCombinations(string componentsStr, vector<vector<vector<unsigned int>>> &componentCombinations);

    unsigned int getCountOfAConnectedPattern(const string& connectedPatternKey, const HandleSeq& connectedPattern);

    unsigned int getAllEntityCountWithSamePredicatesForAPattern(const HandleSeq& pattern);

    void calculateSurprisingness(HTreeNode* HNode);

    void calculateTypeBSurprisingness(HTreeNode* HNode);

    // TODO: not used
    void getOneMoreGramExtendedLinksFromGivenLeaf(Handle& toBeExtendedLink, Handle& leaf, Handle& varNode,
                                                  HandleSeq& outPutExtendedPatternLinks, AtomSpace& from_as);

    bool isALinkOneInstanceOfGivenPattern(const Handle &instanceLink, const Handle& patternLink);

    void reNameNodesForALink(const Handle& inputLink, const Handle& nodeToBeRenamed,
                             Handle& newNamedNode, HandleSeq& renameOutgoingLinks,
                             AtomSpace& to_as);

    bool filters(const HandleSeq& inputLinks, HandleSeqSeq& oneOfEachSeqShouldBeVars, HandleSeq& leaves, HandleSeq& shouldNotBeVars, HandleSeq& shouldBeVars);

    bool containWhiteKeywords(const string& str, QUERY_LOGIC logic);

    bool containKeywords(const string& str, const set<string>& keywords, QUERY_LOGIC logic);

    void initPatternMiner();

    void cleanUpPatternMiner();

    bool loadOutgoingsIntoAtomSpaceFromString(stringstream &outgoingStream, AtomSpace& _as, HandleSeq &outgoings, string parentIndent="");
    bool loadOutgoingsIntoAtomSpaceFromAtomString(stringstream& outgoingStream, AtomSpace& _as, HandleSeq &outgoings, string parentIndent="");
    HandleSeq loadPatternIntoAtomSpaceFromString(string patternStr, AtomSpace& _as);// input string is pattern keystring
    HandleSeq loadPatternIntoAtomSpaceFromFileString(string patternStr, AtomSpace& _as); // input string is normal atom string

public:
    PatternMiner(AtomSpace& _original_as);
    ~PatternMiner();

    bool checkPatternExist(const string& patternKeyStr);

    string unifiedPatternToKeyString(const HandleSeq& inputPattern);

    void OutPutFrequentPatternsToFile(unsigned int n_gram, const vector<vector<HTreeNode*>>& _patternsForGram, const string& _fileNamebasic=string());

    void OutPutStaticsToCsvFile(unsigned int n_gram);

    void OutPutLowFrequencyHighSurprisingnessPatternsToFile(const vector<HTreeNode*>& patternsForThisGram, unsigned int n_gram, unsigned int max_frequency_index);

    void OutPutHighFrequencyHighSurprisingnessPatternsToFile(const vector<HTreeNode*>& patternsForThisGram, unsigned int n_gram, unsigned int min_frequency_index);

    void OutPutHighSurprisingILowSurprisingnessIIPatternsToFile(const vector<HTreeNode*>& patternsForThisGram, unsigned int n_gram, float min_surprisingness_I, float max_surprisingness_II);

    void OutPutInterestingPatternsToFile(const vector<HTreeNode*>& patternsForThisGram, unsigned int n_gram, int surprisingness, const string& _fileNamebasic=string());

    void OutPutSurpringnessBToFile(const vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram);

    void OutPutFinalPatternsToFile(unsigned int n_gram);

    void OutPutAllEntityNumsToFile();

    void queryPatternsWithFrequencySurprisingnessIRanges(unsigned int min_frequency, unsigned int max_frequency, float min_surprisingness_I, float max_surprisingness_I, int gram);

    void queryPatternsWithSurprisingnessIAndIIRanges(unsigned int min_frequency, unsigned int max_frequency,
                                                                   float min_surprisingness_I, float max_surprisingness_I,
                                                                   float min_surprisingness_II, float max_surprisingness_II,int gram);

    void queryPatternsWithFrequencySurprisingnessBRanges(unsigned int min_frequency, unsigned int max_frequency,
                                                                   float min_surprisingness_B, float max_surprisingness_B,
                                                                   unsigned int min_subpattern_num, unsigned int max_subpattern_num,int gram);

    void queryPatternsWithFrequencyAndInteractionInformationRanges(unsigned int min_frequency, unsigned int max_frequency,
                                                                   float min_ii, float max_ii, int gram);

    AtomSpace* getResultAtomSpace() {return as;}

    void runPatternMiner(bool exit_program_after_finish=true);

    void runPatternMinerBreadthFirst();

    void runPatternMinerDepthFirst();

    void runInterestingnessEvaluation();

    void selectSubsetFromCorpus(const set<string> &topics, unsigned int gram, bool if_contian_logic=true);

    void selectSubsetAllEntityLinksContainsKeywords(set<string>& subsetKeywords);

    void loandAllDBpediaKeyNodes();

    void selectSubsetForDBpedia();

    vector<HTreeNode*>& getFinalPatternsForGram(unsigned int gram) {
        return finalPatternsForGram[gram - 1];
    }
    const vector<HTreeNode*>& getFinalPatternsForGram(unsigned int gram) const {
        return finalPatternsForGram[gram - 1];
    }

    void loadPatternsFromResultFile(string fileName);

    void testPatternMatcher();

public:
    void applyWhiteListKeywordfilterAfterMining();

    void resetPatternMiner(bool resetAllSettingsFromConfig);
};

}
}

#endif //_OPENCOG_PATTERNMINER_PATTERNMINER_H
