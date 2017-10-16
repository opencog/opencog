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

#include "HTree.h"

using namespace std;


namespace opencog
{
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
    vector<vector<std::pair<int,std::size_t>>> indexesOfSharedVars;

    bool operator <(const _non_ordered_pattern& other) const
    {
        // NTODO replace by range loop
        for (unsigned int i = 0; i < indexesOfSharedVars.size(); ++ i)
        {
            if (indexesOfSharedVars[i].size() < other.indexesOfSharedVars[i].size())
                return true;
            else if (indexesOfSharedVars[i].size() > other.indexesOfSharedVars[i].size())
                return false;

            for (unsigned int j = 0; j < indexesOfSharedVars[i].size(); ++ j)
            {
                if ((indexesOfSharedVars[i][j]).first< (other.indexesOfSharedVars[i][j]).first)
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
        cout << "\n warning: _non_ordered_pattern: Fail to figure out the order of  two patterns!\n";
        return true;
    }
};


struct advanced_non_ordered_pattern // only when complex patterns like pln patterns, used when enable_unify_unordered_links = true
{
    Handle link;
    vector<vector<std::pair<int,std::size_t>>> indexesOfSharedVars; // the shared vars indexes appear in the already sorted links


    bool operator <(const _non_ordered_pattern& other) const
    {
        // first, use indexesOfSharedVars
        // NTODO replace by range loop
        for (unsigned int i = 0; i < indexesOfSharedVars.size(); ++ i)
        {
            if (indexesOfSharedVars[i].size() < other.indexesOfSharedVars[i].size())
                return true;
            else if (indexesOfSharedVars[i].size() > other.indexesOfSharedVars[i].size())
                return false;

            for (unsigned int j = 0; j < indexesOfSharedVars[i].size(); ++ j)
            {
                if ((indexesOfSharedVars[i][j]).first< (other.indexesOfSharedVars[i][j]).first)
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
        cout << "\n warning: _non_ordered_pattern: Fail to figure out the order of  two patterns!\n";
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

enum QUERY_LOGIC
{
    OR,
    AND
};

class PatternMiner
{
protected:

    HTree* htree;
    AtomSpace* as;              // NTODO what is this used for?
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

    unsigned int THREAD_NUM;

    unsigned int MAX_GRAM;

    string Pattern_mining_mode;

    bool is_distributed;

    unsigned int cur_gram;

    double max_var_num_percent;

    int cur_index;

    int allLinkNumber;

    unsigned int linksPerThread;

    float last_gram_total_float;

    bool enable_filter_leaves_should_not_be_vars;
    bool enable_filter_links_should_connect_by_vars;
    bool enable_filter_links_of_same_type_not_share_second_outgoing;
    bool enable_filter_not_same_var_from_same_predicate;
    bool enable_filter_not_all_first_outgoing_const;
    bool enable_filter_first_outgoing_evallink_should_be_var;
    bool enable_filter_node_types_should_not_be_vars;
    vector<Type> node_types_should_not_be_vars;

    bool enable_filter_node_types_should_be_vars;
    vector<Type> node_types_should_be_vars;

    vector<Type> same_link_types_not_share_second_outgoing;

    unsigned int num_of_patterns_without_superpattern_cur_gram;
    unsigned int *num_of_patterns_with_1_frequency;

    unsigned int thresholdFrequency;

    std::mutex uniqueKeyLock, patternForLastGramLock, removeAtomLock, patternMatcherLock, addNewPatternLock, calculateIILock,
        readNextLinkLock,actualProcessedLinkLock, curDFExtractedLinksLock, readNextPatternLock, threadExtractedLinksLock;

    bool use_keyword_white_list;
    bool use_keyword_black_list;

    // = true will filter out atoms with labels contain keyword, = fasle will only filter out atoms with labels equal to any keyword
    bool keyword_black_logic_is_contain;
    HandleSet black_keyword_Handles; // only use when keyword_black_logic_is_contain = false

    QUERY_LOGIC keyword_white_list_logic;

    bool use_linktype_black_list;
    bool use_linktype_white_list;

    vector<Type> linktype_black_list;
    vector<Type> linktype_white_list;

//    Handle FrequencyHandle;
//    Handle InteractionInformationHandle;
//    Handle SurprisingnessIHandle;
//    Handle SurprisingnessIIHandle;
    Handle PatternValuesHandle;

    bool if_quote_output_pattern;
    Type output_pattern_quoted_linktype;

    bool calculate_type_b_surprisingness;

    bool enable_Interesting_Pattern;

    // Only effective when Enable_Interesting_Pattern is true.
    bool Enable_Interaction_Information;
    bool Enable_surprisingness;

    bool enable_unify_unordered_links; // if the corpus contains unordered Links like AND_LINK

    bool only_mine_patterns_start_from_white_list;
    bool only_mine_patterns_start_from_white_list_contain;

    bool only_output_patterns_contains_white_keywords;

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

    // This is against graph isomorphism problem, make sure the
    // patterns we found are not duplicacted. The input links should
    // be a Pattern in such format:
    //
    //    (InheritanceLink
    //       (VariableNode "$1")
    //       (ConceptNode "Animal")
    //
    //    (InheritanceLink
    //       (VariableNode "$2")
    //       (VariableNode "$1")
    //
    //    (InheritanceLink
    //       (VariableNode "$3")
    //       (VariableNode "$2")
    //
    //    (EvaluationLink (stv 1 1)
    //       (PredicateNode "like_food")
    //       (ListLink
    //          (VariableNode "$3")
    //          (ConceptNode "meat")
    //       )
    //    )
    //
    // Return unified ordered Handle vector
    HandleSeq _UnifyPatternOrder(HandleSeq& inputPattern, unsigned int& unifiedLastLinkIndex);
    HandleSeq UnifyPatternOrder(HandleSeq& inputPattern, unsigned int& unifiedLastLinkIndex, HandleMap& orderedVarNameMap);

    Handle UnifyOneLinkForUnorderedLink(const Handle& link,std::map<Handle,Type>& orderedTmpLinkToType);


    Handle rebindLinkTypeRecursively(const Handle& inputLink, std::map<Handle,Type>& orderedTmpLinkToType);

    void addAtomTypesFromString(string node_types_str, vector<Type>& typeListToAddTo);

    // Traverses link, if encouter a pattern variable not in
    // varNameMap, create a name for it and insert a pair {old
    // variable, new variable} in varNameMap. Produce a new outgoing
    // set of link with all new variables and return it.
    //
    // orderedTmpLinkToType isn't used at the moment.
    HandleSeq findAndRenameVariables(const Handle& link, HandleMap& varNameMap,
                                     std::map<Handle,Type>& orderedTmpLinkToType);

    // Rename the variable names in a ordered pattern according to the orders of the variables appear in the orderedPattern
    HandleSeq RebindVariableNames(HandleSeq& orderedPattern, HandleMap& orderedVarNameMap, std::map<Handle,Type>& orderedTmpLinkToType);

    void ReplaceConstNodeWithVariableForOneLink(Handle link, Handle constNode, Handle newVariableNode, HandleSeq& renameOutgoingLinks);

    HandleSeq ReplaceConstNodeWithVariableForAPattern(const HandleSeq& pattern, Handle constNode, Handle newVariableNode);

    void generateIndexesOfSharedVars(const Handle& link, const HandleSeq& orderedHandles, vector<vector<std::pair<int, size_t>>> &indexes);

    // generate the outgoings for a link in a pattern in the Pattern mining Atomspace, according to the given group of variables
    void generateALinkByChosenVariables(const Handle &originalLink, HandleMap& valueToVarMap, HandleSeq &outputOutgoings);

    // valueToVarMap: the ground value node in the orginal Atomspace
    // to the variable handle in pattenmining Atomspace
    void extractAllNodesInLink(const Handle& link, HandleMap& valueToVarMap);
    void extractAllNodesInLink(Handle link, HandleSet& allNodes);
    void extractAllNodesInLink(Handle link, map<Handle, unsigned int> &allNodes, unsigned index); // just find all the nodes in the original atomspace for this link
    void extractAllVariableNodesInLink(Handle link, HandleSet& allNodes);
    void extractAllConstNodesInALink(Handle link, HandleSet& allConstNodes);

    // if a link contains only variableNodes , no const nodes
    bool onlyContainVariableNodes(Handle link);

    bool containVariableNodes(Handle link);

    void extractAllPossiblePatternsFromInputLinksBF(const HandleSeq& inputLinks, HTreeNode* parentNode, HandleSet& sharedNodes, unsigned int gram);

//    // vector<HTreeNode *> &allHTreeNodes is output all the HTreeNodes found
//    void extractAllPossiblePatternsFromInputLinksDF(HandleSeq& inputLinks,unsigned int sharedLinkIndex, AtomSpace& from_as,
//                                                    vector<HTreeNode*>& allLastGramHTreeNodes, vector<HTreeNode*>& allHTreeNodes, unsigned int gram=1);

	// Copy the outgoings of `link` to `to_as` and return the copies
	//
	// Pattern variables are turned into regular variables while being
	// copied, filling `variables`.
    HandleSeq copyOutgoings(AtomSpace& to_as, const Handle& link,
                            HandleSeq& variables);

	// Copy `h` to `to_as` and return the copy.
	//
	// Pattern variables are turned into regular variables while being
	// copied, filling `variables`.
	Handle copyAtom(AtomSpace& to_as, const Handle& link,
	                HandleSeq& variables);

	// Copy `links` to `to_as` and return the copies.
	//
	// Pattern variables are turned into regular variables while being
	// copied, filling `variables`.
	//
	// NTODO: what guaranties that they are links?
    HandleSeq copyLinks(AtomSpace& to_as, const HandleSeq& links,
                        HandleSeq &variables);

    void swapOneLinkBetweenTwoAtomSpaceForBindLink(AtomSpace& to_as, const Handle& fromLink, HandleSeq& outgoings,
                                          HandleSeq &outVariableNodes, HandleSeq& linksWillBeDel, bool& containVar);

    HandleSeq swapLinksBetweenTwoAtomSpaceForBindLink(AtomSpace& to_as, const HandleSeq& fromLinks, HandleSeq& outVariableNodes, HandleSeq& linksWillBeDel);

    void extractAllVariableNodesInAnInstanceLink(const Handle& instanceLink, const Handle& patternLink, HandleSet& allVarNodes);

    void extractAllVariableNodesInAnInstanceLink(const Handle& instanceLink, const Handle& patternLink, map<Handle, unsigned int>& allVarNodes, unsigned index);

    // Unused
    // void extendAllPossiblePatternsForOneMoreGramDF(const HandleSeq& instance, AtomSpace& from_as, unsigned int gram,
    //                                                vector<HTreeNode*>& allLastGramHTreeNodes, map<HandleSeq, vector<HTreeNode*>>& allFactLinksToPatterns, vector<HandleSet>& newConnectedLinksFoundThisGram);

    void extendAllPossiblePatternsForOneMoreGramBF(const HandleSeq& instance, HTreeNode* curHTreeNode, unsigned int gram);

    //  void extendAllPossiblePatternsTillMaxGramDF(Handle &startLink, AtomSpace& from_as, unsigned int max_gram);
    void addThreadExtractedLinks(unsigned int _gram, unsigned int cur_thread_index, string _extractedLinkUIDs);

    bool existInAllThreadExtractedLinks(unsigned int _gram, string _extractedLinkUIDs);

    bool existInOneThreadExtractedLinks(unsigned int _gram, unsigned int cur_thread_index, string _extractedLinkUIDs);

    void extendAPatternForOneMoreGramRecursively(const Handle& extendedLink, AtomSpace& from_as, const Handle& extendedNode, const HandleSeq& lastGramLinks,
                                                 HTreeNode* parentNode, const HandleMap& lastGramValueToVarMap, const HandleMap& lastGramPatternVarMap,
                                                 bool isExtendedFromVar, set<string>& allNewMinedPatternsCurTask, vector<HTreeNode*>& allHTreeNodesCurTask,
                                                 vector<MinedPatternInfo>& allNewMinedPatternInfo, unsigned int thread_index, bool startFromLinkContainWhiteKeyword);

    bool containsLoopVariable(const HandleSeq& inputPattern);

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

    // void growPatternsDepthFirstTask_old();

    void growPatternsDepthFirstTask(unsigned int thread_index);

    void evaluateInterestingnessTask();

    void generateNextCombinationGroup(bool* &indexes, int n_max);

    bool isLastNElementsAllTrue(bool* array, int size, int n);

    bool isInHandleSeq(const Handle& handle, const HandleSeq& handles);

    bool isInHandleSeqSeq(const Handle& handle, const HandleSeqSeq& handleSeqs);

    bool containsDuplicateHandle(HandleSeq& handles);

    Handle getFirstNonIgnoredIncomingLink(AtomSpace& atomspace, const Handle& handle);

    bool isIgnoredType(Type type);

    bool isTypeInList(Type type, const vector<Type> &typeList);

    // if atomspace = nullptr, it will use the pattern mining Atomspace
    std::string Link2keyString(const Handle& link, const string& indent=string());

    HandleSet _getAllNonIgnoredLinksForGivenNode(Handle keywordNode, const HandleSet& allSubsetLinks);

    HandleSet _extendOneLinkForSubsetCorpus(const HandleSet& allNewLinksLastGram, HandleSet& allSubsetLinks, HandleSet& extractedNodes);

    // will write the subset to a scm file
    void _selectSubsetFromCorpus(const vector<string>& subsetKeywords, unsigned int max_connection, bool logic_contain=true);

    void findAllLinksContainKeyWords(const vector<string>& subsetKeywords, unsigned int max_connection, bool logic_contain, HandleSet& foundLinks);

    bool isIgnoredContent(const string& keyword);

    bool containIgnoredContent(Handle link);

    bool doesLinkContainNodesInKeyWordNodes(const Handle& link, const HandleSet& keywordNodes);

    vector<string> keyword_black_list;
    vector<string> keyword_white_list;

    bool splitDisconnectedLinksIntoConnectedGroups(const HandleSeq& inputLinks, HandleSeqSeq& outputConnectedGroups);

    double calculateEntropyOfASubConnectedPattern(string& connectedSubPatternKey, HandleSeq& connectedSubPattern);

    void calculateInteractionInformation(HTreeNode* HNode);

    void generateComponentCombinations(string componentsStr, vector<vector<vector<unsigned int>>> &componentCombinations);

    unsigned int getCountOfAConnectedPattern(const string& connectedPatternKey, const HandleSeq& connectedPattern);

    unsigned int getAllEntityCountWithSamePredicatesForAPattern(const HandleSeq& pattern);

    void calculateSurprisingness(HTreeNode* HNode);

    void calculateTypeBSurprisingness(HTreeNode* HNode);

	// NTODO: not used
    void getOneMoreGramExtendedLinksFromGivenLeaf(Handle& toBeExtendedLink, Handle& leaf, Handle& varNode,
                                                  HandleSeq& outPutExtendedPatternLinks, AtomSpace& from_as);

    bool isALinkOneInstanceOfGivenPattern(const Handle &instanceLink, const Handle& patternLink);

    void reNameNodesForALink(const Handle& inputLink, const Handle& nodeToBeRenamed,
                             Handle& newNamedNode, HandleSeq& renameOutgoingLinks,
                             AtomSpace& to_as);

    bool filters(const HandleSeq& inputLinks, HandleSeqSeq& oneOfEachSeqShouldBeVars, HandleSeq& leaves, HandleSeq& shouldNotBeVars, HandleSeq& shouldBeVars);

    bool containWhiteKeywords(const string& str, QUERY_LOGIC logic);

    bool containKeywords(const string& str, const vector<string>& keywords, QUERY_LOGIC logic);

    void reSetAllSettingsFromConfig();

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

    void selectSubsetFromCorpus(const vector<string> &topics, unsigned int gram, bool if_contian_logic=true);

    void selectSubsetAllEntityLinksContainsKeywords(vector<string>& subsetKeywords);

    void loandAllDBpediaKeyNodes();

    void selectSubsetForDBpedia();

    vector<HTreeNode*>&  getFinalPatternsForGram(unsigned int gram){ return finalPatternsForGram[gram - 1];}

    void loadPatternsFromResultFile(string fileName);

    void testPatternMatcher();


public:
    // -------------------------------basic settings----------------------

    unsigned int get_Pattern_Max_Gram(){return MAX_GRAM;}
    void set_Pattern_Max_Gram(unsigned int _max_gram){ MAX_GRAM = _max_gram;}

    bool get_Enable_Interesting_Pattern(){return enable_Interesting_Pattern;}
    void set_Enable_Interesting_Pattern(bool _enable){enable_Interesting_Pattern = _enable;}

    unsigned int get_Frequency_threshold() {return thresholdFrequency;}
    void set_Frequency_threshold(unsigned int _Frequency_threshold) {thresholdFrequency = _Frequency_threshold;}

    // -------------------------------end basic settings----------------------

    // -------------------------------filter settings----------------------
    bool get_use_keyword_black_list(){return use_keyword_black_list;}
    void set_use_keyword_black_list(bool _use){use_keyword_black_list = _use;}

    bool get_use_keyword_white_list(){return use_keyword_white_list;}
    void set_use_keyword_white_list(bool _use){use_keyword_white_list = _use;}

    bool get_use_linktype_black_list(){return use_linktype_black_list;}
    void set_use_linktype_black_list(bool _use){use_linktype_black_list = _use;}

    bool get_use_linktype_white_list(){return use_linktype_white_list;}
    void set_use_linktype_white_list(bool _use){use_linktype_white_list = _use;}

    vector<Type> get_linktype_white_list(){return linktype_white_list;}
    bool add_linktype_to_white_list(Type _type);
    bool remove_linktype_from_white_list(Type _type);

    vector<Type> get_Ignore_Link_Types(){return linktype_black_list;}
    bool add_Ignore_Link_Type(Type _type);
    bool remove_Ignore_Link_Type(Type _type);

    vector<string> get_keyword_black_list(){return keyword_black_list;}
    bool add_keyword_to_black_list(string _keyword);
    bool remove_keyword_from_black_list(string _keyword);
    void clear_keyword_black_list(){keyword_black_list.clear();}

    vector<string> get_keyword_white_list(){return keyword_white_list;}
    bool add_keyword_to_white_list(string _keyword);
    bool remove_keyword_from_white_list(string _keyword);
    void clear_keyword_white_list(){keyword_white_list.clear();}

    QUERY_LOGIC get_keyword_white_list_logic(){return keyword_white_list_logic;}
    void set_keyword_white_list_logic(QUERY_LOGIC logic){keyword_white_list_logic = logic;}

    void set_enable_filter_links_of_same_type_not_share_second_outgoing(bool _enable){enable_filter_links_of_same_type_not_share_second_outgoing = _enable;}
    bool get_enable_filter_links_of_same_type_not_share_second_outgoing(){return enable_filter_links_of_same_type_not_share_second_outgoing;}
    vector<Type> get_same_link_types_not_share_second_outgoing(){return same_link_types_not_share_second_outgoing;}
    bool add_link_type_to_same_link_types_not_share_second_outgoing(Type _type);
    bool remove_link_type_from_same_link_types_not_share_second_outgoing(Type _type);
    void clear_same_link_types_not_share_second_outgoing(){same_link_types_not_share_second_outgoing.clear();}

    void set_enable_filter_node_types_should_not_be_vars(bool _enable){enable_filter_node_types_should_not_be_vars=_enable;}
    bool get_enable_filter_node_types_should_not_be_vars(){return enable_filter_node_types_should_not_be_vars;}
    vector<Type> get_node_types_should_not_be_vars(){return node_types_should_not_be_vars;}
    bool add_node_type_to_node_types_should_not_be_vars(Type _type);
    bool remove_node_type_from_node_types_should_not_be_vars(Type _type);
    void clear_node_types_should_not_be_vars(){node_types_should_not_be_vars.clear();}

    void set_enable_filter_node_types_should_be_vars(bool _enable){enable_filter_node_types_should_be_vars = _enable;}
    bool get_enable_filter_node_types_should_be_vars(){return enable_filter_node_types_should_be_vars;}
    vector<Type> get_node_types_should_be_vars(){return node_types_should_be_vars;}
    bool add_node_type_to_node_types_should_be_vars(Type _type);
    bool remove_node_type_from_node_types_should_be_vars(Type _type);
    void clear_node_types_should_be_vars(){node_types_should_be_vars.clear();}

    // -------------------------------end filter settings----------------------

    void applyWhiteListKeywordfilterAfterMining();

    void resetPatternMiner(bool resetAllSettingsFromConfig);

};

}
}

#endif //_OPENCOG_PATTERNMINER_PATTERNMINER_H
