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
#include <map>
#include <vector>
#include "Pattern.h"
#include "HTree.h"
#include <cstdio>
#include <opencog/atomspace/AtomSpace.h>
#include <thread>
#include <mutex>

#include <cpprest/http_listener.h>
#include <cpprest/http_client.h>

using namespace std;
using namespace web;
using namespace web::http;
using namespace web::http::client;
using namespace web::http::experimental::listener;

namespace opencog
{
namespace PatternMining
{
#define FLOAT_MIN_DIFF 0.00001
#define SURPRISINGNESS_I_TOP_THRESHOLD 0.20
#define SURPRISINGNESS_II_TOP_THRESHOLD 0.40

#define LINE_INDENTATION "  "

#define JSON_BUF_MAX_NUM 30

 struct _non_ordered_pattern
 {
     Handle link;
     vector< vector<int> > indexesOfSharedVars;

     bool operator <(const _non_ordered_pattern& other) const
     {

         for (unsigned int i = 0; i < indexesOfSharedVars.size(); ++ i)
         {
             if (indexesOfSharedVars[i].size() < other.indexesOfSharedVars[i].size())
                 return true;
             else if (indexesOfSharedVars[i].size() > other.indexesOfSharedVars[i].size())
                 return false;

             for (unsigned int j = 0; j < indexesOfSharedVars[i].size(); ++ j)
             {
                 if (indexesOfSharedVars[i][j]< other.indexesOfSharedVars[i][j])
                     return true;
                 else if (indexesOfSharedVars[i][j] > other.indexesOfSharedVars[i][j])
                     return false;
             }
         }

         // if all above criteria cannot figure out the order of these two patterns, just return true and output a warning
         cout << "\n warning: _non_ordered_pattern: Fail to figure out the order of  two patterns!\n";
         return true;
     }
 };


 class PatternMiner
 {
 private:

     HTree* htree;
     AtomSpace* atomSpace;
     AtomSpace* originalAtomSpace;
     AtomSpace* observingAtomSpace;

     HandleSeq allLinks;// all links in the orginal atomspace

     // Every pattern is reprented as a unique string as the key in this map, mapping to its cooresponding HTreeNode
     map<string, HTreeNode*> keyStrToHTreeNodeMap;

     vector < vector<HTreeNode*> > patternsForGram;
     vector < vector<HTreeNode*> > finalPatternsForGram;

     std::thread *threads;

     unsigned int THREAD_NUM;

     unsigned int MAX_GRAM;

     string Pattern_mining_mode;

     unsigned int cur_gram;

     int cur_index;

     int allLinkNumber;

     unsigned int linksPerThread;

     float last_gram_total_float;

     bool enable_filter_leaves_should_not_be_vars;
     bool enable_filter_links_should_connect_by_vars;
     bool enable_filter_not_inheritant_from_same_var;
     bool enable_filter_not_same_var_from_same_predicate;
     bool enable_filter_not_all_first_outgoing_const;
     bool enable_filter_first_outgoing_evallink_should_be_var;
     bool enable_filter_node_types_should_not_be_vars;
     vector<Type> node_types_should_not_be_vars;

     unsigned int num_of_patterns_without_superpattern_cur_gram;

     unsigned int thresholdFrequency; // patterns with a frequency lower than thresholdFrequency will be neglected, not grow next gram pattern from them

     std::mutex uniqueKeyLock, patternForLastGramLock, removeAtomLock, patternMatcherLock, addNewPatternLock, calculateIILock,
                readNextLinkLock,actualProcessedLinkLock, curDFExtractedLinksLock, readNextPatternLock;

     Type ignoredTypes[1];

     bool enable_Frequent_Pattern;
     bool enable_Interesting_Pattern;

     // Only effective when Enable_Interesting_Pattern is true. The options are "Interaction_Information", "surprisingness"
     string interestingness_Evaluation_method;

     float atomspaceSizeFloat;

     float surprisingness_II_threshold;

     //debug
     unsigned int processedLinkNum; // include those links of ignored types
     unsigned int actualProcessedLinkNum;

     vector<vector<vector<unsigned int>>> components_ngram[3];



//     // [gram], this to avoid different threads happen to work on the same links.
//     // each string is composed the handles of a group of fact links in the observingAtomSpace in the default hash order using std set
//     set<string>* cur_DF_ExtractedLinks;

     // this is to against graph isomorphism problem, make sure the patterns we found are not dupicacted
     // the input links should be a Pattern in such format:
     //    (InheritanceLink
     //       (VariableNode "$1")
     //       (ConceptNode "Animal")

     //    (InheritanceLink
     //       (VariableNode "$2")
     //       (VariableNode "$1")

     //    (InheritanceLink
     //       (VariableNode "$3")
     //       (VariableNode "$2")

     //    (EvaluationLink (stv 1 1)
     //       (PredicateNode "like_food")
     //       (ListLink
     //          (VariableNode "$3")
     //          (ConceptNode "meat")
     //       )
     //    )
     // Return unified ordered Handle vector
     vector<Handle> UnifyPatternOrder(vector<Handle>& inputPattern, unsigned int &unifiedLastLinkIndex);

     string unifiedPatternToKeyString(vector<Handle>& inputPattern , const AtomSpace *atomspace = 0);

     // this function is called by RebindVariableNames
     void findAndRenameVariablesForOneLink(Handle link, map<Handle,Handle>& varNameMap, HandleSeq& renameOutgoingLinks);

     // rename the variable names in a ordered pattern according to the orders of the variables appear in the orderedPattern
     vector<Handle> RebindVariableNames(vector<Handle>& orderedPattern, map<Handle,Handle>& orderedVarNameMap);

     void generateIndexesOfSharedVars(Handle& link, vector<Handle>& orderedHandles, vector< vector<int> > &indexes);

     // generate the outgoings for a link in a pattern in the Pattern mining Atomspace, according to the given group of variables
     void generateALinkByChosenVariables(Handle &originalLink, map<Handle,Handle>& valueToVarMap, HandleSeq &outputOutgoings, AtomSpace *_fromAtomSpace);

      // valueToVarMap:  the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace
     void extractAllNodesInLink(Handle link, map<Handle,Handle>& valueToVarMap, AtomSpace* _fromAtomSpace);
     void extractAllNodesInLink(Handle link, set<Handle>& allNodes, AtomSpace* _fromAtomSpace);
     void extractAllNodesInLink(Handle link, map<Handle, unsigned int> &allNodes, AtomSpace* _fromAtomSpace, unsigned index); // just find all the nodes in the original atomspace for this link
     void extractAllVariableNodesInLink(Handle link, set<Handle>& allNodes, AtomSpace* _atomSpace);

     // if a link contains only variableNodes , no const nodes
     bool onlyContainVariableNodes(Handle link, AtomSpace* _atomSpace);

     void extractAllPossiblePatternsFromInputLinksBF(vector<Handle>& inputLinks,  HTreeNode* parentNode,set<Handle>& sharedNodes, unsigned int gram);

     // vector<HTreeNode *> &allHTreeNodes is output all the HTreeNodes found
     void extractAllPossiblePatternsFromInputLinksDF(vector<Handle>& inputLinks,unsigned int sharedLinkIndex, AtomSpace* _fromAtomSpace,
                                                     vector<HTreeNode*>& allLastGramHTreeNodes, vector<HTreeNode*>& allHTreeNodes, unsigned int gram = 1);

     void swapOneLinkBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, Handle& fromLink, HandleSeq& outgoings, HandleSeq &outVariableNodes);

     // Generate the links in toAtomSpace the same as the fromLinks in the fromAtomSpace. Return the swapped links in the toAtomSpace.
     // Output all the variable nodes in the toAtomSpace BTW
     HandleSeq swapLinksBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, HandleSeq& fromLinks, HandleSeq &outVariableNodes);


     void swapOneLinkBetweenTwoAtomSpaceBF(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, Handle& fromLink, HandleSeq& outgoings,
                                                       HandleSeq &outVariableNodes, HandleSeq& linksWillBeDel, bool& containVar );

     HandleSeq swapLinksBetweenTwoAtomSpaceBF(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, HandleSeq& fromLinks, HandleSeq& outVariableNodes, HandleSeq& linksWillBeDel);

     void extractAllVariableNodesInAnInstanceLink(Handle& instanceLink, Handle& patternLink, set<Handle>& allVarNodes);

     void extractAllVariableNodesInAnInstanceLink(Handle& instanceLink, Handle& patternLink, map<Handle, unsigned int>& allVarNodes, unsigned index);

     void extendAllPossiblePatternsForOneMoreGramDF(HandleSeq &instance, AtomSpace* _fromAtomSpace, unsigned int gram,
          vector<HTreeNode*>& allLastGramHTreeNodes, map<HandleSeq, vector<HTreeNode*> >& allFactLinksToPatterns, vector<set<Handle>>& newConnectedLinksFoundThisGram);

     void extendAllPossiblePatternsForOneMoreGramBF(HandleSeq &instance, HTreeNode* curHTreeNode, unsigned int gram);

     //  void extendAllPossiblePatternsTillMaxGramDF(Handle &startLink, AtomSpace* _fromAtomSpace, unsigned int max_gram);

     void extendAPatternForOneMoreGramRecursively(const Handle &extendedLink, AtomSpace* _fromAtomSpace, const Handle &extendedNode, const HandleSeq &lastGramLinks,
                                     HTreeNode* parentNode, const map<Handle,Handle> &lastGramValueToVarMap, const map<Handle,Handle> &lastGramPatternVarMap,
                                     bool isExtendedFromVar, vector<HTreeNode*> &allHTreeNodesCurTask, web::json::value &patternJsonArray);

     bool containsLoopVariable(HandleSeq& inputPattern);

     HTreeNode* extractAPatternFromGivenVarCombination(HandleSeq &inputLinks, map<Handle,Handle> &patternVarMap, HandleSeqSeq &oneOfEachSeqShouldBeVars, HandleSeq &leaves,
                                                       HandleSeq &shouldNotBeVars, HandleSeq &shouldBeVars, AtomSpace *_fromAtomSpace, unsigned int &extendedLinkIndex);

     void findAllInstancesForGivenPatternInNestedAtomSpace(HTreeNode* HNode);

     void findAllInstancesForGivenPatternBF(HTreeNode* HNode);

     void growTheFirstGramPatternsTaskBF();

     void ConstructTheFirstGramPatternsBF();

     void growPatternsTaskBF();

     void GrowAllPatternsBF();

     void growPatternsDepthFirstTask_old();

     void growPatternsDepthFirstTask(unsigned int thread_index);

     void evaluateInterestingnessTask();

     void generateNextCombinationGroup(bool* &indexes, int n_max);

     bool isLastNElementsAllTrue(bool* array, int size, int n);

     bool isInHandleSeq(Handle handle, HandleSeq &handles);

     bool isInHandleSeqSeq(Handle handle, HandleSeqSeq &handleSeqs);

     bool containsDuplicateHandle(HandleSeq &handles);

     Handle getFirstNonIgnoredIncomingLink(AtomSpace *atomspace, Handle &handle);

     bool isIgnoredType(Type type);

     // if atomspace = 0, it will use the pattern mining Atomspace
     std::string Link2keyString(Handle& link, string indent = "", const AtomSpace *atomspace = 0);

     void removeLinkAndItsAllSubLinks(AtomSpace *_atomspace, Handle link);

     set<Handle> _getAllNonIgnoredLinksForGivenNode(Handle keywordNode, set<Handle>& allSubsetLinks);

     set<Handle> _extendOneLinkForSubsetCorpus(set<Handle>& allNewLinksLastGram, set<Handle>& allSubsetLinks);

     // will write the subset to a scm file
     void _selectSubsetFromCorpus(vector<string>& subsetKeywords, unsigned int max_connection);

     bool isIgnoredContent(string keyword);

     bool containIgnoredContent(Handle link );

     const static string ignoreKeyWords[];

     bool splitDisconnectedLinksIntoConnectedGroups(HandleSeq& inputLinks, HandleSeqSeq& outputConnectedGroups);

     double calculateEntropyOfASubConnectedPattern(string& connectedSubPatternKey, HandleSeq& connectedSubPattern);

     void calculateInteractionInformation(HTreeNode* HNode);

     void generateComponentCombinations(string componentsStr, vector<vector<vector<unsigned int>>> &componentCombinations);

     unsigned int getCountOfAConnectedPattern(string& connectedPatternKey, HandleSeq& connectedPattern);

     void calculateSurprisingness( HTreeNode* HNode, AtomSpace *_fromAtomSpace);

     void getOneMoreGramExtendedLinksFromGivenLeaf(Handle& toBeExtendedLink, Handle& leaf, Handle& varNode,
                                                                 HandleSeq& outPutExtendedPatternLinks, AtomSpace* _fromAtomSpace);

     bool isALinkOneInstanceOfGivenPattern(Handle &instanceLink, Handle& patternLink, AtomSpace* instanceLinkAtomSpace);

     void reNameNodesForALink(Handle& inputLink, Handle& nodeToBeRenamed, Handle& newNamedNode,HandleSeq& renameOutgoingLinks,
                                            AtomSpace* _fromAtomSpace, AtomSpace* _toAtomSpace);

     bool filters(HandleSeq& inputLinks, HandleSeqSeq& oneOfEachSeqShouldBeVars, HandleSeq& leaves, HandleSeq& shouldNotBeVars, HandleSeq& shouldBeVars,AtomSpace* _atomSpace);




 public:
     PatternMiner(AtomSpace* _originalAtomSpace);
     ~PatternMiner();

     bool checkPatternExist(const string& patternKeyStr);

     void OutPutFrequentPatternsToFile(unsigned int n_gram);

     void OutPutStaticsToCsvFile(unsigned int n_gram);

     void OutPutLowFrequencyHighSurprisingnessPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram);

     void OutPutHighFrequencyHighSurprisingnessPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram, unsigned int min_frequency);

     void OutPutHighSurprisingILowSurprisingnessIIPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram, float min_surprisingness_I, float max_surprisingness_II);

     void OutPutInterestingPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram, int surprisingness = 0);

     void OutPutFinalPatternsToFile(unsigned int n_gram);

     void runPatternMiner(unsigned int _thresholdFrequency = 2);

     void runPatternMinerBreadthFirst();

     void runPatternMinerDepthFirst();

     void selectSubsetFromCorpus(vector<string> &topics, unsigned int gram = 2);

     void testPatternMatcher1();
     void testPatternMatcher2();


     // ---------------start distributed version of pattern miner ---------------
private:
     string centralServerIP;
     string centralServerPort;
     string centralServerBaseURL;

     web::json::value *patternJsonArrays;

     unsigned int pattern_parse_thread_num; // for the central server
     std::thread centralServerListeningThread;
     std::thread *parsePatternTaskThreads;
     std::mutex patternQueueLock, addRelationLock, updatePatternCountLock, modifyWorkerLock;

     // map < uid, <is_still_working, processedFactsNum> >
     map<string, std::pair<bool, unsigned int> > allWorkers;
     bool allWorkersStop;

     list<json::value> waitForParsePatternQueue;

     string clientWorkerUID;

     bool run_as_distributed_worker;
     bool run_as_central_server;

     http_client* httpClient;
     http_listener* serverListener;

     int cur_worker_mined_pattern_num;
     int total_pattern_received; // in the server

     bool waitingForNewClients;


     void handlePost(http_request request);
     void handleRegisterNewWorker(http_request request);
     void handleReportWorkerStop(http_request request);
     void handleFindNewPatterns(http_request request);

     void runParsePatternTaskThread();
     void parseAPatternTask(json::value jval);

     bool checkIfAllWorkersStopWorking();

     void notifyServerThisWorkerStop();

     void startMiningWork();
     void centralServerEvaluateInterestingness();

     void addPatternsToJsonArrayBuf(string curPatternKeyStr, string parentKeyString,  unsigned int extendedLinkIndex, json::value &patternJsonArray);
     void sendPatternsToCentralServer(json::value &patternJsonArray);
     HandleSeq loadPatternIntoAtomSpaceFromString(string patternStr, AtomSpace* _atomSpace);
     bool loadOutgoingsIntoAtomSpaceFromString(stringstream &outgoingStream, AtomSpace *_atomSpace, HandleSeq &outgoings, string parentIndent = "");


 public:

     void launchADistributedWorker();
     void launchCentralServer();
     void centralServerStartListening();
     void centralServerStopListening();

     bool sendRequest(http_request &request, http_response &response);

     void startCentralServer();

     // ---------------end distributed version of pattern miner ---------------
  };

}
}

#endif //_OPENCOG_PATTERNMINER_PATTERNMINER_H
