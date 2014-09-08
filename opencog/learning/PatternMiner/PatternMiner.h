/*
 * opencog/learning/PatternMiner/PatternMiner.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com>
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

using namespace std;

namespace opencog
{
namespace PatternMining
{
#define FLOAT_MIN_DIFF 0.00001

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

     HandleSeq allLinks;// all links in the orginal atomspace

     // Every pattern is reprented as a unique string as the key in this map, mapping to its cooresponding HTreeNode
     map<string, HTreeNode*> keyStrToHTreeNodeMap;

     vector < vector<HTreeNode*> > patternsForGram;

     std::thread *threads;

     unsigned int THREAD_NUM;

     unsigned int MAX_GRAM;

     unsigned int cur_gram;

     unsigned int cur_index;

     float last_gram_total_float;

     unsigned int thresholdFrequency; // patterns with a frequency lower than thresholdFrequency will be neglected, not grow next gram pattern from them

     std::mutex uniqueKeyLock, patternForLastGramLock, removeAtomLock, patternMatcherLock, addNewPatternLock, calculateIILock;

     Type ignoredTypes[1];

     bool enable_Frequent_Pattern;
     bool enable_Interesting_Pattern;

     // Only effective when Enable_Interesting_Pattern is true. The options are "Interaction_Information", "surprisingness"
     string interestingness_Evaluation_method;

     float atomspaceSizeFloat;

     vector<vector<vector<unsigned int>>> components_ngram[3];

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
     vector<Handle> UnifyPatternOrder(vector<Handle>& inputPattern);

     string unifiedPatternToKeyString(vector<Handle>& inputPattern , const AtomSpace *atomspace = 0);

     // this function is called by RebindVariableNames
     void findAndRenameVariablesForOneLink(Handle link, map<Handle,Handle>& varNameMap, HandleSeq& renameOutgoingLinks);

     // rename the variable names in a ordered pattern according to the orders of the variables appear in the orderedPattern
     vector<Handle> RebindVariableNames(vector<Handle>& orderedPattern, map<Handle, Handle> &orderedVarNameMap);

     void generateIndexesOfSharedVars(Handle& link, vector<Handle>& orderedHandles, vector< vector<int> > &indexes);

     // generate the outgoings for a link in a pattern in the Pattern mining Atomspace, according to the given group of variables
     void generateALinkByChosenVariables(Handle &originalLink, map<Handle,Handle>& valueToVarMap, HandleSeq &outputOutgoings);

      // valueToVarMap:  the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace
     void extractAllNodesInLink(Handle link, map<Handle,Handle>& valueToVarMap);
     void extractAllNodesInLink(Handle link, set<Handle>& allNodes); // just find all the nodes in the original atomspace for this link
     void extractAllVariableNodesInLink(Handle link, set<Handle>& allNodes, AtomSpace* _atomSpace);

     // if a link contains only variableNodes , no const nodes
     bool onlyContainVariableNodes(Handle link, AtomSpace* _atomSpace);

     void extractAllPossiblePatternsFromInputLinks(vector<Handle>& inputLinks, HTreeNode* parentNode, set<Handle> &sharedNodes, unsigned int gram = 1);

     void swapOneLinkBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, Handle& fromLink, HandleSeq& outgoings, HandleSeq &outVariableNodes, HandleSeq& linksWillBeDel, bool &containVar);

     // Generate the links in toAtomSpace the same as the fromLinks in the fromAtomSpace. Return the swapped links in the toAtomSpace.
     // Output all the variable nodes in the toAtomSpace BTW
     HandleSeq swapLinksBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, HandleSeq& fromLinks, HandleSeq &outVariableNodes, HandleSeq &linksWillBeDel);

     void extractAllVariableNodesInAnInstanceLink(Handle& instanceLink, Handle& patternLink, set<Handle>& allVarNodes);

     void extendAllPossiblePatternsForOneMoreGram(HandleSeq &instance, HTreeNode* curHTreeNode, unsigned int gram);

     void findAllInstancesForGivenPattern(HTreeNode* HNode);

     void growTheFirstGramPatternsTask();

     void ConstructTheFirstGramPatterns();

     void growPatternsTask();

     void GrowAllPatterns();

     void reportProgress();

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

     unsigned int getCountOfASubConnectedPattern(string& connectedSubPatternKey, HandleSeq& connectedSubPattern);

     void calculateSurprisingness( HTreeNode* HNode);

 public:
     PatternMiner(AtomSpace* _originalAtomSpace, unsigned int max_gram = 3);
     ~PatternMiner();

     bool checkPatternExist(const string& patternKeyStr);

     void OutPutPatternsToFile(unsigned int n_gram, bool is_interesting_pattern = false);

     void runPatternMiner(unsigned int _thresholdFrequency = 2);

     void selectSubsetFromCorpus(vector<string> &topics, unsigned int gram = 2);

     void testPatternMatcher1();
     void testPatternMatcher2();



  };

}
}

#endif //_OPENCOG_PATTERNMINER_PATTERNMINER_H
