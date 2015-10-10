/*
 * opencog/learning/PatternMiner/PatternMinerCentralServer.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com> in 2015
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

#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <sstream>
#include <thread>
#include <set>
#include <string>
#include <functional>


#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/StringManipulator.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>

#include "HTree.h"
#include "PatternMiner.h"

#include <cpprest/http_listener.h>
#include <cpprest/http_msg.h>
#include <cpprest/json.h>


using namespace opencog::PatternMining;
using namespace opencog;

using namespace std;
using namespace web;
using namespace web::http;
using namespace web::http::experimental::listener;
using namespace utility;


bool compareHTreeNodeByFrequency(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeByInteractionInformation(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeBySurprisingness(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeBySurprisingness_I(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeBySurprisingness_II(HTreeNode* node1, HTreeNode* node2);


void PatternMiner::launchCentralServer()
{
    run_as_central_server = true;

    centralServerPort = config().get("PMCentralServerPort");

    serverListener = new http_listener( utility::string_t("http://localhost:" + centralServerPort +"/PatternMinerServer") );

    serverListener->support(methods::POST, std::bind(&PatternMiner::handlePost, this,  std::placeholders::_1));

    patternMiningRunning = true;
    centralServerListeningThread = std::thread([this]{this->centralServerStartListening();});
    parsePatternTaskThread = std::thread([this]{this->runParsePatternTaskThread();});

    cout <<"\nPattern Miner central server started!\n";
    // centralServerListeningThread.join();

}

void PatternMiner::centralServerStartListening()
{
    try
    {
       total_pattern_received = 0;
       serverListener->open().wait();

    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
    }
}

void PatternMiner::handlePost(http_request request)
{
    try
    {
        // cout << "Got request: \n" << request.to_string() << std::endl;

        string path = request.relative_uri().path();
        if (path == "/RegisterNewWorker")
        {
            handleRegisterNewWorker(request);
        }
        else if (path == "/FindANewPattern")
        {
            handleFindANewPattern(request);
        }
        else if (path == "/ReportWorkerStop")
        {
            handleReportWorkerStop(request);
        }
        else
        {
            json::value answer = json::value::object();

            answer["msg"] = json::value("Unknown request command!");

            request.reply(status_codes::NotFound, answer);
        }
    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
       request.reply(status_codes::NotFound);
    }


}


void PatternMiner::handleRegisterNewWorker(http_request request)
{

    try
    {
        auto paths = uri::split_path(uri::decode(request.relative_uri().query()));

        string clientID =  paths[0];
        cout << "Got request to RegisterNewWorker: ClientID = \n" << clientID << std::endl;

        json::value answer = json::value::object();
        answer["msg"] = json::value("Worker registered successfully!");
        request.reply(status_codes::OK, answer);
    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
    }



}

void PatternMiner::handleReportWorkerStop(http_request request)
{
    // todo

}

void PatternMiner::handleFindANewPattern(http_request request)
{

    try
    {
        json::value jval = request.extract_json().get();
        patternQueueLock.lock();
        waitForParsePatternQueue.push_back(jval);
        total_pattern_received ++;
        patternQueueLock.unlock();
        request.reply(status_codes::OK);

        // cout << "Received pattern num = " << total_pattern_received << std::endl;
    }
    catch (exception const & e)
    {
       cout << e.what() << "handleFindANewPattern error: " << request.to_string() << endl;
       request.reply(status_codes::NotFound);
    }
}

void PatternMiner::runParsePatternTaskThread()
{
    while(patternMiningRunning)
    {

        patternQueueLock.lock();
        if (waitForParsePatternQueue.size() > 0)
        {
            json::value jval = waitForParsePatternQueue.front();
            waitForParsePatternQueue.pop_front();
            patternQueueLock.unlock();
            parseAPatternTask(jval);

        }
        else
            patternQueueLock.unlock();


    }
}

void PatternMiner::parseAPatternTask(json::value jval)
{

    // static int tryToParsePatternNum = 0;
    // tryToParsePatternNum ++;
    // cout << "\ntryToParsePatternNum = " << tryToParsePatternNum << std::endl;
    string PatternStr =  jval[U("Pattern")].as_string();
    // cout << "PatternStr = " << PatternStr << std::endl;
    string ParentPatternStr = jval[U("ParentPattern")].as_string();
    // cout << "ParentPatternStr = " << ParentPatternStr << std::endl;
    int ExtendedLinkIndex = jval[U("ExtendedLinkIndex")].as_integer();
    // cout << "ExtendedLinkIndex = " << ExtendedLinkIndex << std::endl;

    HTreeNode* newHTreeNode;
    // check if the pattern key already exist

    map<string, HTreeNode*>::iterator htreeNodeIter = keyStrToHTreeNodeMap.find(PatternStr);

    // static int duplicatePatternNum = 0;

    if (htreeNodeIter != keyStrToHTreeNodeMap.end())
    {
        newHTreeNode = htreeNodeIter->second;

        newHTreeNode->count ++;

//        duplicatePatternNum ++;
//        cout << "\nduplicatePatternNum = " << duplicatePatternNum << std::endl;
    }
    else
    {
        // add this new found pattern into the Atomspace
        HandleSeq patternHandleSeq = loadPatternIntoAtomSpaceFromString(PatternStr, atomSpace);
        if (patternHandleSeq.size() == 0)
        {

            cout << "Warning: Invalid pattern string: " << PatternStr << std::endl;
            return;

        }

        // create a new HTreeNode
        newHTreeNode = new HTreeNode();
        newHTreeNode->pattern = patternHandleSeq;
        newHTreeNode->count = 1;

        keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(PatternStr, newHTreeNode));

    }

    if (newHTreeNode->pattern.size() > 1) // this pattern is more than 1 gram, it should have a parent pattern
    {
        if (ParentPatternStr == "none")
        {

            cout << "Warning: Pattern missing parent pattern string:" << PatternStr;
            return;

        }
        else
        {

            map<string, HTreeNode*>::iterator parentNodeIter = keyStrToHTreeNodeMap.find(ParentPatternStr);

            if (parentNodeIter == keyStrToHTreeNodeMap.end())
            {

                cout << "Warning: Cannot find parent pattern in server: " << ParentPatternStr << std::endl;
                return;

            }
            else
            {
                // add super pattern relation

                HTreeNode* parentNode = parentNodeIter->second;


                ExtendRelation relation;
                relation.extendedHTreeNode = newHTreeNode;
                relation.newExtendedLink = (newHTreeNode->pattern)[ExtendedLinkIndex];

                parentNode->superPatternRelations.push_back(relation);
            }
        }
    }

}

// a patternStr is sent from a distributed worker via json, it's the keystring of a pattern
// the server need to load the string into links into AtomSpace
// e.g. a patternStr =
//  (InheritanceLink )\n
//    (VariableNode $var_1)\n
//    (ConceptNode human)\n\n
//  (EvaluationLink )\n
//    (PredicateNode like_drink)
//    (Listlink )\n
//      (VariableNode $var_1)\n
//      (ConceptNode soda)\n\n
//  (InheritanceLink )\n
//    (VariableNode $var_1)\n
//    (ConceptNode ugly)\n\n

//(AndLink)\n
//  (InheritanceLink )\n
//    (VariableNode $var_1)\n
//    (ConceptNode human)\n
//  (EvaluationLink )\n
//    (PredicateNode like_drink)
//    (Listlink )\n
//      (VariableNode $var_1)\n
//      (ConceptNode soda)\n\n
//(InheritanceLink )\n
//    (VariableNode $var_1)\n
//    (ConceptNode ugly)\n\n
HandleSeq PatternMiner::loadPatternIntoAtomSpaceFromString(string patternStr, AtomSpace *_atomSpace)
{

    std::vector<std::string> strs;
    boost::algorithm::split_regex( strs, patternStr, boost::regex( "\n\n" ) ) ;

    HandleSeq pattern;

//    // debug
//    if (strs.size() > 2)
//    {
//        int i = 0;
//        i++;
//    }

    for (string linkStr : strs) // load each link
    {
        if (linkStr == "")
            continue;

            HandleSeq rootOutgoings;

            std::size_t firstLineEndPos = linkStr.find("\n");
            std::string rootOutgoingStr = linkStr.substr(firstLineEndPos + 1);
            stringstream outgoingStream(rootOutgoingStr);

            if (! loadOutgoingsIntoAtomSpaceFromString(outgoingStream, _atomSpace, rootOutgoings))
            {
                cout << "Warning: loadPatternIntoAtomSpaceFromString: Parse pattern string error: " << linkStr << std::endl;
                HandleSeq emptyPattern;
                return emptyPattern;
            }

            std::size_t typeEndPos = linkStr.find(" ");
            string atomTypeStr = linkStr.substr(1, typeEndPos - 1);
            string linkOrNodeStr = atomTypeStr.substr(atomTypeStr.size() - 4, 4);

            if (linkOrNodeStr != "Link")
            {

                cout << "Warning: loadPatternIntoAtomSpaceFromString: Not a Link: " << linkOrNodeStr << std::endl;
                HandleSeq emptyPattern;
                return emptyPattern;

            }

            Type atomType = classserver().getType(atomTypeStr);
            if (NOTYPE == atomType)
            {

                cout << "Warning: loadPatternIntoAtomSpaceFromString: Not a valid typename: " << atomTypeStr << std::endl;
                HandleSeq emptyPattern;
                return emptyPattern;

            }

            Handle rootLink = _atomSpace->add_link(atomType, rootOutgoings);
            pattern.push_back(rootLink);


    }

    // debug:
    static int pattern_num = 1;
    string patternToStr = "";

    for(Handle h : pattern)
    {
        patternToStr += _atomSpace->atom_as_string(h);
        patternToStr += "\n";
    }

    cout << "\nAdded pattern: NO." << pattern_num << "\n" << patternToStr;
    pattern_num ++;

    return pattern;
}

// recursively function
bool PatternMiner::loadOutgoingsIntoAtomSpaceFromString(stringstream& outgoingStream, AtomSpace *_atomSpace, HandleSeq &outgoings, string parentIndent)
{
    string line;
    string curIndent = parentIndent + LINE_INDENTATION;

    while(getline(outgoingStream, line))
    {

        std::size_t nonIndentStartPos = line.find("(");
        string indent = line.substr(0, nonIndentStartPos);
        string nonIndentSubStr = line.substr(nonIndentStartPos + 1);
        std::size_t typeEndPos = nonIndentSubStr.find(" ");
        string atomTypeStr = nonIndentSubStr.substr(0, typeEndPos);
        string linkOrNodeStr = atomTypeStr.substr(atomTypeStr.size() - 4, 4);
        Type atomType = classserver().getType(atomTypeStr);
        if (NOTYPE == atomType)
        {
            cout << "Warning: loadOutgoingsIntoAtomSpaceFromString: Not a valid typename: " << atomTypeStr << std::endl;
            return false;

        }

        if (indent == curIndent)
        {
            if (linkOrNodeStr == "Node")
            {
                std::size_t nodeNameEndPos = nonIndentSubStr.find(")");
                string nodeName = nonIndentSubStr.substr(typeEndPos + 1, nodeNameEndPos - typeEndPos - 1);
                Handle node = _atomSpace->add_node(atomType, nodeName);
                outgoings.push_back(node);
            }
            else if (linkOrNodeStr == "Link")
            {
                // call this function recursively
                HandleSeq childOutgoings;
                if (! loadOutgoingsIntoAtomSpaceFromString(outgoingStream, _atomSpace, childOutgoings, curIndent))
                    return false;

                Handle link = _atomSpace->add_link(atomType, childOutgoings);
                outgoings.push_back(link);
            }
            else
            {
                cout << "Warning: loadOutgoingsIntoAtomSpaceFromString: Not a Node, neighter a Link: " << linkOrNodeStr << std::endl;
                return false;

            }

        }
        else if (indent.size() < curIndent.size())
        {
            return true;
        }
        else
        {
            // exception
            cout << "Warning: loadOutgoingsIntoAtomSpaceFromString: Indent wrong: " << line << std::endl;
            return false;


        }

    }

    return true;
}

void PatternMiner::centralServerEvaluateInterestingness()
{
    if (enable_Frequent_Pattern)
    {
        std::cout<<"Debug: PatternMiner:  done frequent pattern mining for 1 to "<< MAX_GRAM <<"gram patterns!\n";

        for(unsigned int gram = 1; gram <= MAX_GRAM; gram ++)
        {
            // sort by frequency
            std::sort((patternsForGram[gram-1]).begin(), (patternsForGram[gram-1]).end(),compareHTreeNodeByFrequency );

            // Finished mining gram patterns; output to file
            std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGram[gram-1]).size()) + " patterns found! ";

            OutPutFrequentPatternsToFile(gram);

            std::cout<< std::endl;
        }
    }


    if (enable_Interesting_Pattern)
    {
        for(cur_gram = 2; cur_gram <= MAX_GRAM - 1; cur_gram ++)
        {
            cout << "\nCalculating interestingness for " << cur_gram << " gram patterns by evaluating " << interestingness_Evaluation_method << std::endl;
            cur_index = -1;
            threads = new thread[THREAD_NUM];
            num_of_patterns_without_superpattern_cur_gram = 0;

            for (unsigned int i = 0; i < THREAD_NUM; ++ i)
            {
                threads[i] = std::thread([this]{this->evaluateInterestingnessTask();}); // using C++11 lambda-expression
            }

            for (unsigned int i = 0; i < THREAD_NUM; ++ i)
            {
                threads[i].join();
            }

            delete [] threads;

            std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") interestingness evaluation!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! ";
            std::cout<<"Outputting to file ... ";

            if (interestingness_Evaluation_method == "Interaction_Information")
            {
                // sort by interaction information
                std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByInteractionInformation);
                OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram);
            }
            else if (interestingness_Evaluation_method == "surprisingness")
            {
                // sort by surprisingness_I first
                std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeBySurprisingness_I);
                OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram,1);

                vector<HTreeNode*> curGramPatterns = patternsForGram[cur_gram-1];

                // and then sort by surprisingness_II
                std::sort(curGramPatterns.begin(), curGramPatterns.end(),compareHTreeNodeBySurprisingness_II);
                OutPutInterestingPatternsToFile(curGramPatterns,cur_gram,2);

                // Get the min threshold of surprisingness_II
                int threshold_index_II;
                threshold_index_II = SURPRISINGNESS_II_TOP_THRESHOLD * (float)(curGramPatterns.size() - num_of_patterns_without_superpattern_cur_gram);
                int looptimes = 0;
                while (true)
                {

                    surprisingness_II_threshold = (curGramPatterns[threshold_index_II])->nII_Surprisingness;
                    if (surprisingness_II_threshold <= 0.00000f)
                    {
                        if (++ looptimes > 8)
                        {
                            surprisingness_II_threshold = 0.00000f;
                            break;
                        }

                        threshold_index_II = ((float)threshold_index_II) * SURPRISINGNESS_II_TOP_THRESHOLD;
                    }
                    else
                        break;
                }


                cout<< "surprisingness_II_threshold for " << cur_gram << " gram = "<< surprisingness_II_threshold;

                // go through the top N patterns of surprisingness_I, pick the patterns with surprisingness_II higher than threshold
                int threshold_index_I = SURPRISINGNESS_I_TOP_THRESHOLD * (float)(curGramPatterns.size());
                for (int p = 0; p <= threshold_index_I; p ++)
                {
                    HTreeNode* pNode = (patternsForGram[cur_gram-1])[p];

                    // for patterns have no superpatterns, nII_Surprisingness == -1.0, which should be taken into account
                    if ( (pNode->nII_Surprisingness < 0 ) || (pNode->nII_Surprisingness > surprisingness_II_threshold ) )
                        finalPatternsForGram[cur_gram-1].push_back(pNode);
                }

                // sort by frequency
                std::sort((finalPatternsForGram[cur_gram-1]).begin(), (finalPatternsForGram[cur_gram-1]).end(),compareHTreeNodeByFrequency );

                OutPutFinalPatternsToFile(cur_gram);

            }

            std::cout<< std::endl;
        }
    }
}
