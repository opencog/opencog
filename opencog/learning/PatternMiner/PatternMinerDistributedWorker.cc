/*
 * opencog/learning/PatternMiner/PatternMinerDistributedWorker.cc
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

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/StringManipulator.h>

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams

#include "HTree.h"
#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;


void PatternMiner::launchADistributedWorker(string serverURL)
{
    std::cout<<"Registering to the central server: "<< serverURL << std::endl;

    // Create http_client to send the request.
    httpClient = new http_client(U(serverURL.c_str()));

    // Build request URI and start the request.
    uri_builder builder(U("/RegisterNewWorker"));
    builder.append_query(U("ip"), U(""));
    httpClient->request(methods::GET, builder.to_string());



//// Note: Breadth_First mining is not maintained anymore. Only Depth_First is used in distributed mining.
////    Pattern_mining_mode = config().get("Pattern_mining_mode"); // option: Breadth_First , Depth_First
////    assert( (Pattern_mining_mode == "Breadth_First") || (Pattern_mining_mode == "Depth_First"));
//    Pattern_mining_mode = "Depth_First";

//    std::cout<<"A new PatternMining worker launched! Max gram = " + toString(this->MAX_GRAM) << ", mode = Depth_First" << std::endl;

//    int start_time = time(NULL);

//    originalAtomSpace->get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );

//    allLinkNumber = (int)(allLinks.size());
//    atomspaceSizeFloat = (float)(allLinkNumber);

//    if (Pattern_mining_mode == "Breadth_First")
//        runPatternMinerBreadthFirst();
//    else
//    {
//        runPatternMinerDepthFirst();

//        if (enable_Frequent_Pattern)
//        {
//            std::cout<<"Debug: PatternMiner:  done frequent pattern mining for 1 to "<< MAX_GRAM <<"gram patterns!\n";

//            for(unsigned int gram = 1; gram <= MAX_GRAM; gram ++)
//            {
//                // sort by frequency
//                std::sort((patternsForGram[gram-1]).begin(), (patternsForGram[gram-1]).end(),compareHTreeNodeByFrequency );

//                // Finished mining gram patterns; output to file
//                std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGram[gram-1]).size()) + " patterns found! ";

//                OutPutFrequentPatternsToFile(gram);

//                std::cout<< std::endl;
//            }
//        }


//        if (enable_Interesting_Pattern)
//        {
//            for(cur_gram = 2; cur_gram <= MAX_GRAM - 1; cur_gram ++)
//            {
//                cout << "\nCalculating interestingness for " << cur_gram << " gram patterns by evaluating " << interestingness_Evaluation_method << std::endl;
//                cur_index = -1;
//                threads = new thread[THREAD_NUM];
//                num_of_patterns_without_superpattern_cur_gram = 0;

//                for (unsigned int i = 0; i < THREAD_NUM; ++ i)
//                {
//                    threads[i] = std::thread([this]{this->evaluateInterestingnessTask();}); // using C++11 lambda-expression
//                }

//                for (unsigned int i = 0; i < THREAD_NUM; ++ i)
//                {
//                    threads[i].join();
//                }

//                delete [] threads;

//                std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") interestingness evaluation!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! ";
//                std::cout<<"Outputting to file ... ";

//                if (interestingness_Evaluation_method == "Interaction_Information")
//                {
//                    // sort by interaction information
//                    std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByInteractionInformation);
//                    OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram);
//                }
//                else if (interestingness_Evaluation_method == "surprisingness")
//                {
//                    // sort by surprisingness_I first
//                    std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeBySurprisingness_I);
//                    OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram,1);

//                    vector<HTreeNode*> curGramPatterns = patternsForGram[cur_gram-1];

//                    // and then sort by surprisingness_II
//                    std::sort(curGramPatterns.begin(), curGramPatterns.end(),compareHTreeNodeBySurprisingness_II);
//                    OutPutInterestingPatternsToFile(curGramPatterns,cur_gram,2);

//                    // Get the min threshold of surprisingness_II
//                    int threshold_index_II;
//                    threshold_index_II = SURPRISINGNESS_II_TOP_THRESHOLD * (float)(curGramPatterns.size() - num_of_patterns_without_superpattern_cur_gram);
//                    int looptimes = 0;
//                    while (true)
//                    {

//                        surprisingness_II_threshold = (curGramPatterns[threshold_index_II])->nII_Surprisingness;
//                        if (surprisingness_II_threshold <= 0.00000f)
//                        {
//                            if (++ looptimes > 8)
//                            {
//                                surprisingness_II_threshold = 0.00000f;
//                                break;
//                            }

//                            threshold_index_II = ((float)threshold_index_II) * SURPRISINGNESS_II_TOP_THRESHOLD;
//                        }
//                        else
//                            break;
//                    }


//                    cout<< "surprisingness_II_threshold for " << cur_gram << " gram = "<< surprisingness_II_threshold;

//                    // go through the top N patterns of surprisingness_I, pick the patterns with surprisingness_II higher than threshold
//                    int threshold_index_I = SURPRISINGNESS_I_TOP_THRESHOLD * (float)(curGramPatterns.size());
//                    for (int p = 0; p <= threshold_index_I; p ++)
//                    {
//                        HTreeNode* pNode = (patternsForGram[cur_gram-1])[p];

//                        // for patterns have no superpatterns, nII_Surprisingness == -1.0, which should be taken into account
//                        if ( (pNode->nII_Surprisingness < 0 ) || (pNode->nII_Surprisingness > surprisingness_II_threshold ) )
//                            finalPatternsForGram[cur_gram-1].push_back(pNode);
//                    }

//                    // sort by frequency
//                    std::sort((finalPatternsForGram[cur_gram-1]).begin(), (finalPatternsForGram[cur_gram-1]).end(),compareHTreeNodeByFrequency );

//                    OutPutFinalPatternsToFile(cur_gram);

//                }

//                std::cout<< std::endl;
//            }
//        }
//    }

//    int end_time = time(NULL);
//    printf("Pattern Mining Finish one round! Total time: %d seconds. \n", end_time - start_time);
//    std::cout<< THREAD_NUM << " threads used. \n";
//    std::cout<<"Corpus size: "<< allLinkNumber << " links in total. \n";

////   testPatternMatcher2();

////   selectSubsetFromCorpus();

}
