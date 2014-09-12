/*
 * opencog/learning/PatternMiner/PatternMinerDF.cc
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
#include <opencog/query/BindLink.h>
#include <opencog/util/Config.h>
#include <opencog/util/foreach.h>
#include <opencog/util/StringManipulator.h>

#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;


void PatternMiner::growPatternsDepthFirstTask()
{
    while(true)
    {
        readNextLinkLock.lock();
        cur_index ++;

        if (cur_index < allLinkNumber)
        {
            cout<< "\r" << (float)(cur_index)/(float)(allLinkNumber)*100.0f << + "% completed." ;
            std::cout.flush();
        }
        else
        {
            if (cur_index == allLinkNumber)
            {
                cout<< "\r100% completed." ;
                std::cout.flush();
            }

            readNextLinkLock.unlock();
            break;

        }

        readNextLinkLock.unlock();

        Handle& cur_link = allLinks[cur_index];

        // if this link is listlink, ignore it
        if (originalAtomSpace->getType(cur_link) == opencog::LIST_LINK)
        {
            continue;
        }

        // Add this link into observingAtomSpace
        HandleSeq outgoingLinks,outVariableNodes;

        swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, cur_link, outgoingLinks, outVariableNodes);
        Handle newLink = observingAtomSpace->addLink(originalAtomSpace->getType(cur_link),outgoingLinks,originalAtomSpace->getTV(cur_link));


        HandleSeq observedLinks;
        observedLinks.push_back(newLink);

        // Extract all the possible patterns from this originalLinks, not duplicating the already existing patterns
        set<Handle> sharedNodes;
        extractAllPossiblePatternsFromInputLinks(observedLinks, 0, sharedNodes, observingAtomSpace,1);


        HandleSeqSeq allLastGramConnectedLinks; // for this cur_link
        allLastGramConnectedLinks.push_back(observedLinks);

        unsigned int gram;

        for ( gram = 2; gram <= MAX_GRAM; ++ gram)
        {
            vector<set<Handle>> newConnectedLinksFoundThisGram;

            foreach(HandleSeq linksToExtend, allLastGramConnectedLinks)
            {
                // find all the cur_gram distance neighbour links of newLink
                extendAllPossiblePatternsForOneMoreGram(linksToExtend,0,observingAtomSpace,gram,newConnectedLinksFoundThisGram);
            }

            allLastGramConnectedLinks.clear();
            foreach(set<Handle>& newFoundLinksGroup, newConnectedLinksFoundThisGram)
            {
                HandleSeq newFoundLinks(newFoundLinksGroup.begin(), newFoundLinksGroup.end());
                allLastGramConnectedLinks.push_back(newFoundLinks);
            }

        }

    }
}



void PatternMiner::runPatternMinerDepthFirst()
{
    // observingAtomSpace is used to copy one link everytime from the originalAtomSpace
    observingAtomSpace = new AtomSpace();

    cur_index = -1;

    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {
        threads[i] = std::thread([this]{this->growPatternsDepthFirstTask();}); // using C++11 lambda-expression
    }

    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {
        threads[i].join();
    }

    // release allLinks
    allLinks.clear();
    (vector<Handle>()).swap(allLinks);

    cout << "\nFinished mining 1~" << MAX_GRAM << " gram patterns.\n";

}
