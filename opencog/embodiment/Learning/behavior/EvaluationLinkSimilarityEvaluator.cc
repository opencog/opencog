/*
 * opencog/embodiment/Learning/behavior/EvaluationLinkSimilarityEvaluator.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#include "EvaluationLinkSimilarityEvaluator.h"
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/Link.h>

using namespace behavior;

EvaluationLinkSimilarityEvaluator::~EvaluationLinkSimilarityEvaluator()
{
}

EvaluationLinkSimilarityEvaluator::EvaluationLinkSimilarityEvaluator()
{
}

float EvaluationLinkSimilarityEvaluator::similarity(AtomSpace& atomSpace, Handle evalLinkHandle1, Handle evalLinkHandle2)
{

    // EvalLink("behaved", ListLink(subject, action, action_arg1, action_arg2, ...))

    //printf("arity1 = %d\n", evalLink1->getArity());
    //printf("arity2 = %d\n", evalLink1->getArity());
    //printf("type1 = %s\n", atomSpace.getName(atomSpace.getType(evalLinkHandle1)).c_str());
    //printf("type2 = %s\n", atomSpace.getName(atomSpace.getType(evalLinkHandle2)).c_str());
    if ((atomSpace.getArity(evalLinkHandle1) < 2) || (atomSpace.getArity(evalLinkHandle2) < 2)) {
        return 0;
    }

    Handle listLink1 = atomSpace.getOutgoing(evalLinkHandle1, 1);
    Handle listLink2 = atomSpace.getOutgoing(evalLinkHandle2, 1);

    //printf("llarity1 = %d\n", listLink1->getArity());
    //printf("llarity2 = %d\n", listLink1->getArity());
    if ((atomSpace.getArity(listLink1) != atomSpace.getArity(listLink2)) || (atomSpace.getArity(listLink1) < 2)) {
        return 0;
    }

    // i > 0 is correct. The first element in the ListLink (subject) is
    // not supposed to be considered
    float answer = 1;
    for (int i = atomSpace.getArity(listLink1) - 1; i > 0; i--) {
        Handle h1 = atomSpace.getOutgoing(listLink1, i);
        Handle h2 = atomSpace.getOutgoing(listLink2, i);
        float sim = computeHandleSimilarity(atomSpace, h1, h2);
        answer *= sim;
    }

    return answer;
}

float EvaluationLinkSimilarityEvaluator::computeHandleSimilarity(AtomSpace& a, Handle h1, Handle h2)
{

    //printf("h1 = %s\n", a.atomAsString(h1).c_str());
    //printf("h2 = %s\n", a.atomAsString(h2).c_str());

    if (h1 == h2) {
        //printf("sim = <%f>\n", (float) 1);
        return 1;
    }

    Handle link_handle;
    HandleSeq set = a.getIncoming(h1);
    HandleSeq::iterator si;
    bool found = false;
    for (si = set.begin(); (! found) && (si != set.end()); ++si) {
        //printf("set.next\n");
        //printf("set.handle = %s\n", a.atomAsString(*si).c_str());
        Handle sh = *si;
        if (classserver().isA(a.getType(sh), SIMILARITY_LINK)) {
            std::vector<Handle> outgoing = a.getOutgoing(sh);
            for (int i = (int) outgoing.size() - 1; i >= 0; i--) {
                //printf("[%d]\n", i);
                //printf("outgoung[%d] = %s\n", i, a.atomAsString(outgoing[i]).c_str());
                if (outgoing[i] == h2) {
                    link_handle = sh;
                    found = true;
                    break;
                }
            }
        }
    }

    if (link_handle != Handle::UNDEFINED) {
        //printf("link = %s\n", link->toString().c_str());
        //printf("sim = %f\n", link->getTruthValue().getMean());
        return a.getMean(link_handle);
    } else {
        //printf("link = NULL\n");
        return 0;
    }
}
