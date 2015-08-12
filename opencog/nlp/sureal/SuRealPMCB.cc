/*
 * SuRealPMCB.cc
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#include <opencog/atomutils/AtomUtils.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/nlp/types/atom_types.h>
#include <opencog/nlp/lg-dict/LGDictUtils.h>

#include "SuRealPMCB.h"


using namespace opencog::nlp;
using namespace opencog;


/**
 * The constructor for the PatternMatcherCallback.
 *
 * @param pAS            the corresponding AtomSpace
 * @param vars           the set of nodes that should be treated as variables
 * @param thoroughness   the completeness of the search (the minimum number of
 *                       results being returned)
 */
SuRealPMCB::SuRealPMCB(AtomSpace* pAS, const std::set<Handle>& vars, size_t thoroughness) :
    InitiateSearchCB(pAS),
    DefaultPatternMatchCB(pAS),
    m_as(pAS),
    m_vars(vars),
    m_eval(SchemeEval::get_evaluator(pAS)),
    m_thoroughness(thoroughness)
{

}

SuRealPMCB::~SuRealPMCB()
{
}

/**
 * Override the variable_match callback.
 *
 * Do the special checking for each variable matches.  Note that variables
 * in SuReal will not just be VariableNode, but any nodes with an actual word
 * (eg. (PredicateNode "runs"), (ConceptNode "dog@1324-12213"), etc).
 *
 * For each variable and its potential solution, the corresponding WordNode
 * will be located.  Then we find the source LG connectors used to match to the
 * the solution WordNode, and check each disjuncts of the variable WordNode to
 * see if any of them can match the source.  If so, then the solution WordNode
 * can be replaced by the variable WordNode.
 *
 * @param hPat    the variable, a node extracted from the original query
 * @param hSoln   the potential mapping
 * @return        false if solution is rejected, true if accepted
 */
bool SuRealPMCB::variable_match(const Handle &hPat, const Handle &hSoln)
{
    logger().debug("[SuReal] In variable_match, looking at %s", hSoln->toShortString().c_str());

    // reject if the solution is not of the same type
    if (hPat->getType() != hSoln->getType())
        return false;

    // VariableNode can be matched to any VariableNode, similarly for InterpretationNode
    if (hPat->getType() == VARIABLE_NODE || hPat->getType() == INTERPRETATION_NODE)
        return true;

    std::string sPat = m_as->get_name(hPat);
    std::string sPatWord = sPat.substr(0, sPat.find_first_of('@'));
    std::string sSoln = m_as->get_name(hSoln);

    // get the WordNode associated with the word (extracted from "word@1234" convention)
    Handle hPatWordNode = m_as->get_handle(WORD_NODE, sPatWord);

    // get the corresponding WordInstanceNode for hSoln
    Handle hSolnWordInst = m_as->get_handle(WORD_INSTANCE_NODE, sSoln);

    // no WordInstanceNode? reject!
    if (hSolnWordInst == Handle::UNDEFINED)
        return false;

    // the source connectors for the solution
    HandleSeq qTargetConns;

    // get all the ListLinks connecting to hSolnWordInst
    HandleSeq qSolnListLinks;
    hSolnWordInst->getIncomingSetByType(back_inserter(qSolnListLinks), LIST_LINK);

    // helper for checking if a ListLink has a LgLinkInstanceNode as a neighbor
    auto hasNoLgLinkInstanceNode = [](Handle& h)
    {
        HandleSeq qN = get_neighbors(h, true, false, EVALUATION_LINK);
        return not std::any_of(qN.begin(), qN.end(), [](Handle& hn) { return hn->getType() == LG_LINK_INSTANCE_NODE; });
    };

    // remove any ListLink that doesn't have a LgLinkInstanceNode as a neighbor
    qSolnListLinks.erase(std::remove_if(qSolnListLinks.begin(), qSolnListLinks.end(), hasNoLgLinkInstanceNode), qSolnListLinks.end());

    HandleSeq qLGInstsLeft;
    HandleSeq qLGInstsRight;
    for (Handle& hSolnListLink : qSolnListLinks)
    {
        HandleSeq qOS = LinkCast(hSolnListLink)->getOutgoingSet();

        // divide them into two groups, assuming there are only two WordInstanceNodes in the ListLink
        if (qOS[0] == hSolnWordInst) qLGInstsRight.push_back(hSolnListLink);
        if (qOS[1] == hSolnWordInst) qLGInstsLeft.push_back(hSolnListLink);
    }

    // helper for sorting links in reverse word sequence order
    auto sortLeftWords = [](const Handle& h1, const Handle& h2)
    {
        HandleSeq qOS1 = LinkCast(h1)->getOutgoingSet();
        HandleSeq qOS2 = LinkCast(h2)->getOutgoingSet();

        Handle& hLeftWordInst1 = qOS1[0];
        Handle& hLeftWordInst2 = qOS2[0];

        HandleSeq qN1 = get_neighbors(hLeftWordInst1, false, true, WORD_SEQUENCE_LINK);
        HandleSeq qN2 = get_neighbors(hLeftWordInst2, false, true, WORD_SEQUENCE_LINK);

        // get the NumberNode and compare their word sequences
        return NodeCast(qN1[0])->getName() > NodeCast(qN2[0])->getName();
    };

    // helper for sorting links in word sequence order
    auto sortRightWords = [](const Handle& h1, const Handle& h2)
    {
        HandleSeq qOS1 = LinkCast(h1)->getOutgoingSet();
        HandleSeq qOS2 = LinkCast(h2)->getOutgoingSet();

        Handle& hRightWordInst1 = qOS1[1];
        Handle& hRightWordInst2 = qOS2[1];

        HandleSeq qN1 = get_neighbors(hRightWordInst1, false, true, WORD_SEQUENCE_LINK);
        HandleSeq qN2 = get_neighbors(hRightWordInst2, false, true, WORD_SEQUENCE_LINK);

        // get the NumberNode and compare their word sequences
        return NodeCast(qN1[0])->getName() < NodeCast(qN2[0])->getName();
    };

    // sort the qLGInstsLeft in reverse word sequence order
    std::sort(qLGInstsLeft.begin(), qLGInstsLeft.end(), sortLeftWords);

    // sort the qLGInstsRight in word sequence order
    std::sort(qLGInstsRight.begin(), qLGInstsRight.end(), sortRightWords);

    // get the LG connectors for those in the qLGInstsLeft
    for (Handle& hLGListLink : qLGInstsLeft)
    {
        HandleSeq qN = get_neighbors(hLGListLink, true, true, EVALUATION_LINK);

        auto it = std::find_if(qN.begin(), qN.end(), [](Handle& h){ return h->getType() == LG_LINK_INSTANCE_NODE; });
        if (it != qN.end())
        {
            HandleSeq qLGConns = get_neighbors(*it, true, true, LG_LINK_INSTANCE_LINK);

            // get the first LG connector
            qTargetConns.push_back(qLGConns[0]);
        }
    }

    // get the LG connectors for those in the qLGInstsRight
    for (Handle& hLGListLink : qLGInstsRight)
    {
        HandleSeq qN = get_neighbors(hLGListLink, true, true, EVALUATION_LINK);

        auto it = std::find_if(qN.begin(), qN.end(), [](Handle& h){ return h->getType() == LG_LINK_INSTANCE_NODE; });
        if (it != qN.end())
        {
            HandleSeq qLGConns = get_neighbors(*it, true, true, LG_LINK_INSTANCE_LINK);

            // get the second LG connector
            qTargetConns.push_back(qLGConns[1]);
        }
    }

    // disjuncts of the hPatWordNode
    HandleSeq qDisjuncts;

    // check if we got the disjuncts of the hPatWordNode already, otherwise
    // store them in a map so that we only need to do this disjuncts-getting procedure once
    auto iter = m_disjuncts.find(hPatWordNode);
    if (iter == m_disjuncts.end())
    {
        HandleSeq qOr = get_neighbors(hPatWordNode, false, true, LG_WORD_CSET, false);

        auto insertHelper = [&](const Handle& h)
        {
            HandleSeq q = m_as->get_outgoing(h);
            qDisjuncts.insert(qDisjuncts.end(), q.begin(), q.end());
        };

        std::for_each(qOr.begin(), qOr.end(), insertHelper);

        m_disjuncts.insert({hPatWordNode, qDisjuncts});
    }
    else qDisjuncts = iter->second;

    logger().debug("[SuReal] Looking at %d disjuncts of %s", qDisjuncts.size(), hPat->toShortString().c_str());

    // for each disjunct, get its outgoing set, and match 1-to-1 with qTargetConns
    auto matchHelper = [&](const Handle& hDisjunct)
    {
        std::list<Handle> sourceConns;
        std::list<Handle> targetConns(qTargetConns.begin(), qTargetConns.end());

        // check if hDisjunct is LgAnd or just a lone connector
        if (hDisjunct->getType() == LG_AND)
        {
            HandleSeq q = m_as->get_outgoing(hDisjunct);
            sourceConns = std::list<Handle>(q.begin(), q.end());
        }
        else
        {
            sourceConns.push_back(hDisjunct);
        }

        Handle hMultiConn = Handle::UNDEFINED;

        // loop thru all connectors on both list
        while (not sourceConns.empty() && not targetConns.empty())
        {
            bool bResult;

            if (hMultiConn != Handle::UNDEFINED)
                bResult = lg_conn_linkable(hMultiConn, targetConns.front());
            else
                bResult = lg_conn_linkable(sourceConns.front(), targetConns.front());

            // if the two connectors cannot be linked
            if (not bResult)
            {
                // don't pop anything if we were retrying a multi-connector
                if (hMultiConn != Handle::UNDEFINED)
                {
                    hMultiConn = Handle::UNDEFINED;
                    continue;
                }

                return false;
            }

            // pop only the target connector if we were repeating a multi-conn
            if (hMultiConn != Handle::UNDEFINED)
            {
                targetConns.pop_front();
                continue;
            }

            // dumb hacky way of checking of the connector is a multi-connector
            if (m_as->get_outgoing(sourceConns.front()).size() == 3)
                hMultiConn = sourceConns.front();

            sourceConns.pop_front();
            targetConns.pop_front();
        }

        // check if both source and target are used up
        if (not sourceConns.empty() or not targetConns.empty())
            return false;

        logger().debug("[SuReal] " + hDisjunct->toShortString() + " passed!");

        return true;
    };

    return std::any_of(qDisjuncts.begin(), qDisjuncts.end(), matchHelper);
}

/**
 * Override the clause_match callback.
 *
 * Reject a clause match if the matched link is not in a SetLink linked to an
 * InterpretationNode.  The reason of doing it here instead of earlier in
 * link_match() is because it is possible a SetLink is used for other purpose
 * as part of R2L's output.
 *
 * @param pattrn_link_h   the matched clause, extracted from the input SetLink
 * @param grnd_link_h     the corresponding grounding to be checked
 * @return                false if rejected, true if accepted
 */
bool SuRealPMCB::clause_match(const Handle &pattrn_link_h, const Handle &grnd_link_h)
{
    logger().debug("[SuReal] In clause_match, looking at %s", grnd_link_h->toShortString().c_str());

    HandleSeq qISet = m_as->get_incoming(grnd_link_h);

    // keep only SetLink, and check if any of the SetLink has an InterpretationNode as neightbor
    qISet.erase(std::remove_if(qISet.begin(), qISet.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qISet.end());

    // helper lambda function to check for linkage to an InterpretationNode, given SetLink
    auto hasInterpretation = [this](Handle& h)
    {
        HandleSeq qN = get_neighbors(h, true, false, REFERENCE_LINK, false);
        return std::any_of(qN.begin(), qN.end(), [](Handle& hn) { return hn->getType() == INTERPRETATION_NODE; });
    };

    // reject match (ie. return false) if there is no InterpretationNode
    return std::any_of(qISet.begin(), qISet.end(), hasInterpretation);
}

/**
 * Implement the grounding method.
 *
 * Check all matched clauses to see if they are all within the same SetLink
 * linking to the same InterpretationNode.  If so, store the solution and try
 * to find more.
 *
 * @param var_soln   the variable & links mapping
 * @param pred_soln  the clause mapping
 * @return           always return false to search for more solutions
 */
bool SuRealPMCB::grounding(const std::map<Handle, Handle> &var_soln, const std::map<Handle, Handle> &pred_soln)
{
    logger().debug("[SuReal] grounding a solution");

    // helper to get the InterpretationNode
    auto getInterpretation = [this](const Handle& h)
    {
        HandleSeq qISet = m_as->get_incoming(h);
        qISet.erase(std::remove_if(qISet.begin(), qISet.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qISet.end());

        HandleSeq results;

        for (auto& hSetLink : qISet)
        {
            HandleSeq qN  = get_neighbors(hSetLink, true, false, REFERENCE_LINK, false);
            qN.erase(std::remove_if(qN.begin(), qN.end(), [](Handle& h) { return h->getType() != INTERPRETATION_NODE; }), qN.end());

            results.insert(results.end(), qN.begin(), qN.end());
        }

        return results;
    };

    HandleSeq qItprNode = getInterpretation(pred_soln.begin()->second);
    std::sort(qItprNode.begin(), qItprNode.end());

    // try to find a common InterpretationNode to the solutions
    for (std::map<Handle, Handle>::const_iterator it = ++pred_soln.begin(); it != pred_soln.end(); ++it)
    {
        HandleSeq thisSeq = getInterpretation(it->second);
        std::sort(thisSeq.begin(), thisSeq.end());

        HandleSeq overlapSeq;
        std::set_intersection(qItprNode.begin(), qItprNode.end(), thisSeq.begin(), thisSeq.end(), std::back_inserter(overlapSeq));

        qItprNode = overlapSeq;
    }

    // no common InterpretationNode, ignore this grounding
    if (qItprNode.empty())
        return false;

    // shrink var_soln to only contain solution for the variables
    std::map<Handle, Handle> shrinked_soln;

    for (const auto& kv : var_soln)
    {
        if (m_vars.count(kv.first) == 0)
            continue;

        auto checker = [&](const std::pair<Handle, Handle>& nkv)
        {
            return kv.second == nkv.second;
        };

        // if another variable already map to the same solution, discard this grounding
        // because each variable should map to its own unique solution
        if (std::any_of(shrinked_soln.begin(), shrinked_soln.end(), checker))
            return false;

        shrinked_soln[kv.first] = kv.second;
    }

    // store the solution; all common InterpretationNode are solutions for this
    // grounding, so store the solution for each InterpretationNode
    for (auto n : qItprNode)
    {
        // there could also be multiple solutions for one InterpretationNode,
        // so store them in a vector
        if (m_results.count(n) == 0)
            m_results[n] = std::vector<std::map<Handle, Handle> >();

        logger().debug("[SuReal] grounding Interpreation: %s", n->toShortString().c_str());
        m_results[n].push_back(shrinked_soln);
    }

    return false;
}

/**
 * Implement the initiate_search method.
 *
 * Similar to DefaultPatternMatcherCB::initiate_search, in which we start search
 * by looking at the thinnest clause with constants.  However, since most clauses
 * for SuReal will have 0 constants, most searches will require looking at all
 * the links.  This implementation improves that by looking at links within a
 * SetLink within a ReferenceLink with a InterpretationNode neightbor, thus
 * limiting the search space.
 *
 * @param pPME       pointer to the PatternMatchEngine
 * @param vars       a set of nodes that are variables
 * @param clauses    the clauses for the query
 * @param negations  the negative clauses
 */
bool SuRealPMCB::initiate_search(PatternMatchEngine* pPME)
{
    _search_fail = false;
    if (not _variables->varset.empty())
    {
        bool found = neighbor_search(pPME);
        if (not _search_fail) return found;
    }

    // Reaching here means no constants, so do some search space
    // reduction here
    Handle bestClause = _pattern->mandatory[0];

    logger().debug("[SuReal] Start pred is: %s",
                   bestClause->toShortString().c_str());

    // keep only links of the same type as bestClause and have linkage to InterpretationNode
    HandleSeq qCandidate;
    m_as->get_handles_by_type(std::back_inserter(qCandidate), bestClause->getType());

    // selected candidates, a subset of qCandidate
    std::vector<CandHandle> sCandidate;

    for (auto& c : qCandidate)
    {
        auto rm = [](Handle& h)
        {
            if (h->getType() != SET_LINK) return true;

            HandleSeq qN = get_neighbors(h, true, false, REFERENCE_LINK, false);
            return not std::any_of(qN.begin(), qN.end(),
                [](Handle& hn) { return hn->getType() == INTERPRETATION_NODE; });
        };

        HandleSeq qISet = m_as->get_incoming(c);

        // erase atoms that are neither R2L-Setlink nor SetLink
        qISet.erase(std::remove_if(qISet.begin(), qISet.end(), rm), qISet.end());

        if (qISet.size() >= 1)
        {
            size_t maxSize = 0;
            for (Handle& q : qISet)
            {
                size_t s = LinkCast(q)->getArity();
                if (s > maxSize) maxSize = s;
            }

            CandHandle ch;
            ch.handle = c;
            ch.r2lSetLinkSize = maxSize;
            sCandidate.push_back(ch);
        }
    }

    auto sortBySize = [](const CandHandle& h1, const CandHandle& h2)
    { return h1.r2lSetLinkSize < h2.r2lSetLinkSize; };

    std::sort(sCandidate.begin(), sCandidate.end(), sortBySize);

    for (auto& c : sCandidate)
    {
        logger().debug("[SuReal] Loop candidate: %s", c.handle->toShortString().c_str());

        if (pPME->explore_neighborhood(bestClause, bestClause, c.handle))
            return true;

        // stop the search if it already found enough results in a non-complete search
        if (m_thoroughness > 0 and m_results.size() >= m_thoroughness) return true;
    }
    return false;
}

/**
 * Override the find_starter method in DefaultPatternMatchCB.
 *
 * Override find_starter so that it will not treat VariableNode variables,
 * but instead compare against the variable list.
 *
 * @param h       same as InitiateSearchCB::find_starter
 * @param depth   same as InitiateSearchCB::find_starter
 * @param start   same as InitiateSearchCB::find_starter
 * @param width   same as InitiateSearchCB::find_starter
 * @return        same as InitiateSearchCB::find_starter but change
 *                the result if is a variable
 */
Handle SuRealPMCB::find_starter(const Handle& h, size_t& depth,
                                Handle& start, size_t& width)
{
    Handle rh = InitiateSearchCB::find_starter(h, depth, start, width);

    // if the non-VariableNode is actually a variable
    if (m_vars.count(rh) == 1)
        return Handle::UNDEFINED;

    return rh;
}
