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

#include <opencog/nlp/types/atom_types.h>

#include "SuRealPMCB.h"


using namespace opencog::nlp;
using namespace opencog;


/**
 * The constructor for the PatternMatcherCallback.
 *
 * @param pAS    the corresponding AtomSpace
 * @param vars   the set of nodes that should be treated as variables
 */
SuRealPMCB::SuRealPMCB(AtomSpace* pAS, std::set<Handle> vars) : DefaultPatternMatchCB(pAS), m_vars(vars)
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
 * will be located.  Then we following the WordNode to find LgWordCset to get
 * the LG dictionary entries.  If the two words have identical LG entries,
 * then the two words matches.  Just one pair of LG entries need to match even
 * if a word has multiple entries.
 *
 * @param hPat    the variable, a node extracted from the original query
 * @param hSoln   the potential mapping
 * @return        true if solution is rejected, false if accepted
 */
bool SuRealPMCB::variable_match(Handle &hPat, Handle &hSoln)
{
    logger().debug("[SuReal] In variable_match, looking at %s", hSoln->toShortString().c_str());

    // reject if the solution is not of the same type
    if (hPat->getType() != hSoln->getType())
        return true;

    // VariableNode can be matched to any VariableNode, similarly for InterpretationNode
    if (hPat->getType() == VARIABLE_NODE || hPat->getType() == INTERPRETATION_NODE)
        return true;

    std::string sPat = _as->getName(hPat);
    std::string sPatWord = sPat.substr(0, sPat.find_first_of('@'));
    std::string sSoln = _as->getName(hSoln);
    std::string sSolnWord = sSoln.substr(0, sSoln.find_first_of('@'));

    // get the WordNode associated with the word (extracted from "word@1234" convention)
    Handle hPatWordNode = _as->getHandle(WORD_NODE, sPatWord);
    Handle hSolnWordNode = _as->getHandle(WORD_NODE, sSolnWord);

    // no WordNode? try NumberNode
    if (hSolnWordNode == Handle::UNDEFINED)
    {
        hPatWordNode = _as->getHandle(NUMBER_NODE, sPatWord);
        hSolnWordNode = _as->getHandle(NUMBER_NODE, sSolnWord);

        // no NumberNode neither
        if (hSolnWordNode == Handle::UNDEFINED)
            return true;
    }

    // get the neighbor of each WordNode within LgWordCset
    HandleSeq qPatSet =_as->getNeighbors(hPatWordNode, false, true, LG_WORD_CSET, false);
    HandleSeq qSolnSet = _as->getNeighbors(hSolnWordNode, false, true, LG_WORD_CSET, false);

    // check the HandleSeq to see if there is any pair that is equal (common LG entry)
    std::sort(qPatSet.begin(), qPatSet.end());
    std::sort(qSolnSet.begin(), qSolnSet.end());

    HandleSeq qItrsect;
    std::set_intersection(qPatSet.begin(), qPatSet.end(), qSolnSet.begin(), qSolnSet.end(), std::back_inserter(qItrsect));

    // if there's no common LG dictionary rules, then the two words do not match
    if (qItrsect.empty())
        return true;

    return false;
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
 * @return                true if rejected, false if accepted
 */
bool SuRealPMCB::clause_match(Handle &pattrn_link_h, Handle &grnd_link_h)
{
    logger().debug("[SuReal] In clause_match, looking at %s", grnd_link_h->toShortString().c_str());

    HandleSeq qISet = _as->getIncoming(grnd_link_h);

    // keep only SetLink, and check if any of the SetLink has an InterpretationNode as neightbor
    qISet.erase(std::remove_if(qISet.begin(), qISet.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qISet.end());

    // helper lambda function to check for linkage to an InterpretationNode, given SetLink
    auto hasInterpretation = [this](Handle& h)
    {
        HandleSeq qN = _as->getNeighbors(h, true, false, REFERENCE_LINK, false);
        return std::any_of(qN.begin(), qN.end(), [](Handle& hn) { return hn->getType() == INTERPRETATION_NODE; });
    };

    // reject match (ie. return true) if there is no InterpretationNode
    return not std::any_of(qISet.begin(), qISet.end(), hasInterpretation);
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
        HandleSeq qISet = _as->getIncoming(h);
        qISet.erase(std::remove_if(qISet.begin(), qISet.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qISet.end());

        HandleSeq results;

        for (auto& hSetLink : qISet)
        {
            HandleSeq qN  = _as->getNeighbors(hSetLink, true, false, REFERENCE_LINK, false);
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

        shrinked_soln[kv.first] = kv.second;
    }

    // store the solution; all common InterpretationNode are solutions for this
    // grounding, so store the solution for each InterpretationNode
    for (auto n : qItprNode)
        m_results[n] = shrinked_soln;

    return false;
}

/**
 * Implement the perform_search method.
 *
 * Similar to DefaultPatternMatcherCB::perform_search, in which we start search
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
void SuRealPMCB::perform_search(PatternMatchEngine* pPME, std::set<Handle>& vars, HandleSeq& clauses, HandleSeq& negations)
{
    size_t bestClauseIndex;
    Handle bestClause, bestSubClause, bestSubNode;

    // find the thinnest clause with constants
    bestSubNode = find_thinnest(clauses, bestSubClause, bestClauseIndex);

    if (bestSubNode != Handle::UNDEFINED && !vars.empty())
    {
        bestClause = clauses[bestClauseIndex];

        logger().debug("[SuReal] Search start node: %s", bestSubNode->toShortString().c_str());
        logger().debug("[SuReal] Start pred is: %s", bestSubClause->toShortString().c_str());

        IncomingSet iset = get_incoming_set(bestSubNode);

        for (auto& l : iset)
        {
            Handle h(l);
            logger().debug("[SuReal] Loop candidate: %s", h->toShortString().c_str());

            if (pPME->do_candidate(bestClause, bestSubClause, h))
                break;
        }

        return;
    }

    // reaching here means no contants, so do some search space reduction here
    bestClause = clauses[0];

    logger().debug("[SuReal] Start pred is: %s", bestClause->toShortString().c_str());

    // helper for checking for InterpretationNode
    auto hasNoInterpretation = [this](Handle& h)
    {
        HandleSeq qISet = _as->getIncoming(h);
        qISet.erase(std::remove_if(qISet.begin(), qISet.end(), [](Handle& hl) { return hl->getType() != SET_LINK; }), qISet.end());

        auto hasNode = [this](Handle& hl)
        {
            HandleSeq qN = _as->getNeighbors(hl, true, false, REFERENCE_LINK, false);
            return std::any_of(qN.begin(), qN.end(), [](Handle& hn) { return hn->getType() == INTERPRETATION_NODE; });
        };

        return not std::any_of(qISet.begin(), qISet.end(), hasNode);
    };

    // keep only links of the same type as bestClause and have linkage to InterpretationNode
    HandleSeq qCandidate;
    _as->getHandlesByType(std::back_inserter(qCandidate), bestClause->getType());
    qCandidate.erase(std::remove_if(qCandidate.begin(), qCandidate.end(), hasNoInterpretation), qCandidate.end());

    for (auto& c : qCandidate)
    {
        logger().debug("[SuReal] Loop candidate: %s", c->toShortString().c_str());

        if (pPME->do_candidate(bestClause, bestClause, c))
            break;
    }
}

/**
 * Override the find_starter method in DefaultPatternMatchCB.
 *
 * Override find_starter so that it will not treat VariableNode variables,
 * but instead compare against the variable list.
 *
 * @param h       same as DefaultPatternMatchCB::find_starter
 * @param depth   same as DefaultPatternMatchCB::find_starter
 * @param start   same as DefaultPatternMatchCB::find_starter
 * @param width   same as DefaultPatternMatchCB::find_starter
 * @return        same as DefaultPatternMatchCB::find_starter but change
 *                the result if is a variable
 */
Handle SuRealPMCB::find_starter(Handle h, size_t& depth, Handle& start, size_t& width)
{
    Handle rh = DefaultPatternMatchCB::find_starter(h, depth, start, width);

    // if the non-VariableNode is actually a variable
    if (m_vars.count(rh) == 1)
        return Handle::UNDEFINED;

    return rh;
}

