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
#include <opencog/atomutils/FindUtils.h>
#include <opencog/atomutils/Neighbors.h>
#include <opencog/nlp/types/atom_types.h>
#include <opencog/nlp/lg-dict/LGDictUtils.h>

#include "SuRealPMCB.h"
#include "SuRealCache.h"


using namespace opencog::nlp;
using namespace opencog;
using namespace std;


/**
 * The constructor for the PatternMatcherCallback.
 *
 * @param pAS            the corresponding AtomSpace
 * @param vars           the set of nodes that should be treated as variables
 */
SuRealPMCB::SuRealPMCB(AtomSpace* pAS, const HandleSet& vars, bool use_cache) :
    InitiateSearchCB(pAS),
    DefaultPatternMatchCB(pAS),
    m_as(pAS),
    m_vars(vars)
{
    m_use_cache = use_cache;
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
 * @param hPat    the variable, a node extracted from the original query
 * @param hSoln   the potential mapping
 * @return        false if solution is rejected, true if accepted
 */
bool SuRealPMCB::variable_match(const Handle &hPat, const Handle &hSoln)
{

    if (m_use_cache) {
        int cached = SuRealCache::instance().variable_match(hPat, hSoln);
        if (cached >= 0) {
            if (cached == 0) {
                return false;
            } else {
                return true;
            }
        }
    }

    logger().debug("[SuReal] In variable_match, looking at %s",
          hSoln->toShortString().c_str());

    bool answer;

    // Reject if the solution is not of the same type.
    if (hPat->getType() != hSoln->getType()) {
        answer = false;
    } else {
        // VariableNode can be matched to any VariableNode, similarly
        // for InterpretationNode
        if (hPat->getType() == VARIABLE_NODE or
            hPat->getType() == INTERPRETATION_NODE) {
            answer = true;
        } else {
            std::string sSoln = hSoln->getName();
            // get the corresponding WordInstanceNode for hSoln
            Handle hSolnWordInst = m_as->get_handle(WORD_INSTANCE_NODE, sSoln);
            // no WordInstanceNode? reject!
            if (hSolnWordInst == Handle::UNDEFINED) {
                answer = false;
            } else {
                answer = true;
            }
        }
    }

    if (m_use_cache) {
        SuRealCache::instance().add_variable_match(hPat, hSoln, answer);
    }

    return answer;
}

/**
 * Get all the nodes within a link and its sublinks.
 *
 * @param h     the top level link
 * @return      a HandleSeq of nodes
 */
static void get_nodes(const Handle& h, HandleSeq& node_list)
{
   if (h->isNode())
   {
      node_list.emplace_back(h);
      return;
   }

   for (const Handle& o : h->getOutgoingSet())
      get_nodes(o, node_list);
}

static void get_all_nodes(const Handle& h, HandleSeq& node_list, bool use_cache)
{
    if (use_cache) {
        bool cached = SuRealCache::instance().get_node_list(h, node_list);
        if (! cached) {
            get_nodes(h, node_list);
            SuRealCache::instance().add_node_list(h, node_list);
        }
    } else {
        get_nodes(h, node_list);
    }
}

/**
 * Override the clause_match callback.
 *
 * Reject a clause match if the matched link is not in a SetLink linked to an
 * InterpretationNode.  The reason of doing it here instead of earlier in
 * link_match() is because it is possible a SetLink is used for other purpose
 * as part of R2L's output.
 *
 * For each accepted SetLink, we look at its nodes one by one, find the source
 * LG connectors used to match to the pattern WordNode, and check each disjuncts
 * of the solution WordNode to see if any of them can match the source.
 * If so, then the solution WordNode can be replaced by the pattern WordNode.
 *
 * @param pattrn_link_h   the matched clause, extracted from the input SetLink
 * @param grnd_link_h     the corresponding grounding to be checked
 * @return                false if rejected, true if accepted
 */
bool SuRealPMCB::clause_match(const Handle &pattrn_link_h, const Handle &grnd_link_h)
{
    if (m_use_cache) {
        int cached = SuRealCache::instance().clause_match(pattrn_link_h, grnd_link_h);
        if (cached >= 0) {
            if (cached == 0) {
                return false;
            } else {
                return true;
            }
        }
    }

    logger().debug("[SuReal] In clause_match, looking at %s",
        grnd_link_h->toShortString().c_str());

    HandleSeq qISet;
    grnd_link_h->getIncomingSetByType(back_inserter(qISet), SET_LINK);

    // Store the InterpretationNodes, will be needed if this grounding
    // is accepted.
    HandleSeq qTempInterpNodes;

    // helper lambda function to check for linkage to an InterpretationNode,
    // and see if it's one of the targets we are looking for, given SetLink
    auto hasInterpretation = [&](Handle& h)
    {
        HandleSeq qN = get_source_neighbors(h, REFERENCE_LINK);
        return std::any_of(qN.begin(), qN.end(), [&](Handle& hn) {
                bool isInterpNode = hn->getType() == INTERPRETATION_NODE;
                bool isTarget = m_targets.size() > 0? (m_targets.find(hn) != m_targets.end()) : true;
                if (isInterpNode) qTempInterpNodes.push_back(hn);
                return isInterpNode and isTarget; });
    };

    // reject match (ie. return false) if there is no InterpretationNode, or
    // the InterpretationNode is not one of the targets
    if (not std::any_of(qISet.begin(), qISet.end(), hasInterpretation)) {
        if (m_use_cache) {
            SuRealCache::instance().add_clause_match(pattrn_link_h, grnd_link_h, false);
        }
        return false;
    }

    // Get all the nodes from the pattern and the potential solution.
    HandleSeq qAllPatNodes;
    get_all_nodes(pattrn_link_h, qAllPatNodes, m_use_cache);
    HandleSeq qAllSolnNodes;
    get_all_nodes(grnd_link_h, qAllSolnNodes, m_use_cache);

    // Just in case if their sizes are not the same, reject the match.
    if (qAllPatNodes.size() != qAllSolnNodes.size()) {
        if (m_use_cache) {
            SuRealCache::instance().add_clause_match(pattrn_link_h, grnd_link_h, false);
        }
        return false;
    }

    // Compare the disjuncts for each of the nodes.
    for (size_t i = 0; i < qAllPatNodes.size(); i++)
    {
        Handle& hPatNode = qAllPatNodes[i];
        Handle& hSolnNode = qAllSolnNodes[i];

        // move on if the pattern node is a VariableNode or an InperpretationNode
        if (hPatNode->getType() == VARIABLE_NODE || hPatNode->getType() == INTERPRETATION_NODE)
            continue;

        // postpone the disjunct match for predicates to grounding()
        if (hPatNode->getType() == PREDICATE_NODE) continue;

        // get the corresponding WordNode of the pattern node
        Handle hPatWordNode;
        auto it = m_words.find(hPatNode);
        if (it == m_words.end())
        {
            std::string sPat = hPatNode->getName();
            std::string sPatWord = sPat.substr(0, sPat.find_first_of('@'));

            // Get the WordNode associated with the word
            // (extracted from "word@1234" convention).
            hPatWordNode = m_as->get_handle(WORD_NODE, sPatWord);

            m_words.insert({hPatNode, hPatWordNode});
        }
        else hPatWordNode = it->second;

        // get the corresponding WordInstanceNode of the solution node
        std::string sSoln = hSolnNode->getName();
        Handle hSolnWordInst = m_as->get_handle(WORD_INSTANCE_NODE, sSoln);

        // if the node is a variable, it would have matched to a node with
        // corresponding WordInstanceNode by variable match
        // if the node is not a variable, then the standard node_match would
        // have made it matched to itself, so let's move on and check the rest
        // of the nodes
        if (hSolnWordInst == Handle::UNDEFINED)
            continue;

        // do the actual disjunct match
        if (not disjunct_match(hPatWordNode, hSolnWordInst)) {
            if (m_use_cache) {
                SuRealCache::instance().add_clause_match(pattrn_link_h, grnd_link_h, false);
            }
            return false;
        }
    }

    // store the accepted InterpretationNodes, which will be the targets for the
    // next disconnected clause in the pattern, if any
    m_interp.insert(qTempInterpNodes.begin(), qTempInterpNodes.end());

    if (m_use_cache) {
        SuRealCache::instance().add_clause_match(pattrn_link_h, grnd_link_h, true);
    }

    return true;
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
 * @return           always return false to search for more solutions, unless a
 *                   good enough solution is found
 */
bool SuRealPMCB::grounding(const std::map<Handle, Handle> &var_soln, const std::map<Handle, Handle> &pred_soln)
{
    if (m_use_cache) {
        int cached = SuRealCache::instance().grounding_match(var_soln, pred_soln);
        if (cached >= 0) {
            if (cached == 0) {
                return false;
            } else {
                return true;
            }
        }
    }

    logger().debug("[SuReal] grounding a solution");

    // helper to get the InterpretationNode
    auto getInterpretation = [&](const Handle& h)
    {
        HandleSeq qISet;
        h->getIncomingSetByType(back_inserter(qISet), SET_LINK);

        HandleSeq results;
        for (auto& hSetLink : qISet)
        {
            HandleSeq qN = get_source_neighbors(hSetLink, REFERENCE_LINK);
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
    if (qItprNode.empty()) {
        if (m_use_cache) {
            SuRealCache::instance().add_grounding_match(pred_soln, false); // pred
        }
        return false;
    }

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
        if (std::any_of(shrinked_soln.begin(), shrinked_soln.end(), checker)) {
            if (m_use_cache) {
                SuRealCache::instance().add_grounding_match(var_soln, false); // var
            }
            return false;
        }

        std::string sName = kv.first->getName();
        std::string sWord = sName.substr(0, sName.find_first_of('@'));
        Handle hPatWord = m_as->get_handle(WORD_NODE, sWord);
        Handle hSolnWordInst = m_as->get_handle(WORD_INSTANCE_NODE, kv.second->getName());

        // do a disjunct match for PredicateNodes as well
        if (kv.first->getType() == PREDICATE_NODE and kv.second->getType() == PREDICATE_NODE)
        {
            IncomingSet qLemmaLinks = hPatWord->getIncomingSetByType(LEMMA_LINK);

            // if there is no LemmaLink conntecting to it, it's probably
            // not a lemma, so just do a disjunct match for it
            if (qLemmaLinks.size() == 0)
            {
                // reject it if disjuncts do not match
                if (not disjunct_match(hPatWord, hSolnWordInst)) {
                    if (m_use_cache) {
                        SuRealCache::instance().add_grounding_match(var_soln, false); // var
                    }
                    return false;
                }

                // store the result
                shrinked_soln[kv.first] = kv.second;
            }

            // if it's a lemma, trace all other "forms" of it via LemmaLink
            // and do a disjunct match for each of them
            else
            {
                bool found = false;
                std::set<std::string> qChkWords;

                for (LinkPtr lpll : qLemmaLinks)
                {
                    HandleSeq qOS = lpll->getOutgoingSet();

                    // just in case... double checking
                    if (qOS[0]->getType() != WORD_INSTANCE_NODE)
                        continue;

                    std::string sName = qOS[0]->getName();
                    std::string sWord = sName.substr(0, sName.find_first_of('@'));

                    // Skip if we have seen it before
                    if (std::find(qChkWords.begin(), qChkWords.end(), sWord) != qChkWords.end())
                        continue;
                    else qChkWords.insert(sWord);

                    // make sure the tense matches
                    // first get the tense of the solution instance node
                    // from the InheritanceLink in this form, e.g.
                    // (InheritanceLink
                    //    (PredicateNode "eats@111")
                    //    (DefinedLinguisticConceptNode "present"))
                    std::string sTense;
                    IncomingSet qSolnIS = kv.second->getIncomingSetByType(INHERITANCE_LINK);
                    for (LinkPtr lpInhLk : qSolnIS)
                    {
                        HandleSeq qInhOS = lpInhLk->getOutgoingSet();
                        if (qInhOS[0] == kv.second and
                                qInhOS[1]->getType() == DEFINED_LINGUISTIC_CONCEPT_NODE)
                        {
                            sTense = qInhOS[1]->getName();
                            break;
                        }
                    }

                    // then get the tense of the one in this LemmaLink and see if they match
                    Handle hPatPredNode = m_as->get_handle(PREDICATE_NODE, sName);
                    bool has_tense = false;
                    bool eq_tense = false;
                    if (hPatPredNode != Handle::UNDEFINED)
                    {
                        IncomingSet qPatIS = hPatPredNode->getIncomingSetByType(INHERITANCE_LINK);
                        for (LinkPtr lpInhLk : qPatIS)
                        {
                            HandleSeq qInhOS = lpInhLk->getOutgoingSet();
                            if (qInhOS[0] == hPatPredNode and
                                qInhOS[1]->getType() == DEFINED_LINGUISTIC_CONCEPT_NODE) {
                                has_tense = true;
                                eq_tense = sTense == qInhOS[1]->getName();
                                break;
                            }
                        }
                    }

                    // reject if their tenses don't match
                    if (has_tense and not eq_tense) continue;

                    Handle hWordNode = m_as->get_handle(WORD_NODE, sWord);

                    if (disjunct_match(hWordNode, hSolnWordInst))
                    {
                        Handle hNewPred = m_as->get_handle(PREDICATE_NODE, sWord);

                        if (hNewPred == Handle::UNDEFINED)
                            hNewPred = m_as->add_node(PREDICATE_NODE, sWord);

                        // update the mapping by replacing the lemma
                        // by the one that passed the disjunct match
                        shrinked_soln[hNewPred] = kv.second;
                        found = true;
                        break;
                    }
                }

                // will not consider it as a match if none of them
                // passed the disjunct match
                if (not found) {
                    if (m_use_cache) {
                        SuRealCache::instance().add_grounding_match(var_soln, false); // var
                    }
                    return false;
                }
            }
        }

        else
        {
            // depends on what the input is, clause_match() may not always be called,
            // do the same kind of checking here to make sure the disjuncts match
            if (hPatWord != Handle::UNDEFINED and hSolnWordInst != Handle::UNDEFINED)
            {
                if (disjunct_match(hPatWord, hSolnWordInst))
                    shrinked_soln[kv.first] = kv.second;

                else return false;
            }

            // the above only takes care of the words, for nodes that do not correspond
            // to any words, they should have gone through the node_match/variable_match
            // callbacks. Since they get all the way here, that means they are good,
            // so accept them directly
            shrinked_soln[kv.first] = kv.second;
        }
    }

    HandleSet qSolnSetLinks;

    // get the R2L-SetLinks that are related to these InterpretationNodes
    for (Handle& hItprNode : qItprNode)
    {
        HandleSeq qN = get_target_neighbors(hItprNode, REFERENCE_LINK);
        qN.erase(std::remove_if(qN.begin(), qN.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qN.end());

        // just in case... make sure all the pred_solns exist in the SetLink
        for (auto& hSetLink : qN)
        {
            if (std::all_of(pred_soln.begin(), pred_soln.end(),
                            [&](std::pair<Handle, Handle> soln) { return is_atom_in_tree(hSetLink, soln.second); }))
                qSolnSetLinks.insert(hSetLink);
        }
    }

    // if there are more than one common InterpretationNodes at this point,
    // we should also have more than one SetLinks here, so examine them one by one
    for (Handle hSetLink : qSolnSetLinks)
    {
        bool isGoodEnough = true;

        // extract the leftovers from the solution SetLink
        HandleSeq qLeftover = hSetLink->getOutgoingSet();
        for (auto it = pred_soln.begin(); it != pred_soln.end(); it++)
        {
            auto itc = std::find(qLeftover.begin(), qLeftover.end(), it->second);

            if (itc != qLeftover.end())
                qLeftover.erase(itc);
        }

        // for words in a sentence, they should be linked to their
        // corresponding WordInstanceNodes by a ReferenceLink, for example:
        //   ReferenceLink
        //     ConceptNode "water@123"
        //     WordInstanceNode "water@123"
        auto checker = [] (Handle& h)
        {
            HandleSeq qN = get_target_neighbors(h, REFERENCE_LINK);
            return qN.size() != 1 or qN[0]->getType() != WORD_INSTANCE_NODE;
        };

        HandleSeq qWordInstNodes;
        get_all_nodes(hSetLink, qWordInstNodes, m_use_cache);
        qWordInstNodes.erase(std::remove_if(qWordInstNodes.begin(), qWordInstNodes.end(),
                                            checker), qWordInstNodes.end());

        // check if all of the leftovers of this SetLink are unary -- doesn't
        // form a logical relationship with more than one word of the sentence
        for (Handle& l : qLeftover)
        {
            std::set<UUID> sWordFound;
            HandleSeq qNodes;
            get_all_nodes(l, qNodes, m_use_cache);

            for (const Handle& n : qNodes)
            {
                auto matchWordInst = [&](Handle& w)
                {
                    std::string wordInstName = w->getName();
                    std::string nodeName = n->getName();

                    if (wordInstName.compare(nodeName) == 0)
                    {
                        sWordFound.insert(w.value());
                        return true;
                    }

                    return false;
                };

                std::find_if(qWordInstNodes.begin(), qWordInstNodes.end(), matchWordInst);
            }

            // if it is not a unary link, the solution is not good enough
            if (sWordFound.size() > 1)
            {
                size_t cnt = sWordFound.size();

                // see how many of the words found in the leftover-link have
                // actually been grounded
                for (auto i = var_soln.begin(); i != var_soln.end(); i++)
                    if (std::find(sWordFound.begin(), sWordFound.end(), i->second.value()) != sWordFound.end())
                        cnt--;

                // so it would not be considered as good enough if there are
                // at least one ungrounded word in a leftover-link containing
                // two or more words
                if (cnt > 0)
                {
                    isGoodEnough = false;
                    break;
                }
            }
        }

        // if we find a good enough solution, clear previous (not good enough) ones, if any
        if (isGoodEnough)
            m_results.clear();

        // store the solution; all common InterpretationNode are solutions for this
        // grounding, so store the solution for each InterpretationNode
        // but normally there should be only one InterpretationNode for a
        // given R2L-SetLink
        for (auto n : qItprNode)
        {
            // there could also be multiple solutions for one InterpretationNode,
            // so store them in a vector
            if (m_results.count(n) == 0)
                m_results[n] = std::vector<std::map<Handle, Handle> >();

            logger().debug("[SuReal] grounding Interpreation: %s", n->toShortString().c_str());
            m_results[n].push_back(shrinked_soln);
        }
        if (m_use_cache) {
            //SuRealCache::instance().add_grounding_match(var_soln, pred_soln, true);
            return true;
        } else {
            return isGoodEnough;
        }
    }

    if (m_use_cache) {
        SuRealCache::instance().add_grounding_match(var_soln, pred_soln, false);
    }

    return false;
}

/**
 * Check the disjuncts between two words and see if they match.
 *
 * @param hPatWordNode   a WordNode from the input pattern
 * @param hSolnWordInst  a WordInstanceNode from a potential solution
 * @return               true if matches, false otherwise
 */
bool SuRealPMCB::disjunct_match(const Handle& hPatWordNode, const Handle& hSolnWordInst)
{
    // the source connectors for the solution
    HandleSeq qTargetConns;

    HandleSeq qSolnEvalLinks = get_predicates(hSolnWordInst, LG_LINK_INSTANCE_NODE);

    HandleSeq qLGInstsLeft;
    HandleSeq qLGInstsRight;
    for (Handle& hSolnEvalLink : qSolnEvalLinks)
    {
        HandleSeq qOS = hSolnEvalLink->getOutgoingSet();
        HandleSeq qWordInsts = qOS[1]->getOutgoingSet();

        // divide them into two groups, assuming there are only two WordInstanceNodes in the ListLink
        if (qWordInsts[0] == hSolnWordInst) qLGInstsRight.push_back(hSolnEvalLink);
        if (qWordInsts[1] == hSolnWordInst) qLGInstsLeft.push_back(hSolnEvalLink);
    }

    // helper for sorting EvaluationLinks in reverse word sequence order
    auto sortLeftInsts = [](const Handle& h1, const Handle& h2)
    {
        // get the ListLinks from the EvaluationLinks
        const Handle& hListLink1 = h1->getOutgoingAtom(1);
        const Handle& hListLink2 = h2->getOutgoingAtom(1);

        // get the first WordInstanceNodes from the ListLinks
        const Handle& hWordInst1 = hListLink1->getOutgoingAtom(0);
        const Handle& hWordInst2 = hListLink2->getOutgoingAtom(0);

        // get the NumberNodes from the WordSequenceLinks
        Handle hNumNode1 = get_target_neighbors(hWordInst1, WORD_SEQUENCE_LINK)[0];
        Handle hNumNode2 = get_target_neighbors(hWordInst2, WORD_SEQUENCE_LINK)[0];

        // compare their word sequences
        return hNumNode1->getName() > hNumNode2->getName();
    };

    // helper for sorting EvaluationLinks in word sequence order
    auto sortRightInsts = [](const Handle& h1, const Handle& h2)
    {
        // get the ListLinks from the EvaluationLinks
        const Handle& hListLink1 = h1->getOutgoingAtom(1);
        const Handle& hListLink2 = h2->getOutgoingAtom(1);

        // get the second WordInstanceNodes from the ListLinks
        const Handle& hWordInst1 = hListLink1->getOutgoingAtom(1);
        const Handle& hWordInst2 = hListLink2->getOutgoingAtom(1);

        // get the NumberNodes from the WordSequenceLinks
        Handle hNumNode1 = get_target_neighbors(hWordInst1, WORD_SEQUENCE_LINK)[0];
        Handle hNumNode2 = get_target_neighbors(hWordInst2, WORD_SEQUENCE_LINK)[0];

        // compare their word sequences
        return hNumNode1->getName() < hNumNode2->getName();
    };

    // sort the qLGInstsLeft in reverse word sequence order
    std::sort(qLGInstsLeft.begin(), qLGInstsLeft.end(), sortLeftInsts);

    // sort the qLGInstsRight in word sequence order
    std::sort(qLGInstsRight.begin(), qLGInstsRight.end(), sortRightInsts);

    // get the LG connectors for those in the qLGInstsLeft
    for (Handle& hEvalLink : qLGInstsLeft)
    {
        const Handle& hLinkInstNode = hEvalLink->getOutgoingAtom(0);

        HandleSeq qLGConns = get_all_neighbors(hLinkInstNode, LG_LINK_INSTANCE_LINK);

        // get the first LG connector
        qTargetConns.push_back(qLGConns[0]);
    }

    // get the LG connectors for those in the qLGInstsRight
    for (Handle& hEvalLink : qLGInstsRight)
    {
        const Handle& hLinkInstNode = hEvalLink->getOutgoingAtom(0);

        HandleSeq qLGConns = get_all_neighbors(hLinkInstNode, LG_LINK_INSTANCE_LINK);

        // get the second LG connector
        qTargetConns.push_back(qLGConns[1]);
    }

    // disjuncts of the hPatWordNode
    HandleSeq qDisjuncts;

    // check if we got the disjuncts of the hPatWordNode already,
    // otherwise store them in a map so that we only need to do
    // this disjuncts-getting procedure once
    auto iter = m_disjuncts.find(hPatWordNode);
    if (iter == m_disjuncts.end())
    {
        qDisjuncts = get_target_neighbors(hPatWordNode, LG_DISJUNCT);
        m_disjuncts.insert({hPatWordNode, qDisjuncts});
    }
    else qDisjuncts = iter->second;

    logger().debug("[SuReal] Looking at %d disjuncts of %s", qDisjuncts.size(), hPatWordNode->toShortString().c_str());

    // for each disjunct, get its outgoing set, and match 1-to-1 with qTargetConns
    auto matchHelper = [&](const Handle& hDisjunct)
    {
        std::list<Handle> sourceConns;
        std::list<Handle> targetConns(qTargetConns.begin(), qTargetConns.end());

        // check if hDisjunct is LgAnd or just a lone connector
        if (hDisjunct->getType() == LG_AND)
        {
            const HandleSeq& q = hDisjunct->getOutgoingSet();
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

            // pop only the target connector if we were repeating
            // a multi-conn.
            if (hMultiConn != Handle::UNDEFINED)
            {
                targetConns.pop_front();
                continue;
            }

            // dumb hacky way of checking of the connector is
            // a multi-connector
            if (sourceConns.front()->getArity() == 3)
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

    // reject if the disjuncts do not match
    if (not std::any_of(qDisjuncts.begin(), qDisjuncts.end(), matchHelper))
        return false;

    return true;
}

/**
 * Implement the initiate_search method.
 *
 * Similar to InitiateSearchCB::initiate_search, in which we start search
 * by looking at the thinnest clause with constants.  However, since most clauses
 * for SuReal will have 0 constants, most searches will require looking at all
 * the links.  This implementation improves that by looking at links within a
 * SetLink within a ReferenceLink with a InterpretationNode neightbor, thus
 * limiting the search space.
 *
 * @param pPME       pointer to the PatternMatchEngine
 */
bool SuRealPMCB::initiate_search(PatternMatchEngine* pPME)
{
    // set targets, m_targets should always be a subset of m_interp
    if (m_interp.size() > 0)
    {
        m_targets = m_interp;
        m_interp.clear();
    }

    _search_fail = false;
    if (not _variables->varset.empty())
    {
        bool found = neighbor_search(pPME);
        if (not _search_fail) return found;
    }

    // Not sure quite what triggers this, but there are patterns
    // with no mandatory clauses.
    if (0 ==  _pattern->mandatory.size()) return false;

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
        auto rm = [&](Handle& h)
        {
            if (h->getType() != SET_LINK) return true;

            HandleSeq qN = get_source_neighbors(h, REFERENCE_LINK);
            return not std::any_of(qN.begin(), qN.end(),
                [&](Handle& hn) {
                    bool isInterpNode = hn->getType() == INTERPRETATION_NODE;
                    bool isTarget = m_targets.size() > 0? (m_targets.find(hn) != m_targets.end()) : true;
                    return isInterpNode and isTarget;
                });
        };

        HandleSeq qISet;
        c->getIncomingSet(back_inserter(qISet));

        // erase atoms that are neither a SetLink nor a target
        qISet.erase(std::remove_if(qISet.begin(), qISet.end(), rm), qISet.end());

        if (qISet.size() >= 1)
        {
            size_t maxSize = 0;
            for (Handle& q : qISet)
            {
                size_t s = q->getArity();
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
    }
    return false;
}

/**
 * Override the find_starter_recursive method in InitiateSearchCB.
 *
 * Override find_starter_recursive so that it will not treat VariableNode variables,
 * but instead compare against the variable list.
 *
 * @param h       same as InitiateSearchCB::find_starter_recursive
 * @param depth   same as InitiateSearchCB::find_starter_recursive
 * @param start   same as InitiateSearchCB::find_starter_recursive
 * @param width   same as InitiateSearchCB::find_starter_recursive
 * @return        same as InitiateSearchCB::find_starter_recursive but change
 *                the result if is a variable
 */
Handle SuRealPMCB::find_starter_recursive(const Handle& h, size_t& depth,
                                          Handle& start, size_t& width)
{
    Handle rh = InitiateSearchCB::find_starter_recursive(h, depth, start, width);

    // if the non-VariableNode is actually a variable
    if (m_vars.count(rh) == 1)
        return Handle::UNDEFINED;

    return rh;
}
