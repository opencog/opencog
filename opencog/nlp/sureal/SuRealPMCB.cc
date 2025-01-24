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

#include <list>

#include <opencog/atoms/core/FindUtils.h>
#include <opencog/atoms/pattern/PatternTerm.h>
#include <opencog/query/PatternMatchEngine.h>
#include <opencog/neighbors/GetPredicates.h>
#include <opencog/neighbors/Neighbors.h>
#include <opencog/nlp/types/atom_types.h>
#include <opencog/nlp/oc-types/atom_types.h>
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
    InitiateSearchMixin(pAS),
    TermMatchMixin(pAS),
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

    logger().debug("[SuReal] In variable_match, trying to ground:\n%sto\n%s",
        hPat->to_short_string().c_str(), hSoln->to_short_string().c_str());

    bool answer;

    // Reject if the solution is not of the same type.
    if (hPat->get_type() != hSoln->get_type()) {
        logger().debug("[SuReal] In variable_match, type mismatch!");
        answer = false;
    } else {
        // VariableNode can be matched to any VariableNode, similarly
        // for InterpretationNode
        if (hPat->get_type() == VARIABLE_NODE or
            hPat->get_type() == INTERPRETATION_NODE) {
            answer = true;
        } else {
            std::string sSoln = hSoln->get_name();
            // get the corresponding WordInstanceNode for hSoln
            Handle hSolnWordInst = m_as->get_handle(WORD_INSTANCE_NODE, sSoln);
            // no WordInstanceNode? reject!
            if (hSolnWordInst == Handle::UNDEFINED) {
                logger().debug(
                    "[SuReal] In variable_match, no word instance found!");
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
   if (h->is_node())
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

    logger().debug("[SuReal] In clause_match, trying to ground:\n%sto\n%s",
        pattrn_link_h->to_short_string().c_str(),
            grnd_link_h->to_short_string().c_str());

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
                bool isInterpNode = hn->get_type() == INTERPRETATION_NODE;
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

        logger().debug("[SuReal] In clause_match, no target InterpretationNode found!");
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

        logger().debug("[SuReal] In clause_match, size mismatch!");
        return false;
    }

    // Compare the disjuncts for each of the nodes.
    for (size_t i = 0; i < qAllPatNodes.size(); i++)
    {
        Handle& hPatNode = qAllPatNodes[i];
        Handle& hSolnNode = qAllSolnNodes[i];

        // move on if the pattern node is a VariableNode or an InperpretationNode
        if (hPatNode->get_type() == VARIABLE_NODE || hPatNode->get_type() == INTERPRETATION_NODE)
            continue;

        // postpone the disjunct match for predicates to grounding()
        if (hPatNode->get_type() == PREDICATE_NODE) continue;

        // get the corresponding WordNode of the pattern node
        Handle hPatWordNode;
        auto it = m_words.find(hPatNode);
        if (it == m_words.end())
        {
            HandleSeq neighbor_win = get_target_neighbors(hPatNode, REFERENCE_LINK);

            if (neighbor_win.size() != 0)
            {
                HandleSeq neighbor_wn = get_target_neighbors(neighbor_win[0], REFERENCE_LINK);
                hPatWordNode = neighbor_wn[0];

            }
            else
            {
                string sPat = hPatNode->get_name();
                string sPatWord = sPat.substr(0, sPat.find_first_of('@'));
                sPatWord = sPatWord.substr(0, sPatWord.find_last_of('.'));
                hPatWordNode = m_as->get_handle(WORD_NODE, sPatWord);
            }
        }
        else hPatWordNode = it->second;

        // get the corresponding WordInstanceNode of the solution node
        std::string sSoln = hSolnNode->get_name();
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

            logger().debug("[SuReal] In clause_match, disjuncts mismatch:\n%s%s",
                hPatWordNode->to_short_string().c_str(),
                    hSolnWordInst->to_short_string().c_str());

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
bool SuRealPMCB::propose_grounding(const HandleMap &var_soln, const HandleMap &pred_soln)
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

    // helper to get the InterpretationNode
    auto getInterpretation = [&](const Handle& h)
    {
        HandleSeq qISet;
        h->getIncomingSetByType(back_inserter(qISet), SET_LINK);

        HandleSeq results;
        for (auto& hSetLink : qISet)
        {
            HandleSeq qN = get_source_neighbors(hSetLink, REFERENCE_LINK);
            qN.erase(std::remove_if(qN.begin(), qN.end(), [](Handle& h) { return h->get_type() != INTERPRETATION_NODE; }), qN.end());

            results.insert(results.end(), qN.begin(), qN.end());
        }

        return results;
    };

    HandleSeq qItprNode = getInterpretation(pred_soln.begin()->second);
    std::sort(qItprNode.begin(), qItprNode.end());

    string debug_str = (pred_soln.begin()->second)->to_short_string();

    // try to find a common InterpretationNode to the solutions
    // i.e. all the solution-clauses have to come from the same interpretation (of a sentence)
    for (HandleMap::const_iterator it = ++pred_soln.begin(); it != pred_soln.end(); ++it)
    {
        debug_str.append(it->second->to_short_string());

        HandleSeq thisSeq = getInterpretation(it->second);
        std::sort(thisSeq.begin(), thisSeq.end());

        HandleSeq overlapSeq;
        std::set_intersection(qItprNode.begin(), qItprNode.end(), thisSeq.begin(), thisSeq.end(), std::back_inserter(overlapSeq));

        qItprNode = overlapSeq;
    }

    logger().debug("[SuReal] In grounding, trying to find a common interpretation between:\n%s",
      debug_str.c_str());

    // no common InterpretationNode, ignore this grounding
    if (qItprNode.empty()) {
        if (m_use_cache) {
            SuRealCache::instance().add_grounding_match(pred_soln, false); // pred
        }

        logger().debug("[SuReal] In grounding, no common InterpretationNode found!");
        return false;
    }

    // shrink var_soln to only contain solution for the variables
    HandleMap shrinked_soln;

    for (const auto& kv : var_soln)
    {
        if (m_vars.count(kv.first) == 0)
            continue;

        auto checker = [&](const HandlePair& nkv)
        {
            return kv.second == nkv.second;
        };

        // if another variable already map to the same solution, discard this grounding
        // because each variable should map to its own unique solution
        if (std::any_of(shrinked_soln.begin(), shrinked_soln.end(), checker)) {
            if (m_use_cache) {
                SuRealCache::instance().add_grounding_match(var_soln, false); // var
            }

            logger().debug("[SuReal] In grounding, can't ground it to the same solution!");
            return false;
        }

        std::string sName = kv.first->get_name();

        Handle hPatWord;
        HandleSeq neighbor_win = get_target_neighbors(kv.first, REFERENCE_LINK);
        if (neighbor_win.size() != 0 )
        {
            HandleSeq neighbor_wn = get_target_neighbors(neighbor_win[0], REFERENCE_LINK);
            hPatWord = neighbor_wn[0];
        }
        else
        {
            string sWord = sName.substr(0, sName.find_first_of('@'));
            sWord = sWord.substr(0, sWord.find_last_of('.'));
            hPatWord = m_as->get_handle(WORD_NODE, sWord);
        }

        Handle hSolnWordInst = m_as->get_handle(WORD_INSTANCE_NODE, kv.second->get_name());
        // do a disjunct match for PredicateNodes as well
        if (kv.first->get_type() == PREDICATE_NODE and kv.second->get_type() == PREDICATE_NODE)
        {
            IncomingSet qLemmaLinks = hPatWord->getIncomingSetByType(LEMMA_LINK);

            // if there is no LemmaLink conntecting to it, it's probably
            // not a lemma, so just do a disjunct match for it
            if (qLemmaLinks.size() == 0)
            {
                logger().debug("[SuReal] In grounding, this predicate is probably not a lemma: %s",
                  hPatWord->to_short_string().c_str());

                // reject it if disjuncts do not match
                if (not disjunct_match(hPatWord, hSolnWordInst)) {
                    if (m_use_cache) {
                        SuRealCache::instance().add_grounding_match(var_soln, false); // var
                    }

                    logger().debug("[SuReal] In grounding, disjunct mismatch!\n%s%s",
                        hPatWord->to_short_string().c_str(),
                            hSolnWordInst->to_short_string().c_str());

                    return false;
                }

                // store the result
                shrinked_soln[kv.first] = kv.second;
            }

            // if it's a lemma, trace all other "forms" of it via LemmaLink
            // and do a disjunct match for each of them
            else
            {
                logger().debug("[SuReal] In grounding, this predicate is a lemma: %s",
                  hPatWord->to_short_string().c_str());

                bool found = false;

                for (const Handle& lpll : qLemmaLinks)
                {
                    HandleSeq qOS = lpll->getOutgoingSet();

                    // just in case... double checking
                    if (qOS[0]->get_type() != WORD_INSTANCE_NODE)
                        continue;

                    std::string sName = qOS[0]->get_name();
                    std::string sWord = get_target_neighbors(qOS[0], REFERENCE_LINK)[0]->get_name();

                    // make sure the tense matches
                    // first get the tense of the solution instance node
                    // from the InheritanceLink in this form, e.g.
                    // (InheritanceLink
                    //    (PredicateNode "eats@111")
                    //    (DefinedLinguisticConceptNode "present"))
                    std::string sTense;
                    IncomingSet qSolnIS = kv.second->getIncomingSetByType(INHERITANCE_LINK);
                    for (const Handle& lpInhLk : qSolnIS)
                    {
                        HandleSeq qInhOS = lpInhLk->getOutgoingSet();
                        if (qInhOS[0] == kv.second and
                                qInhOS[1]->get_type() == DEFINED_LINGUISTIC_CONCEPT_NODE)
                        {
                            sTense = qInhOS[1]->get_name();

                            logger().debug("[SuReal] In grounding, tense of %sis %s",
                                kv.second->to_short_string().c_str(), sTense.c_str());

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
                        for (const Handle& lpInhLk : qPatIS)
                        {
                            HandleSeq qInhOS = lpInhLk->getOutgoingSet();
                            if (qInhOS[0] == hPatPredNode and
                                qInhOS[1]->get_type() == DEFINED_LINGUISTIC_CONCEPT_NODE) {
                                has_tense = true;
                                eq_tense = sTense == qInhOS[1]->get_name();

                                logger().debug("[SuReal] In grounding, tense of %sis %s",
                                    hPatPredNode->to_short_string().c_str(),
                                        qInhOS[1]->get_name().c_str());

                                break;
                            }
                        }
                    }

                    // reject if their tenses don't match
                    if (has_tense and not eq_tense)
                    {
                        logger().debug("[SuReal] In grounding, tense mismatch!");
                        continue;
                    }

                    Handle hWordNode = m_as->get_handle(WORD_NODE, sWord);

                    if (disjunct_match(hWordNode, hSolnWordInst))
                    {
                        Handle hNewPred = m_as->get_handle(PREDICATE_NODE, sWord);

                        if (hNewPred == Handle::UNDEFINED)
                            hNewPred = m_as->add_node(PREDICATE_NODE, std::move(sWord));

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

                    logger().debug("[SuReal] In grounding, no matching disjunct found!");
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

                else
                {
                    logger().debug("[SuReal] In grounding, disjunct mismatch!\n%s\n%s",
                        hPatWord->to_short_string().c_str(),
                            hSolnWordInst->to_short_string().c_str());

                    return false;
                }
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
        qN.erase(std::remove_if(qN.begin(), qN.end(), [](Handle& h) { return h->get_type() != SET_LINK; }), qN.end());

        // just in case... make sure all the pred_solns exist in the SetLink
        for (auto& hSetLink : qN)
        {
            if (std::all_of(pred_soln.begin(), pred_soln.end(),
                            [&](HandlePair soln) { return is_atom_in_tree(hSetLink, soln.second); }))
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
            return qN.size() != 1 or qN[0]->get_type() != WORD_INSTANCE_NODE;
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
                    std::string wordInstName = w->get_name();
                    std::string nodeName = n->get_name();

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
                    logger().debug("[SuReal] In grounding, solution is not good enough!");
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
                m_results[n] = HandleMapSeq();

            logger().debug("[SuReal] grounding Interpreation: %s", n->to_short_string().c_str());
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
    logger().debug("[SuReal] In disjunct_match, checking disjuncts for:\n%s%s",
        hPatWordNode->to_short_string().c_str(),
            hSolnWordInst->to_short_string().c_str());

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
        return hNumNode1->get_name() > hNumNode2->get_name();
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
        return hNumNode1->get_name() < hNumNode2->get_name();
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

    logger().debug("[SuReal] Looking at %d disjuncts of %s", qDisjuncts.size(), hPatWordNode->to_short_string().c_str());

    // for each disjunct, get its outgoing set, and match 1-to-1 with qTargetConns
    auto matchHelper = [&](const Handle& hDisjunct)
    {
        std::list<Handle> sourceConns;
        std::list<Handle> targetConns(qTargetConns.begin(), qTargetConns.end());

        // check if hDisjunct is LgAnd or just a lone connector
        if (hDisjunct->get_type() == LG_AND)
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
            if (sourceConns.front()->get_arity() == 3)
                hMultiConn = sourceConns.front();

            sourceConns.pop_front();
            targetConns.pop_front();
        }

        // check if both source and target are used up
        if (not sourceConns.empty() or not targetConns.empty())
            return false;

        logger().debug("[SuReal] " + hDisjunct->to_short_string() + " passed!");

        return true;
    };

    // reject if the disjuncts do not match
    if (not std::any_of(qDisjuncts.begin(), qDisjuncts.end(), matchHelper))
        return false;

    return true;
}

/**
 * Implement the perform_search method.
 *
 * Similar to InitiateSearchMixin::perform_search, in which we start search
 * by looking at the thinnest clause with constants.  However, since most clauses
 * for SuReal will have 0 constants, most searches will require looking at all
 * the links.  This implementation improves that by looking at links within a
 * SetLink within a ReferenceLink with a InterpretationNode neightbor, thus
 * limiting the search space.
 *
 * @param pPME       pointer to the PatternMatchEngine
 */
bool SuRealPMCB::perform_search(PatternMatchCallback& pmc)
{
    // set targets, m_targets should always be a subset of m_interp
    if (m_interp.size() > 0)
    {
        m_targets = m_interp;
        m_interp.clear();
    }

    if (not _variables->varset.empty())
    {
        if (setup_neighbor_search(_pattern->pmandatory))
            return choice_loop(pmc, "sssssss neighbor_search uuuuuuuu");
    }

    // Not sure quite what triggers this, but there are patterns
    // with no mandatory clauses.
    if (0 ==  _pattern->pmandatory.size()) return false;

    // Reaching here means no constants, so do some search space
    // reduction here
    PatternTermPtr root_clause = _pattern->pmandatory[0];
    PatternTermPtr bestClause = root_clause;

    logger().debug("[SuReal] Start pred is: %s",
                   bestClause->to_full_string().c_str());

    // keep only links of the same type as bestClause and
    // have linkage to InterpretationNode
    HandleSeq qCandidate;
    m_as->get_handles_by_type(qCandidate, bestClause->getHandle()->get_type());

    // selected candidates, a subset of qCandidate
    std::vector<CandHandle> sCandidate;

    for (auto& c : qCandidate)
    {
        auto rm = [&](Handle& h)
        {
            if (h->get_type() != SET_LINK) return true;

            HandleSeq qN = get_source_neighbors(h, REFERENCE_LINK);
            return not std::any_of(qN.begin(), qN.end(),
                [&](Handle& hn) {
                    bool isInterpNode = hn->get_type() == INTERPRETATION_NODE;
                    bool isTarget = m_targets.size() > 0? (m_targets.find(hn) != m_targets.end()) : true;
                    return isInterpNode and isTarget;
                });
        };

        HandleSeq qISet = c->getIncomingSet();

        // erase atoms that are neither a SetLink nor a target
        qISet.erase(std::remove_if(qISet.begin(), qISet.end(), rm), qISet.end());

        if (qISet.size() >= 1)
        {
            size_t maxSize = 0;
            for (Handle& q : qISet)
            {
                size_t s = q->get_arity();
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

    PatternMatchEngine pme(pmc);
    pme.set_pattern(*_variables, *_pattern);
    for (auto& c : sCandidate)
    {
        logger().debug("[SuReal] Loop candidate: %s", c.handle->to_short_string().c_str());

        if (pme.explore_neighborhood(bestClause, c.handle, root_clause))
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
Handle SuRealPMCB::find_starter_recursive(const PatternTermPtr& ptm, size_t& depth,
                                          PatternTermPtr& start, size_t& width)
{
    Handle rh = InitiateSearchMixin::find_starter_recursive(ptm, depth, start, width);

    // if the non-VariableNode is actually a variable
    if (m_vars.count(rh) == 1)
        return Handle::UNDEFINED;

    return rh;
}
