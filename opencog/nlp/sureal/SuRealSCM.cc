/*
 * SuRealSCM.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
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

#include <opencog/atomspace/AtomSpaceUtils.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/query/PatternUtils.h>
#include <opencog/nlp/types/atom_types.h>

#include "SuRealSCM.h"
#include "SuRealPMCB.h"


using namespace opencog::nlp;
using namespace opencog;


/**
 * The constructor for SuRealSCM.
 */
SuRealSCM::SuRealSCM()
{
    static bool is_init = false;
    if (is_init) return;
    is_init = true;
    scm_with_guile(init_in_guile, this);
}

/**
 * Init function for using with scm_with_guile.
 *
 * Creates the sureal scheme module and uses it by default.
 *
 * @param self   pointer to the SuRealSCM object
 * @return       null
 */
void* SuRealSCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog nlp sureal", init_in_module, self);
    scm_c_use_module("opencog nlp sureal");
    return NULL;
}

/**
 * The main function for defining stuff in the sureal scheme module.
 *
 * @param data   pointer to the SuRealSCM object
 */
void SuRealSCM::init_in_module(void* data)
{
    SuRealSCM* self = (SuRealSCM*) data;
    self->init();
}

/**
 * The main init function for the SuRealSCM object.
 */
void SuRealSCM::init()
{
#ifdef HAVE_GUILE
    define_scheme_primitive("sureal-match", &SuRealSCM::do_sureal_match, this, "nlp sureal");
    define_scheme_primitive("sureal-get-mapping", &SuRealSCM::do_sureal_get_mapping, this, "nlp sureal");
#endif
}

/**
 * Implement the "sureal-match" scheme primitive.
 *
 * Uses the pattern matcher to find all InterpretationNodes whoses
 * corresponding SetLink contains a structure similar to the input,
 * taking into accounting each ConceptNode/PredicateNode's corresponding
 * WordNode and their LG dictionary entry.
 *
 * @param h   a SetLink contains the atoms which will become the clauses
 * @return    a sequence of InterpretationNodes ranked with best result first
 */
HandleSeq SuRealSCM::do_sureal_match(Handle h)
{
#ifdef HAVE_GUILE
    // clear the old results on new "sureal-match" to sync with "sureal-get-mapping"
    m_results.clear();

    // only accept SetLink
    if (h->getType() != SET_LINK)
        return HandleSeq();

    AtomSpace* pAS = SchemeSmob::ss_get_env_as("sureal-match");

    std::set<Handle> sVars;
    HandleSeq qNegs;

    // Extract the graph under the SetLink; this is done so that the content
    // of the SetLink could be matched to another SetLink with differnet arity.
    // It is possible to keep the clauses in a SetLink and override the PM's
    // link_match() callback to skip SetLink's arity check , but that would
    // be assuming R2L will never use SetLink for other purposes.
    HandleSeq qClauses = pAS->getOutgoing(h);

    // get all the nodes to be treated as variable in the Pattern Matcher
    // XXX perhaps it's better to write a eval_q in SchemeEval to convert
    //     a scm list to HandleSeq, so can just use the scheme utilities?
    UnorderedHandleSet allNodes = AtomSpaceUtils::getAllUniqueNodes(pAS, h);

    // isolate which nodes are actually words, and which are not; all words
    // need to become variable for the Pattern Matcher
    for (auto& n : allNodes)
    {
        // special treatment for InterpretationNode and VariableNode, treating
        // them as variables;  this is because we have
        //    (InterpreationNode "MicroplanningNewSentence")
        // from the microplanner that should be matched to any InterpretationNode
        if (n->getType() == INTERPRETATION_NODE || n->getType() == VARIABLE_NODE)
        {
            sVars.insert(n);
            continue;
        }

        std::string sName = pAS->getName(n);
        std::string sWord = sName.substr(0, sName.find_first_of('@'));
        Handle hWordNode = pAS->getHandle(WORD_NODE, sWord);

        // no WordNode found
        if (hWordNode == Handle::UNDEFINED)
            continue;

        // if no LG dictionary entry
        if (pAS->getNeighbors(hWordNode, false, true, LG_WORD_CSET, false).empty())
            continue;

        sVars.insert(n);
    }

    // separate the disconnected clauses (this will happen often with SuReal)
    std::set<HandleSeq> connectedClauses;
    get_connected_components(sVars, qClauses, connectedClauses);

    logger().debug("[SuReal] Found %d disconnected components", connectedClauses.size());

    std::map<Handle, std::vector<std::map<Handle, Handle> > > collector;

    // call the pattern matcher on each set of disconnected commponents
    for (auto& c : connectedClauses)
    {
        logger().debug("[SuReal] starting pattern matcher");

        // copy the clause for passing const stuff to non-const argument list
        HandleSeq qClause(c);

        SuRealPMCB pmcb(pAS, sVars);
        m_pme.match(&pmcb, sVars, qClause, qNegs);

        // no pattern matcher result
        if (pmcb.m_results.empty())
            return HandleSeq();

        // first disconnected component & result? add it all
        if (collector.empty())
        {
            collector = pmcb.m_results;
            continue;
        }

        // if we are checking subsequent disconnected components, we want to
        // make sure the new results can be merged with results from previously
        // checked components; ie. only keep those with common
        // InterpretationNode & no overlapping mappings
        for (auto it = collector.begin(); it != collector.end(); )
        {
            // no common Interpretation, erase the old results as it can no
            // longer be satisfied
            if (pmcb.m_results.count(it->first) == 0)
            {
                logger().debug("[SuReal] Discarding a result for %s", it->first->toShortString().c_str());

                it = collector.erase(it);
                continue;
            }

            auto& existingMaps = it->second;                // a vector of previous mappings
            auto& appendMaps = pmcb.m_results[it->first];   // a vector of unmerged mappings
            std::vector<std::map<Handle, Handle> > newMaps;

            // check all combinations of all the old mappings to the new
            for (auto& em : existingMaps)
            {
                for (auto& am: appendMaps)
                {
                    // at this point, we know nothing in em & am would map the
                    // same variable because they are from two disconnected
                    // clauses.  Instead, we want to check to make sure no two
                    // variables get mapping to the same node.
                    auto checker = [&](const std::pair<Handle, Handle>& ekv)
                    {
                        return std::any_of(am.begin(), am.end(),
                                           [&](const std::pair<Handle, Handle>& akv) { return ekv.second == akv.second; });
                    };

                    if (std::any_of(em.begin(), em.end(), checker))
                        continue;

                    // clone the old mapping and append the new
                    std::map<Handle, Handle> newMap(em);
                    newMap.insert(am.begin(), am.end());

                    // add as one mapping of one InterpretationNode
                    newMaps.push_back(newMap);
                }
            }

            // if none of the mappings can be merged, the interpretation is bad
            if (newMaps.empty())
            {
                logger().debug("[SuReal] Discarding a result for %s", it->first->toShortString().c_str());

                it = collector.erase(it);
                continue;
            }

            // replace the old results with the new appended ones
            it->second = newMaps;

            // go to the next InterpretationNode
            ++it;
        }

        // if there's no way to connect the disconnected components
        if (collector.empty())
            return HandleSeq();
    }

    // get the results out of pmcb
    m_results = collector;

    HandleSeq results;

    // construct the list of InterpretationNode to be returned
    for (auto& r : m_results)
        results.push_back(r.first);

    // sort the InterpretationNode bases on the size of the corresponding
    // SetLink; the idea is that the larger the size, the more stuff in
    // the SetLink that are not part of the query's clauses
    auto itprComp = [&pAS](const Handle& hi, const Handle& hj)
    {
        // get the corresponding SetLink
        HandleSeq qi = pAS->getNeighbors(hi, false, true, REFERENCE_LINK, false);
        HandleSeq qj = pAS->getNeighbors(hj, false, true, REFERENCE_LINK, false);
        qi.erase(std::remove_if(qi.begin(), qi.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qi.end());
        qj.erase(std::remove_if(qj.begin(), qj.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qj.end());

        // assuming each InterpretationNode is only linked to one SetLink
        // and compare using arity
        return pAS->getArity(qi[0]) < pAS->getArity(qj[0]);
    };

    std::sort(results.begin(), results.end(), itprComp);

    return results;

#else
    return HandleSeq();
#endif
}

/**
 * Get the corresponding node mapping for a matched InterpretationNode.
 *
 * Defines a "sureal-get-mapping" primitive that could be called after a
 * successful run of "sureal-match".  Takes an InterpretationNode returned
 * by "sureal-match" as input, and returns the complete node-to-node mapping.
 *
 * @param h   an InterpretationNode returned by "sureal-match"
 * @return    a list of lists; the first list will be the nodes extracted
 *            from the original query, the subsequent lists will the the
 *            corresponding mapping for each node.
 */
HandleSeqSeq SuRealSCM::do_sureal_get_mapping(Handle h)
{
    // no pattern matcher result or no result mapping to the Handle h
    if (m_results.empty() || m_results.count(h) == 0)
        return HandleSeqSeq();

    // construct the result
    std::vector<std::map<Handle, Handle> >& mappings = m_results[h];

    logger().debug("[SuReal] %d mapping(s) for %s", mappings.size(), h->toShortString().c_str());

    HandleSeq qKeys, qVars;

    for (auto& kv : mappings[0])
    {
        qKeys.push_back(kv.first);
        qVars.push_back(kv.second);
    }

    HandleSeqSeq results;
    results.push_back(qKeys);
    results.push_back(qVars);

    // if there are more than one mapping, loop thru them
    for (unsigned i = 1; i < mappings.size(); ++i)
    {
        std::map<Handle, Handle>& mapping = mappings[i];

        qVars.clear();

        // making sure to use the same ordering of keys as the first mapping
        for (auto& key : qKeys)
            qVars.push_back(mapping[key]);

        results.push_back(qVars);
    }

    return results;
}


void opencog_nlp_sureal_init(void)
{
    static SuRealSCM sureal;
}
