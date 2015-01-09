/*
 * SuRealModule.cc
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

#include <opencog/atomspace/AtomSpaceUtils.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/nlp/types/atom_types.h>

#include "SuRealModule.h"
#include "SuRealPMCB.h"


using namespace opencog::nlp;
using namespace opencog;

DECLARE_MODULE(SuRealModule);


/**
 * The constructor for SuRealModule.
 *
 * @param cs   the OpenCog server
 */
SuRealModule::SuRealModule(CogServer& cs) : Module(cs)
{

}

/**
 * The required implementation for the pure virtual init method.
 */
void SuRealModule::init(void)
{
#ifdef HAVE_GUILE
    define_scheme_primitive("sureal-match", &SuRealModule::do_sureal_match, this);
    define_scheme_primitive("sureal-get-mapping", &SuRealModule::do_sureal_get_mapping, this);
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
HandleSeq SuRealModule::do_sureal_match(Handle h)
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

        // try NumberNode if no WordNode found
        if (hWordNode == Handle::UNDEFINED)
        {
            hWordNode = pAS->getHandle(NUMBER_NODE, sWord);

            // if there's no NumberNode neither
            if (hWordNode == Handle::UNDEFINED)
                continue;
        }

        // if no LG dictionary entry
        if (pAS->getNeighbors(hWordNode, false, true, LG_WORD_CSET, false).empty())
            continue;

        sVars.insert(n);
    }

    logger().debug("[SuReal] starting pattern matcher");

    SuRealPMCB pmcb(pAS, sVars);
    m_pme.match(&pmcb, sVars, qClauses, qNegs);

    // get the results out of pmcb
    m_results = pmcb.m_results;

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
 * @return    a list of two lists; the first list will be the nodes extracted
 *            from the original query, the second list will the the
 *            corresponding mapping for each node.
 */
HandleSeqSeq SuRealModule::do_sureal_get_mapping(Handle h)
{
    // no pattern matcher result or no result mapping to the Handle h
    if (m_results.empty() || m_results.count(h) == 0)
        return HandleSeqSeq();

    // construct the result
    std::map<Handle, Handle>& mapping = m_results[h];

    HandleSeq qKeys, qVars;

    for (auto& kv : mapping)
    {
        qKeys.push_back(kv.first);
        qVars.push_back(kv.second);
    }

    HandleSeqSeq results;
    results.push_back(qKeys);
    results.push_back(qVars);

    return results;
}

