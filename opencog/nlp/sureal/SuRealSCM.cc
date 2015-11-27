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

#include <opencog/util/Logger.h>
#include <opencog/atomutils/AtomUtils.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/atoms/pattern/PatternUtils.h>
#include <opencog/atoms/pattern/PatternLink.h>
#include <opencog/guile/SchemePrimitive.h>
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
#endif
}

/**
 * Implement the "sureal-match" scheme primitive.
 *
 * Uses the pattern matcher to find all InterpretationNodes whoses
 * corresponding SetLink contains a structure similar to the input,
 * taking into accounting each ConceptNode/PredicateNode's corresponding
 * WordNode and their LG dictionary entry.  Then construct a compact
 * structure indicating the mappings for each InterpretationNode.
 *
 * @param h   a SetLink contains the atoms which will become the clauses
 * @return    a list of the form returned by sureal_get_mapping, but spanning
 *            multiple InterpretationNode
 */
HandleSeqSeq SuRealSCM::do_sureal_match(Handle h)
{
#ifdef HAVE_GUILE
    // only accept SetLink
    if (h->getType() != SET_LINK)
        return HandleSeqSeq();

    AtomSpace* pAS = SchemeSmob::ss_get_env_as("sureal-match");

    std::set<Handle> sVars;

    // Extract the graph under the SetLink; this is done so that the content
    // of the SetLink could be matched to another SetLink with differnet arity.
    // It is possible to keep the clauses in a SetLink and override the PM's
    // link_match() callback to skip SetLink's arity check , but that would
    // be assuming R2L will never use SetLink for other purposes.
    HandleSeq qClauses = pAS->get_outgoing(h);

    // get all the nodes to be treated as variable in the Pattern Matcher
    // XXX perhaps it's better to write a eval_q in SchemeEval to convert
    //     a scm list to HandleSeq, so can just use the scheme utilities?
    UnorderedHandleSet allNodes = get_all_unique_nodes(h);

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

        // special treatment for DefinedLinguisticConceptNode and
        // DefinedLinguisticPredicateNode, do not treat them as variables
        // because they are not actual words of a sentence.
        if (n->getType() == DEFINED_LINGUISTIC_CONCEPT_NODE || n->getType() == DEFINED_LINGUISTIC_PREDICATE_NODE)
           continue;

        std::string sName = pAS->get_name(n);
        std::string sWord = sName.substr(0, sName.find_first_of('@'));
        Handle hWordNode = pAS->get_handle(WORD_NODE, sWord);

        // no WordNode found
        if (hWordNode == Handle::UNDEFINED)
            continue;

        // if no LG dictionary entry
        if (get_neighbors(hWordNode, false, true, LG_WORD_CSET, false).empty())
            continue;

        sVars.insert(n);
    }

    SuRealPMCB pmcb(pAS, sVars);
    PatternLinkPtr slp(createPatternLink(sVars, qClauses));
    slp->satisfy(pmcb);

    HandleSeq keys;

    // construct the list of InterpretationNode to be returned
    for (auto& r : pmcb.m_results)
        keys.push_back(r.first);

    // sort the InterpretationNode bases on the size of the corresponding
    // SetLink; the idea is that the larger the size, the more stuff in
    // the SetLink that are not part of the query's clauses
    auto itprComp = [&pAS](const Handle& hi, const Handle& hj)
    {
        // get the corresponding SetLink
        HandleSeq qi = get_neighbors(hi, false, true, REFERENCE_LINK, false);
        HandleSeq qj = get_neighbors(hj, false, true, REFERENCE_LINK, false);
        qi.erase(std::remove_if(qi.begin(), qi.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qi.end());
        qj.erase(std::remove_if(qj.begin(), qj.end(), [](Handle& h) { return h->getType() != SET_LINK; }), qj.end());

        // assuming each InterpretationNode is only linked to one SetLink
        // and compare using arity
        return pAS->get_arity(qi[0]) < pAS->get_arity(qj[0]);
    };

    std::sort(keys.begin(), keys.end(), itprComp);

    HandleSeqSeq results;

    for (auto& k : keys)
    {
        HandleSeqSeq mappings = sureal_get_mapping(k, pmcb.m_results[k]);
        results.insert(results.end(), mappings.begin(), mappings.end());
    }

    return results;

#else
    return HandleSeqSeq();
#endif
}

/**
 * Construct the proper structure for an InterpretationNode's mapping.
 *
 * Takes an InterpretationNode and its mappings, and return the node-to-node
 * mapping in a list-of-list of the form:
 *
 * (
 *  ((InterpretationNode "sentence@1234") nodes from original query)
 *  ((InterpretationNode "sentence@1234") 1st mapping of the corresponding nodes)
 *  ((InterpretationNode "sentence@1234") 2nd mapping of the corresponding nodes)
 * )
 *
 * For example, it could be
 *
 * (
 *  ((InterpretationNode "sentence@1234") (ConceptNode "she") (PredicateNode "runs"))
 *  ((InterpretationNode "sentence@1234") (ConceptNode "he@1234") (PredicateNode "walks@1234"))
 * )
 *
 * if there is only one mapping for a sentence.
 *
 * @param h          an InterpretationNode with the mapping
 * @param mappings   the node-to-node mapping
 * @return           a list-of-list of the above structure
 */
HandleSeqSeq SuRealSCM::sureal_get_mapping(Handle& h, std::vector<std::map<Handle, Handle> >& mappings)
{
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

    // insert the InterpretationNode to the start of each list
    for (auto& v : results)
        v.insert(v.begin(), h);

    return results;
}


void opencog_nlp_sureal_init(void)
{
    static SuRealSCM sureal;
}
