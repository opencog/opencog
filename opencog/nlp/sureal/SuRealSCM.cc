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
#include <opencog/atoms/core/FindUtils.h>
#include <opencog/atoms/pattern/PatternUtils.h>
#include <opencog/atoms/pattern/PatternLink.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/neighbors/Neighbors.h>
#include <opencog/nlp/types/atom_types.h>
#include <opencog/nlp/oc-types/atom_types.h>

#include "SuRealSCM.h"
#include "SuRealPMCB.h"
#include "SuRealCache.h"


using namespace opencog::nlp;
using namespace opencog;

using namespace std;


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
    define_scheme_primitive("sureal-match", &SuRealSCM::do_non_cached_sureal_match, this, "nlp sureal");
    define_scheme_primitive("cached-sureal-match", &SuRealSCM::do_cached_sureal_match, this, "nlp sureal");
    define_scheme_primitive("reset-cache", &SuRealSCM::reset_cache, this, "nlp sureal");
#endif
}


/**
 * Get all the nodes within a link and its sublinks.
 *
 * @param h     the top level link
 * @return      a HandleSeq of nodes
 */
static void get_all_unique_nodes(const Handle& h,
                                 UnorderedHandleSet& node_set)
{
   if (h->is_node())
   {
      node_set.insert(h);
      return;
   }

   for (const Handle& o : h->getOutgoingSet())
      get_all_unique_nodes(o, node_set);
}

/**
 * Implement the "reset-sureal-cache" scheme primitive.
 *
 */
void SuRealSCM::reset_cache(void)
{
    SuRealCache::instance().reset();
}

/**
 * Implement the "cached-sureal-match" scheme primitive.
 *
 */
HandleSeqSeq SuRealSCM::do_cached_sureal_match(Handle h)
{
    return do_sureal_match(h, true);
}

/**
 * Implement the "sureal-match" scheme primitive.
 *
 */
HandleSeqSeq SuRealSCM::do_non_cached_sureal_match(Handle h)
{
    return do_sureal_match(h, false);
}

/**
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
HandleSeqSeq SuRealSCM::do_sureal_match(Handle h, bool use_cache)
{
#ifdef HAVE_GUILE
    // only accept SetLink
    if (h->get_type() != SET_LINK)
        return HandleSeqSeq();

    AtomSpacePtr asp = SchemeSmob::ss_get_env_as("sureal-match");
    AtomSpace* pAS = asp.get();

    HandleSet sVars;

    // Extract the graph under the SetLink; this is done so that the content
    // of the SetLink could be matched to another SetLink with differnet arity.
    // It is possible to keep the clauses in a SetLink and override the PM's
    // link_match() callback to skip SetLink's arity check , but that would
    // be assuming R2L will never use SetLink for other purposes.
    const HandleSeq& qClauses = h->getOutgoingSet();

    // get all the nodes to be treated as variable in the Pattern Matcher
    // XXX perhaps it's better to write a eval_q in SchemeEval to convert
    //     a scm list to HandleSeq, so can just use the scheme utilities?
    UnorderedHandleSet allNodes;
    get_all_unique_nodes(h, allNodes);

    // isolate which nodes are actually words, and which are not; all words
    // need to become variable for the Pattern Matcher
    for (auto& n : allNodes)
    {
        // special treatment for InterpretationNode and VariableNode, treating
        // them as variables;  this is because we have
        //    (InterpreationNode "MicroplanningNewSentence")
        // from the microplanner that should be matched to any InterpretationNode
        if (n->get_type() == INTERPRETATION_NODE || n->get_type() == VARIABLE_NODE)
        {
            sVars.insert(n);
            continue;
        }

        // special treatment for DefinedLinguisticConceptNode and
        // DefinedLinguisticPredicateNode, do not treat them as variables
        // because they are not actual words of a sentence.
        if (n->get_type() == DEFINED_LINGUISTIC_CONCEPT_NODE or
            n->get_type() == DEFINED_LINGUISTIC_PREDICATE_NODE)
           continue;

        std::string sName = n->get_name();

        // if it is an instance, check if it has the LG relationships
        if (sName.find("@") != std::string::npos)
        {
            Handle hWordInstNode = pAS->get_handle(WORD_INSTANCE_NODE, sName);

            // no corresponding WordInstanceNode found
            if (hWordInstNode == nullptr)
                continue;

            // if no LG link generated for the instance
            if (get_target_neighbors(hWordInstNode, LG_WORD_CSET).empty())
                continue;
        } 
        // n is a concept or predicate node
        Handle hWordNode;
        HandleSeq neighbor_win = get_target_neighbors(n, REFERENCE_LINK);
        if (neighbor_win.size() != 0)
        {
            HandleSeq neighbor_wn = get_target_neighbors(neighbor_win[0], REFERENCE_LINK);
            hWordNode = neighbor_wn[0];
        }
        else
        {
            std::string sWord = sName.substr(0, sName.find_first_of('@'));
            hWordNode = pAS->get_handle(WORD_NODE, sWord);
        }
        // no WordNode found
        if (hWordNode == nullptr)
            continue;

        sVars.insert(n);
    }

    SuRealPMCB pmcb(pAS, sVars, use_cache);
    PatternLinkPtr slp(createPatternLink(sVars, qClauses));
    pmcb.satisfy(slp);

    // The cached version of SuReal is supposed to return only true or false,
    // not the set of all acceptable answers. To keep ortogonality with the
    // interface of the standard (non-cached) version, this shortcut is
    // returning an empty HandleSeq meaning 'false' or a HandleSeq with a single
    // (actually meaningless) Handle meaning 'true'.
    //
    // Ideally there should be two different methods with different interfaces
    // for the cached and the non-cached versions.
    if (use_cache) {
        if (pmcb.m_results.empty()) {
            return HandleSeqSeq();
        } else {
            HandleSeqSeq results;
            HandleSeq item;
            Handle h;
            item.push_back(h);
            results.push_back(item);
            return results;
        }
    }

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
        HandleSeq qi = get_target_neighbors(hi, REFERENCE_LINK);
        HandleSeq qj = get_target_neighbors(hj, REFERENCE_LINK);
        qi.erase(std::remove_if(qi.begin(), qi.end(), [](Handle& h) { return h->get_type() != SET_LINK; }), qi.end());
        qj.erase(std::remove_if(qj.begin(), qj.end(), [](Handle& h) { return h->get_type() != SET_LINK; }), qj.end());

        // assuming each InterpretationNode is only linked to one SetLink
        // and compare using arity
        return qi[0]->get_arity() < qj[0]->get_arity();
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
HandleSeqSeq SuRealSCM::sureal_get_mapping(Handle& h, std::vector<HandleMap >& mappings)
{
    logger().debug("[SuReal] %d mapping(s) for %s", mappings.size(), h->to_short_string().c_str());

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
        HandleMap& mapping = mappings[i];

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
