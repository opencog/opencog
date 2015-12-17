/*
 * Fuzzy.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * Author: Leung Man Hin <https://github.com/leungmanhin>
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
#include <opencog/atomutils/AtomUtils.h>
#include <opencog/atomutils/FindUtils.h>

#include "Fuzzy.h"

using namespace opencog::nlp;
using namespace opencog;

Fuzzy::Fuzzy(AtomSpace* as, Type t, const HandleSeq& l) :
    FuzzyPatternMatch(as),
    rtn_type(t),
    excl_list(l)
{
}

Fuzzy::~Fuzzy()
{
}

void Fuzzy::set_pattern(const Variables& vars, const Pattern& pat)
{
    FuzzyPatternMatch::set_pattern(vars, pat);

    pattern = pat.mandatory[0];
    pat_nodes = get_all_nodes(pattern);
    std::sort(pat_nodes.begin(), pat_nodes.end());
}

bool Fuzzy::accept_starter(const NodePtr np)
{
    return FuzzyPatternMatch::accept_starter(np) and
           (np->getType() == CONCEPT_NODE or np->getType() == PREDICATE_NODE);
}

void Fuzzy::similarity_match(const Handle& pat, const Handle& soln)
{
    if (soln->getType() != rtn_type)
        return;

    HandleSeq soln_nodes = get_all_nodes(soln);

    auto checker = [&] (Handle& h) {
        return std::find(excl_list.begin(), excl_list.end(), h) != excl_list.end();
    };

    if (std::any_of(soln_nodes.begin(), soln_nodes.end(), checker))
        return;

    // Find out how many nodes it has in common with the pattern
    HandleSeq common_nodes;
    std::sort(soln_nodes.begin(), soln_nodes.end());
    std::set_intersection(pat_nodes.begin(), pat_nodes.end(),
                          soln_nodes.begin(), soln_nodes.end(),
                          std::back_inserter(common_nodes));

    // Remove duplicates
    std::sort(common_nodes.begin(), common_nodes.end());
    common_nodes.erase(std::unique(common_nodes.begin(), common_nodes.end()), common_nodes.end());

    double similarity = 0.0;

    for (const Handle& common_node : common_nodes) {
        // If both the pattern and the potential solution share some "instance"
        // node, that probably means the potential solution is a sub-pattern
        // of the input pattern, or is other related atoms that is generated
        // with the pattern, which is pretty much not what we want here
        if (NodeCast(common_node)->getName().find("@") != std::string::npos)
            return;

        double weight = 0;

        std::string cn_name = NodeCast(common_node)->getName();
        Type type = common_node->getType();

        auto get_instance = [&] (Handle n) {
            std::string name = NodeCast(n)->getName();

            if (name.length() < cn_name.length()+1)
                return false;

            // e.g. Getting (ConceptNode "cat@123") from (ConceptNode "cat")
            if (n->getType() == type and name.compare(0, cn_name.length()+1, cn_name+'@') == 0)
                return true;

            // e.g. Getting (PredicateNode "is@123") from (PredicateNode "be")
            if (type == PREDICATE_NODE) {
                for (const Handle& ll : LinkCast(pattern)->getOutgoingSet()) {
                    if (ll->getType() == IMPLICATION_LINK and
                        is_atom_in_tree(ll, common_node) and is_atom_in_tree(ll, n))
                        return true;
                }
            }

            return false;
        };

        auto inst = std::find_if(pat_nodes.begin(), pat_nodes.end(), get_instance);

        if (inst != pat_nodes.end()) {
            // Get the WordInstanceNode from ReferenceLink
            HandleSeq word_inst = get_neighbors(*inst, false, true, REFERENCE_LINK, false);

            if (word_inst.size() > 0) {
                HandleSeq eval_links = get_predicates(word_inst[0], DEFINED_LINGUISTIC_RELATIONSHIP_NODE);

                for (const Handle& el : eval_links) {
                    std::string ling_rel = NodeCast(LinkCast(el)->getOutgoingSet()[0])->getName();

                    if (ling_rel.compare("_subj") == 0)
                        weight += 2.0;
                    else if (ling_rel.compare("_obj") == 0)
                        weight += 2.0;
                    else if (ling_rel.compare("_adj") == 0)
                        weight += 2.0;
                    else
                        weight += 1.0;
                }
            }
        }

        // similarity += (weight / common_node->getIncomingSetSize());
        similarity += weight;
    }

    // The size different between the pattern and the potential solution
    size_t diff = std::abs((int)(pat_nodes.size() - soln_nodes.size()));

    solns[soln] = std::make_pair(similarity, diff);
}


HandleSeq Fuzzy::get_solns()
{
    HandleSeq rtn_solns;

    for (auto s = solns.begin(); s != solns.end(); s++) {
        rtn_solns.push_back(s->first);
    }

    std::sort(rtn_solns.begin(), rtn_solns.end(),
              [&](Handle h1, Handle h2) {
                  if (solns[h1].first == solns[h2].first)
                      return solns[h1].second < solns[h2].second;
                  else
                      return solns[h1].first > solns[h2].first;
              });

    return rtn_solns;
}
