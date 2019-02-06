/*
 * FuzzyMatchBasic.cc
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

#include <opencog/atoms/base/Node.h>
#include <opencog/atoms/core/FindUtils.h>

#include "FuzzyMatchBasic.h"

using namespace opencog;

/**
 * Get all the atoms within a link and its sublinks.
 *
 * @param h      The top level link
 * @param nlist  A list storeing the nodes
 * @param alist  A list storing the atoms
 */
static void get_all_atoms(const Handle& h, HandleSeq& nlist, HandleSeq& alist)
{
	alist.emplace_back(h);

	if (h->is_node())
	{
		nlist.emplace_back(h);
		return;
	}

	if (h->is_link())
	{
		for (const Handle& o : h->getOutgoingSet())
			get_all_atoms(o, nlist, alist);
	}
}

/**
 * Set up the target.
 *
 * @param trg  The target
 */
void FuzzyMatchBasic::start_search(const Handle& trg)
{
	target = trg;
	get_all_atoms(target, target_nodes, target_atoms);
	std::sort(target_nodes.begin(), target_nodes.end());
}

/**
 * hp is a subtree of the target tree. Should we start a search
 * at that location?  Answer: yes if its a node, no, if its a link.
 *
 * @param hp  A subtree of the target tree
 * @return    True if it is a node, false otherwise
 */
bool FuzzyMatchBasic::accept_starter(const Handle& hp)
{
	if (hp->is_link()) return false;
	return true;
}

/**
 * Estimate how similar the proposed matching tree is to the target.
 *
 * @param soln  The proposed match.
 */
bool FuzzyMatchBasic::try_match(const Handle& soln)
{
	if (is_atom_in_tree(target, soln)) return false;

	// Find out how many atoms it has in common with the pattern
	HandleSeq soln_nodes;
	HandleSeq soln_atoms;
	get_all_atoms(soln, soln_nodes, soln_atoms);
	std::sort(soln_nodes.begin(), soln_nodes.end());

	HandleSeq common_nodes;
	std::set_intersection(target_nodes.begin(), target_nodes.end(),
	                      soln_nodes.begin(), soln_nodes.end(),
	                      std::back_inserter(common_nodes));

	// The size different between the pattern and the potential solution
	size_t diff = std::abs((int)target_atoms.size() - (int)soln_atoms.size());

	double similarity = common_nodes.size();

	// Roughly estimate how "rare" each node is by using 1 / incoming set size
	// TODO: May use Truth Value instead
	// for (const Handle& common_node : common_nodes)
	// 	similarity += 1.0 / common_node->getIncomingSetSize();

	LAZY_LOG_FINE << "\n========================================\n"
	              << "Comparing:\n" << target->to_short_string()
	              << "----- and:\n" << soln->to_short_string() << "\n"
	              << "Common nodes = " << common_nodes.size() << "\n"
	              << "Size diff = " << diff << "\n"
	              << "Similarity = " << similarity << "\n"
	              << "Most similar = " << max_similarity << "\n"
	              << "========================================\n";

	// Decide if we should accept the potential solutions or not
	if ((similarity > max_similarity) or
		(similarity == max_similarity and diff < min_size_diff))
	{
		max_similarity = similarity;
		min_size_diff = diff;
		solns.clear();
		solns.push_back({soln, similarity});
	}

	else if (similarity == max_similarity and diff == min_size_diff) {
		solns.push_back({soln, similarity});
	}

	return true;
}

/* No-op; we already build "solns", just return it. */
RankedHandleSeq FuzzyMatchBasic::finished_search(void)
{
	return solns;
}
