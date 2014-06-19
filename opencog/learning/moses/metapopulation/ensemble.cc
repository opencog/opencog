/**
 * ensemble.cc ---
 *
 * Copyright (C) 2014 Aidyia Limited
 *
 * Authors: Linas Vepstas
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

#include <math.h>

#include <algorithm>
#include <iostream>

#include  <opencog/util/oc_assert.h>
#include "ensemble.h"

namespace opencog {
namespace moses {

using namespace combo;

ensemble::ensemble(const ensemble_parameters& ep) :
	_params(ep)
{}

/**
 * Implement a boosted ensemble. Candidate combo trees are added to
 * the ensemble one at a time, weights are adjusted, and etc.
 */
void ensemble::add_candidates(scored_combo_tree_set& cands)
{
	int promoted = 0;
	// We need the length of the behavioral score, as normalization
	double behave_len = cands.begin()->get_bscore().size();
	while (true) {
		// Find the element with the least error
		scored_combo_tree_set::iterator best_p = 
			std::min_element(cands.begin(), cands.end(),
				[](const scored_combo_tree& a, const scored_combo_tree& b) {
					return a.get_score() > b.get_score(); });

		// Compute alpha
		double err = - best_p->get_score() / behave_len;
		OC_ASSERT(0.0 < err and err < 1.0, "boosting score out of range");
		double alpha = 0.5 * log ((1.0 - err) / err);

std::cout << "===================================== "<<std::endl;
std::cout << "duuude best " << *best_p  << " err=" << err << " apja=" << alpha << std::endl;
		// Set the wieght for the tree, and stick it in the ensemble
		scored_combo_tree best = *best_p;
		best.set_weight(alpha);
		_scored_trees.insert(best);

		// Remove from the set of candidates.
		cands.erase(best_p);

std::cout << "===================================== ensemble"<<std::endl;
		// Score the ensemble. XXX This should probably go into some class.
		for (const scored_combo_tree& sct : _scored_trees)
		{
std::cout << "duuude ensemb is " << sct << std::endl;
		}

		// Now, re-score the candidates! Ugh.
std::cout << "===================================== cands"<<std::endl;
		for (const scored_combo_tree& sct : cands)
		{
std::cout << "duuude cand is " << sct << std::endl;
		}

		// Are we done yet?
		promoted ++;
		if (_params.num_to_promote < promoted) break;
		if (0 == cands.size()) break;
	}
}


}}; // namespace opencog::moses

