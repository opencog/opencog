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

ensemble::ensemble(behave_cscore& cs, const ensemble_parameters& ep) :
	_params(ep)
{
	_booster = dynamic_cast<boosting_ascore*>(&(cs.get_ascorer()));
	OC_ASSERT(_booster, "Ensemble can only be used with a weighted scorer");
}

// Is this behavioral score correct? For boolean scores, correct is 0.0
// and incorrect is -1.0.
static inline bool is_correct(score_t val)
{
	return -0.5 < val;
}

/**
 * Implement a boosted ensemble. Candidate combo trees are added to
 * the ensemble one at a time, weights are adjusted, and etc.
 */
void ensemble::add_candidates(scored_combo_tree_set& cands)
{
	int promoted = 0;
	// We need the length of the behavioral score, as normalization
	// XXX we should be using the user-weighted thingy here .. XX FIXME

	double behave_len = cands.begin()->get_bscore().size();
	while (true) {
		// Find the element with the least error
		scored_combo_tree_set::iterator best_p = 
			std::min_element(cands.begin(), cands.end(),
				[](const scored_combo_tree& a, const scored_combo_tree& b) {
					return a.get_score() > b.get_score(); });

		double err = - best_p->get_score() / behave_len;
		OC_ASSERT(0.0 <= err and err < 1.0, "boosting score out of range; got %g", err);

		// XXX FIXME, this should be something else ... 
		if (0.0 == err) break;

		// any score worse than half is terrible. half gives a weight of zero.
		if (0.5 <= err) break;

		// Compute alpha
		double alpha = 0.5 * log ((1.0 - err) / err);
		double expalpha = exp(alpha);
		double rcpalpha = 1.0 / expalpha;
std::cout << "===================================== "<<std::endl;
std::cout << "duuude best " << *best_p  << " err=" << err << " alpha=" << alpha <<" expalph=" << expalpha << std::endl;

		// Recompute the weights
		const behavioral_score& bs = best_p->get_bscore();
		std::vector<double>& weights = _booster->get_weights();
		double znorm = 0.0;
		for (int i=0; i<behave_len; i++)
		{
			weights[i] *= is_correct(bs[i]) ? rcpalpha : expalpha;
			znorm += weights[i];
		}

		// Normalization: sum of scores must equal vector length.
		znorm = behave_len / znorm;
		for (int i=0; i<behave_len; i++)
		{
			weights[i] *= znorm;
		}

		// Set the weight for the tree, and stick it in the ensemble
		scored_combo_tree best = *best_p;
		best.set_weight(alpha);
		_scored_trees.insert(best);

		// Remove from the set of candidates.
		cands.erase(best_p);

		// Obtain cumulative score. XXX This can't stay here.
		double cum = 0.0;
		for (const scored_combo_tree& sct : _scored_trees)
		{
			double tree_weight = sct.get_weight();
			double raw_score = sct.get_score();
			raw_score *= 2.0;
			raw_score += behave_len;
std::cout << "duuude raw= " << raw_score << " wieght=" << tree_weight <<" tre="<< sct<<std::endl;
			cum += tree_weight * raw_score;
		}
std::cout << "duuude cum score is " << cum << std::endl;

		// Are we done yet?
		promoted ++;
		if (_params.num_to_promote < promoted) break;
		if (0 == cands.size()) break;
	}
}


}}; // namespace opencog::moses

