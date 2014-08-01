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
	_best_possible_score = cs.best_possible_score();
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
	OC_ASSERT(_booster, "Ensemble can only be used with a weighted scorer");

	int promoted = 0;
	// We need the length of the behavioral score, as normalization
	// XXX we should be using the user-weighted thingy here .. XXX FIXME

	double behave_len = cands.begin()->get_bscore().size();
	while (true) {
		// Find the element with the least error
		scored_combo_tree_set::iterator best_p = 
			std::min_element(cands.begin(), cands.end(),
				[](const scored_combo_tree& a, const scored_combo_tree& b) {
					return a.get_score() > b.get_score(); });

		double err = (_best_possible_score - best_p->get_score()) / behave_len;
		OC_ASSERT(0.0 <= err and err < 1.0, "boosting score out of range; got %g", err);

		// XXX FIXME, this should be something else ... 
		if (0.0 == err) break;

		// Any score worse than half is terrible. half gives a weight of zero.
		if (0.5 <= err) {
			logger().info() << "Boosting: no improvement, ensemble not expanded";
			break;
		}

		// Compute alpha
		double alpha = 0.5 * log ((1.0 - err) / err);
		double expalpha = exp(alpha);
		double rcpalpha = 1.0 / expalpha;
		logger().info() << "Boosting: add to ensemble " << *best_p  << std::endl
			<< "With err=" << err << " alpha=" << alpha <<" exp(alpha)=" << expalpha;

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

		// Are we done yet?
		promoted ++;
		if (_params.num_to_promote <= promoted) break;
		if (0 == cands.size()) break;
	}
}

/// Return the ensemble contents as a single, large weighted tree.
/// 
/// Returns the combo tree expressing 
/// (sum_i weight_i * (tree_i ? 1.0 : -1.0)) > 0)
/// i.e. true if the summation is positive, else false, as per standard
/// AdaBoost definition.
///
combo::combo_tree ensemble::get_weighted_tree() const
{
	combo::combo_tree tr;

	combo::combo_tree::pre_order_iterator head, plus, times, minus, impulse;

	head = tr.set_head(combo::id::greater_than_zero);
	plus = tr.append_child(head, combo::id::plus);

	for (const scored_combo_tree& sct : _scored_trees)
	{
		// times is (weight * (tree - 0.5))
		times = tr.append_child(plus, combo::id::times);
		vertex weight = sct.get_weight();
		tr.append_child(times, weight);

		// minus is (tree - 0.5) so that minus is equal to +0.5 if
		// tree is true, else it is equal to -0.5
		minus = tr.append_child(times, combo::id::plus);
		vertex half = -0.5;
		tr.append_child(minus, half);
		impulse = tr.append_child(minus, combo::id::impulse);
		tr.append_child(impulse, sct.get_tree().begin());
	}

	return tr;
}


}}; // namespace opencog::moses

