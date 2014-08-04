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
	_params(ep), _bcscorer(cs)
{
	_booster = dynamic_cast<boosting_ascore*>(&(cs.get_ascorer()));
	_effective_length = -cs.worst_possible_score();
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

	// We need the length of the behavioral score, as normalization.
	// The correct "length" is kind-of tricky to understand when a table
	// has weighted rows, or when it is degenerate, so that no matter
	// what selection is made, some rows will be wrong.  So we review
	// the cases here: the table may have degenerate or non-degenerate
	// rows, and these may be weighted or non-weighted.  Here, the
	// "weights" are not the boosting weights, but the user-specified row
	// weights.
	//
	//  non-degenerate, non weighted:
	//       (each row has defacto weight of 1.0)
	//       best score = 0 so  err = score / num rows;
	//
	//  non-degenerate, weighted:
	//       best score = 0 so  err = score / weighted num rows;
	//       since the score is a sum of weighted rows.
	//
	//       e.g. two rows with user-specified weights:
	//            0.1
	//            2.3
	//       so if first row is wrong, then err = 0.1/2.4
	//       and if second row is wrong, err = 2.3/2.4
	//
	//  degenerate, non-weighted:
	//       best score > 0   err = (score - best_score) / eff_num_rows;
	//
	//       where eff_num_rows = sum_row fabs(up-count - down-count)
	//       is the "effective" number of rows, as opposing rows
	//       effectively cancel each-other out.  This is also the
	//       "worst possible score", what would be returned if every
	//       row was marked wrong.
	//
	//       e.g. table five uncompressed rows:
	//            up:1  input-a
	//            dn:2  input-a
	//            up:2  input-b
	//       best score is -1 (i.e. is 4-5 where 4 = 2+2).
	//       so if first row is wrong, then err = (1-1)/5 = 0/3
	//       so if second row is wrong, then err = (2-1)/5 = 1/3
	//       so if third & first is wrong, then err = (3-1)/3 = 2/3
	//       so if third & second is wrong, then err = (4-1)/3 = 3/3
	//
	// Thus, the "effective_length" is (minus) the worst possible score.
	//
	// Also: Note: the bet_score needs to be continually re-computed
	// using the current (boosted) row weights.
	while (true) {
		// Find the element (the combo tree) with the least error. This is
		// the element with the highest score.
		scored_combo_tree_set::iterator best_p =
			std::min_element(cands.begin(), cands.end(),
				[](const scored_combo_tree& a, const scored_combo_tree& b) {
					return a.get_score() > b.get_score(); });

      double best_score = _bcscorer.weighted_best_score();
		logger().debug() << "Best score=" << best_score
		                 << " Actual score=" << best_p->get_score()
		                 << " Effective legnth:" << _effective_length;
		double err = (best_score - best_p->get_score()) / _effective_length;
		OC_ASSERT(0.0 <= err and err < 1.0, "boosting score out of range; got %g", err);

		// XXX FIXME, this should be something else ...
		if (0.0 == err) break;

		// Any score worse than half is terrible. Half gives a weight of zero.
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
		size_t bslen = bs.size();
		std::vector<double>& weights = _booster->get_weights();
		double znorm = 0.0;
		for (size_t i=0; i<bslen; i++)
		{
			weights[i] *= is_correct(bs[i]) ? rcpalpha : expalpha;
			znorm += weights[i];
		}

		// Normalization: sum of scores must equal vector length.
		znorm = _effective_length / znorm;
		for (size_t i=0; i<bslen; i++)
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

/**
 * Return the plain, unweighted, "flat" score for the ensemble as a
 * whole.  This is the score that the ensemble would get when used
 * for prediction; by contrast, the weighted score only applies for
 * training, and is always driven to be 50% wrong, on average.
 */
score_t ensemble::flat_score() const
{
	behavioral_score bs = _bcscorer.get_bscore(_scored_trees);
	return _flat_scorer(bs);
}


}}; // namespace opencog::moses

