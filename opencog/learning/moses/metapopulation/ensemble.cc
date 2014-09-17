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
#include  <opencog/comboreduct/reduct/reduct.h>
#include  <opencog/learning/moses/moses/complexity.h>
#include "ensemble.h"

namespace opencog {
namespace moses {

using namespace combo;

ensemble::ensemble(behave_cscore& cs, const ensemble_parameters& ep) :
	_params(ep), _bscorer(cs.get_bscorer())
{
	// Don't mess with the scorer weights if not doing boosting.
	if (not ep.do_boosting) return;

	// Initialize the weights that the scorer will use.
	_bscorer.reset_weights();

	// _tolerance is an estimate of the accumulated rounding error
	// that arises when totaling the bscores.  As usual, assumes a
	// normal distribution for this, so that its a square-root.
	size_t bslen = _bscorer.size();
	_tolerance = 2.0 * epsilon_score;
	_tolerance *= sqrt((double) bslen);

	_bias = 0.0;
	if (not ep.exact_experts) {
		_row_bias = std::vector<double>(bslen, 0.0);
	}

	logger().info() << "Boosting: number to promote: " << ep.num_to_promote;
	if (ep.experts) {
		logger().info() << "Boosting: exact experts: " << ep.exact_experts;
		logger().info() << "Boosting: expalpha: " << ep.expalpha;
	}
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
	if (_params.experts) {
		add_expert(cands);
		return;
	}
	add_adaboost(cands);
}

/**
 * Implement a boosted ensemble, using the classic AdaBoost algo.
 */
void ensemble::add_adaboost(scored_combo_tree_set& cands)
{
	int promoted = 0;

	while (true) {
		// Find the element (the combo tree) with the least error. This is
		// the element with the highest score.
		scored_combo_tree_set::iterator best_p =
			std::min_element(cands.begin(), cands.end(),
				[](const scored_combo_tree& a, const scored_combo_tree& b) {
					return a.get_score() > b.get_score(); });

		logger().info() << "Boosting: candidate score=" << best_p->get_score();
		double err = _bscorer.get_error(best_p->get_bscore());
		OC_ASSERT(0.0 <= err and err < 1.0, "boosting score out of range; got %g", err);

		// This condition indicates "perfect score". It does not typically
		// happen, but if it does, then we have no need for all the boosting
		// done up to this point.  Thus, we erase the entire ensemble; its
		// now superfluous, and replace it by the single best answer.
		if (err < _tolerance) {
			logger().info() << "Boosting: perfect score found: " << &best_p;

			// Wipe out the ensemble.
			_scored_trees.clear();
			scored_combo_tree best(*best_p);
			best.set_weight(1.0);
			_scored_trees.insert(best);

			// Clear out the scorer weights, to avoid down-stream confusion.
			_bscorer.reset_weights();

			return;
		}

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

		// Set the weight for the tree, and stick it in the ensemble
		scored_combo_tree best = *best_p;
		best.set_weight(alpha);
		_scored_trees.insert(best);

		// Recompute the weights
		const behavioral_score& bs = best_p->get_bscore();
		size_t bslen = _bscorer.size();
		std::vector<double> weights(bslen);
		for (size_t i=0; i<bslen; i++)
		{
			weights[i] = is_correct(bs[i]) ? rcpalpha : expalpha;
		}
      _bscorer.update_weights(weights);

		// Remove from the set of candidates.
		cands.erase(best_p);

		// Are we done yet?
		promoted ++;
		if (_params.num_to_promote <= promoted) break;
		if (0 == cands.size()) break;
	}
}

/**
 * Add trees that expertly select rows.  These are trees that select
 * only a handful of rows, usually just the true-postive rows, unless
 * exact_experts is set, in which case only the trees that are exactly
 * correct are admitted to the ensemble.
 */
void ensemble::add_expert(scored_combo_tree_set& cands)
{
	int promoted = 0;
	while (true) {
		// Find the element (the combo tree) with the least error.
		scored_combo_tree_set::iterator best_p;
		if (_params.exact_experts) {
			// Find the element (the combo tree) with the least error. This is
			// the element with the highest score, that does not also accept
			// undesirable members. (i.e. has an imperfect classifcation)
			best_p =
				std::min_element(cands.begin(), cands.end(),
					[&](const scored_combo_tree& a, const scored_combo_tree& b) {
						bool abad = (_tolerance < _bscorer.get_error(a.get_bscore()));
						bool bbad = (_tolerance < _bscorer.get_error(b.get_bscore()));
						if (abad and bbad) return a.get_score() > b.get_score();
			 	 		if (abad) return false;
						if (bbad) return true;
						return a.get_score() > b.get_score(); });
		} else {
			// Find the element (the combo tree) with the least error. This is
			// the element with the highest score. Imprecise experts are allowed.
			best_p =
				std::min_element(cands.begin(), cands.end(),
					[&](const scored_combo_tree& a, const scored_combo_tree& b) {
						return a.get_score() > b.get_score(); });
		}

		double err = _bscorer.get_error(best_p->get_bscore());
		logger().info() << "Expert: candidate score=" << best_p->get_score()
                      << " error=" << err;

		OC_ASSERT(0.0 <= err and err < 1.0, "boosting score out of range; got %g", err);

		// Set the weight for the tree, and stick it in the ensemble
		scored_combo_tree best = *best_p;
		double alpha;
		double expalpha;
		if (_params.exact_experts) {
			// This condition indicates "perfect score". This is the only type
			// of tree that we accept into the ensemble.  Its unlikely that the
			// next-highest scoring one will be any good, so just break.
			if (_tolerance <= err) {
				logger().info() << "Exact expert: Best tree not good enough: " << err;
				break;
			}

			alpha = 1.0;    // Ad-hoc but uniform weight.
			expalpha = _params.expalpha; // Add-hoc boosting value ...
			logger().info() << "Exact expert: add to ensemble " << best;
		} else {
			// Any score worse than half is terrible. Half gives a weight of zero.
			// More than half gives negative weights, which wreaks things.
			if (0.5 <= err) {
				logger().info() << "Expert: terrible precision, ensemble not expanded: " << err;
				break;
			}
			// AdaBoost-style alpha; except we allow perfect scorers.
			if (err < _tolerance) err = _tolerance;
			alpha = 0.5 * log ((1.0 - err) / err);
			expalpha = exp(alpha);
			logger().info() << "Expert: add to ensemble; err=" << err
			                << " alpha=" << alpha <<" exp(alpha)=" << expalpha
			                << std::endl << best;
		}

		best.set_weight(alpha);
		_scored_trees.insert(best);

		// Adjust the bias, if needed.
		if (not _params.exact_experts) {
			const behavioral_score& bs = best_p->get_bscore();
			size_t bslen = _bscorer.size();

			// Now, look to see where this scorer was wrong, and bump the
			// bias for that.  Here, we make the defacto assumption that
			// the scorer is the "pre" scorer, and so the bs ranges from
			// -0.5 to +0.5, with zero denoting "row not selected by tree",
			// a positive score denoting "row correctly selected by tree",
			// and a negative score denoting "row wrongly selected by tree".
			// The scores differ from +/-0.5 only if the rows are degenerate.
			// Thus, the ensemble will incorrectly pick a row if it picks
			// the row, and the weight isn't at least _bias.  (Keep in mind
			// that alpha is the weight of the current tree.)
			for (size_t i=0; i<bslen; i++) {
				if (bs[i] >= 0.0) continue;
				// -2.0 to cancel the -0.5 for bad row.
				_row_bias[i] += -2.0 * alpha * bs[i];
				if (_bias < _row_bias[i]) _bias = _row_bias[i];
			}
			logger().info() << "Experts: bias is now: " << _bias;
		}

		// Increase the importance of all remaining, unselected rows.
		double rcpalpha = 1.0 / expalpha;
		const behavioral_score& bs = best_p->get_bscore();
		size_t bslen = _bscorer.size();
		std::vector<double> weights(bslen);
		for (size_t i=0; i<bslen; i++)
		{
			// Again, here we explicitly assume the pre scorer: A row is
			// correctly selected if its score is strictly positive.
			// The weights of unselected rows must increase.
			weights[i] = (0.0 < bs[i]) ? rcpalpha : expalpha;
		}
      _bscorer.update_weights(weights);

		// Remove from the set of candidates.
		cands.erase(best_p);

		// Are we done yet?
		promoted ++;
		if (_params.num_to_promote <= promoted) break;
		if (0 == cands.size()) break;
	}
}

/// Return the ensemble contents as a single, large tree.
///
const combo::combo_tree& ensemble::get_weighted_tree() const
{
	if (_params.experts) {
		if (_params.exact_experts)
			get_exact_tree();
		else
			get_expert_tree();
	} else {
		get_adaboost_tree();
	}
	return _weighted_tree;
}

/// Return the ensemble contents as a single, large weighted tree.
///
/// Returns the combo tree expressing
///    (sum_i weight_i * (tree_i ? 1.0 : -1.0)) > 0)
/// i.e. true if the summation is positive, else false, as per standard
/// AdaBoost definition.
///
const combo::combo_tree& ensemble::get_adaboost_tree() const
{
	_weighted_tree.clear();

	if (1 == _scored_trees.size()) {
		_weighted_tree = _scored_trees.begin()->get_tree();
		return _weighted_tree;
	}

	combo::combo_tree::pre_order_iterator head, plus;

	head = _weighted_tree.set_head(combo::id::greater_than_zero);
	plus = _weighted_tree.append_child(head, combo::id::plus);

	for (const scored_combo_tree& sct : _scored_trees)
	{
		combo::combo_tree::pre_order_iterator times, minus, impulse;

		// times is (weight * (tree - 0.5))
		times = _weighted_tree.append_child(plus, combo::id::times);
		vertex weight = sct.get_weight();
		_weighted_tree.append_child(times, weight);

		// minus is (tree - 0.5) so that minus is equal to +0.5 if
		// tree is true, else it is equal to -0.5
		minus = _weighted_tree.append_child(times, combo::id::plus);
		vertex half = -0.5;
		_weighted_tree.append_child(minus, half);
		impulse = _weighted_tree.append_child(minus, combo::id::impulse);
		_weighted_tree.append_child(impulse, sct.get_tree().begin());
	}

	return _weighted_tree;
}

/// Return the ensemble contents as a single, large tree.
///
/// Returns the combo tree expressing
///   or_i tree_i
/// i.e. true if any member of the ensemble picked true.
///
const combo::combo_tree& ensemble::get_exact_tree() const
{
	_weighted_tree.clear();
	if (1 == _scored_trees.size()) {
		_weighted_tree = _scored_trees.begin()->get_tree();
		return _weighted_tree;
	}

	combo::combo_tree::pre_order_iterator head;
	head = _weighted_tree.set_head(combo::id::logical_or);

	for (const scored_combo_tree& sct : _scored_trees)
		_weighted_tree.append_child(head, sct.get_tree().begin());

	// Very surprisingly, reduct does not seem to make a differrence! Hmmm.
	// std::cout << "before reduct " << tree_complexity(_weighted_tree) << std::endl;
	// reduct::logical_reduce(1, _weighted_tree);
	// std::cout << "ater reduct " << tree_complexity(_weighted_tree) << std::endl;

	return _weighted_tree;
}

/// Return the ensemble contents as a single, large tree.
///
/// Returns the combo tree expressing
///    (sum_i weight_i * (tree_i ? 1.0 : 0.0)) > _bias)
/// i.e. true if the summation is greater than the bias, else false.
/// The goal of the bias is to neuter those members of the ensemble
/// that are making mistakes in their selection.
///
const combo::combo_tree& ensemble::get_expert_tree() const
{
	_weighted_tree.clear();
	if (1 == _scored_trees.size()) {
		_weighted_tree = _scored_trees.begin()->get_tree();
		return _weighted_tree;
	}

	combo::combo_tree::pre_order_iterator head, plus;

	head = _weighted_tree.set_head(combo::id::greater_than_zero);
	plus = _weighted_tree.append_child(head, combo::id::plus);

	// score must be bettter than the bias.
	vertex bias = -_bias * _params.bias_scale;
	_weighted_tree.append_child(plus, bias);

	for (const scored_combo_tree& sct : _scored_trees)
	{
		combo::combo_tree::pre_order_iterator times, impulse;

		// times is (weight * tree)
		times = _weighted_tree.append_child(plus, combo::id::times);
		vertex weight = sct.get_weight();
		_weighted_tree.append_child(times, weight);

		// impulse is +1.0 if tree is true, else it is equal to 0.0
		impulse = _weighted_tree.append_child(times, combo::id::impulse);
		_weighted_tree.append_child(impulse, sct.get_tree().begin());
	}
	return _weighted_tree;
}

/**
 * Return the plain, unweighted, "flat" score for the ensemble as a
 * whole.  This is the score that the ensemble would get when used
 * for prediction; by contrast, the weighted score only applies for
 * training, and is always driven to be 50% wrong, on average.
 */
score_t ensemble::flat_score() const
{
	behavioral_score bs(_bscorer(_scored_trees));
	return boost::accumulate(bs, 0.0);
}


}}; // namespace opencog::moses

