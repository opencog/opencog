/*
 * opencog/learning/moses/scoring/scoring_base.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012,2013 Poulin Holdings LLC
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Moshe Looks, Nil Geisweiller, Linas Vepstas
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

#include "scoring_base.h"

namespace opencog { namespace moses {

// Note that this function returns a POSITIVE number, since p < 0.5
score_t discrete_complexity_coef(unsigned alphabet_size, double p)
{
    return -log((double)alphabet_size) / log(p/(1-p));
}

        // Note that this returns a POSITIVE number.
score_t contin_complexity_coef(unsigned alphabet_size, double stdev)
{
    return log(alphabet_size) * 2 * sq(stdev);
}

void bscore_base::set_complexity_coef(unsigned alphabet_size, float p)
{
    // Both p==0.0 and p==0.5 are singularities in the forumla.
    // See the explanation in the comment above ctruth_table_bscore.
    _complexity_coef = 0.0;
    if (p > 0.0f && p < 0.5f)
        _complexity_coef = discrete_complexity_coef(alphabet_size, p);

    logger().info() << "BScore noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/_complexity_coef;
}

void bscore_base::set_complexity_coef(score_t complexity_ratio)
{
    _complexity_coef = 0.0;
    if (complexity_ratio > 0.0)
        _complexity_coef = 1.0 / complexity_ratio;

    logger().info() << "BScore complexity ratio = " << 1.0/_complexity_coef;
}

////////////////////
// behave_cscore //
///////////////////

composite_score behave_cscore::operator()(const combo_tree& tr) const
{
    score_t res = 0.0;
    try {
        res = boost::accumulate(_bscorer(tr), 0.0);
    }
    catch (EvalException& ee)
    {
        // Exceptions are raised when operands are out of their
        // valid domain (negative input log or division by zero),
        // or outputs a value which is not representable (too
        // large exp or log). The error is logged as level fine
        // because this happens very often when learning continuous
        // functions, and it clogs up the log when logged at a
        // higher level.
        logger().fine()
           << "The following candidate: " << tr << "\n"
           << "has failed to be evaluated, "
           << "raising the following exception: "
           << ee.get_message() << " " << ee.get_vertex();

        return worst_composite_score;
    }

    complexity_t cpxy = _bscorer.get_complexity(tr);
    score_t cpxy_coef = _bscorer.get_complexity_coef();
    if (logger().isFineEnabled()) {
        logger().fine() << "behave_cscore: " << res
                        << " complexity: " << cpxy
                        << " cpxy_coeff: " << cpxy_coef;
    }

    return composite_score(res, cpxy, cpxy * cpxy_coef, 0.0);
}


////////////////////////
// multibehave_cscore //
////////////////////////

// main operator
composite_score multibehave_cscore::operator()(const combo_tree& tr) const
{
    score_t sum_score = 0.0;
    score_t sum_coef = 0.0;
    score_t sum_penalty = 0.0;

    for (const bscore_base& bsc : _bscorers) {
        score_t res = 0.0;
        try {
            res = boost::accumulate(bsc(tr), 0.0);
        }
        catch (EvalException& ee)
        {
            // Exceptions are raised when operands are out of their
            // valid domain (negative input log or division by zero),
            // or outputs a value which is not representable (too
            // large exp or log). The error is logged as level fine
            // because this happens very often when learning continuous
            // functions, and it clogs up the log when logged at a
            // higher level.
            logger().fine()
               << "The following candidate: " << tr << "\n"
               << "has failed to be evaluated, "
               << "raising the following exception: "
               << ee.get_message() << " " << ee.get_vertex();

            return worst_composite_score;
        }

        complexity_t cpxy = bsc.get_complexity(tr);
        score_t cpxy_coef = bsc.get_complexity_coef();

        sum_score += res;
        sum_coef += cpxy_coef;
        sum_penalty += cpxy_coef * cpxy;  // weighted sum !!
    }

    // Weighted average complexity over all results.
    score_t cpxy = sum_penalty / sum_coef;

    if (logger().isFineEnabled()) {
        logger().fine() << "behave_cscore: " << sum_score
                        << " complexity: " << cpxy
                        << " penalty: " << sum_penalty;
    }

    return composite_score(sum_score, cpxy, sum_penalty, 0.0);
}

score_t multibehave_cscore::best_possible_score() const
{
    score_t best = 0.0;
    for (const bscore_base& bsc : _bscorers) {
        best += boost::accumulate(bsc.best_possible_bscore(), 0.0);
    }
    return best;
}

// return the min of all min_improv
score_t multibehave_cscore::min_improv() const
{
    // @todo can be turned in to 1-line with boost::min_element
    // boost::min_element(_bscorers | boost::transformed(/*)
    score_t res = very_best_score;
    for (const bscore_base& bs : _bscorers)
        res = std::min(res, bs.min_improv());
    return res;
}

void multibehave_cscore::ignore_idxs(const std::set<arity_t>& idxs) const
{
    for (const bscore_base& bs : _bscorers)
        bs.ignore_idxs(idxs);
}


//////////////////////////////
// multibscore_based_bscore //
//////////////////////////////
// XXX TODO FIXME  the code below is 'obsolete', see the header file for an
// explanation.

// main operator
behavioral_score multibscore_based_bscore::operator()(const combo_tree& tr) const
{
    behavioral_score bs;
    for (const bscore_base& bsc : _bscorers) {
        behavioral_score abs = bsc(tr);
        boost::push_back(bs, abs);
    }
    return bs;
}

behavioral_score multibscore_based_bscore::best_possible_bscore() const
{
    behavioral_score bs;
    for (const bscore_base& bsc : _bscorers) {
        boost::push_back(bs, bsc.best_possible_bscore());
    }
    return bs;
}

// return the min of all min_improv
score_t multibscore_based_bscore::min_improv() const
{
    // @todo can be turned in to 1-line with boost::min_element
    // boost::min_element(_bscorers | boost::transformed(/*)
    score_t res = very_best_score;
    for (const bscore_base& bs : _bscorers)
        res = std::min(res, bs.min_improv());
    return res;
}

void multibscore_based_bscore::ignore_idxs(const std::set<arity_t>& idxs) const
{
    for (const bscore_base& bs : _bscorers)
        bs.ignore_idxs(idxs);
}
} // ~namespace moses
} // ~namespace opencog
