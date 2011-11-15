/*
 * opencog/learning/moses/moses/scoring.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _MOSES_SCORING_H
#define _MOSES_SCORING_H

#include <iostream>
#include <fstream>

#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>
#include <opencog/comboreduct/reduct/meta_rules.h>

#include "using.h"
#include "../representation/representation.h"
#include "types.h"
#include "ant_scoring.h"

namespace opencog { namespace moses {

#define NEG_INFINITY INT_MIN
 
typedef float fitness_t;

/**
 * score calculated based on the behavioral score. Useful to avoid
 * redundancy of code and computation in case there is a cache over
 * bscore. The score is calculated as the minus of the sum of the
 * bscore over all features, that is:
 * score = - sum_f BScore(f),
 */
template<typename BScore>
struct bscore_based_score : public unary_function<combo_tree, score_t>
{
    bscore_based_score(const BScore& bs) : bscore(bs) {}
    score_t operator()(const combo_tree& tr) const {
        try {
            behavioral_score bs = bscore(tr);
            score_t res = -std::accumulate(bs.begin(), bs.end(), 0.0);
            // Logger
            if(logger().getLevel() >= Logger::FINE) {
                stringstream ss_tr;
                ss_tr << "bscore_based_score - Candidate: " << tr;
                logger().fine(ss_tr.str());
                stringstream ss_sc;
                ss_sc << "Scored: " << res;
                logger().fine(ss_sc.str());                
            }
            // ~Logger
            return res;
        } catch (EvalException& ee) {
            // Logger
            stringstream ss1;
            ss1 << "The following candidate: " << tr;
            logger().fine(ss1.str());
            stringstream ss2;
            ss2 << "has failed to be evaluated,"
                << " raising the following exception: "
                << ee.get_message() << " " << ee.get_vertex();
            logger().fine(ss2.str());
            // ~Logger
            return get_score(worst_composite_score);
        }
    }
    const BScore& bscore;
};

/**
 * Like logical_score but on behaviors (features). Each feature
 * corresponds to an input tuple, 0 if the output of the candidate
 * matches the output of the intended function (lower is better).
 */
struct logical_bscore : public unary_function<combo_tree, behavioral_score> {
    template<typename Func>
    logical_bscore(const Func& func, int a)
            : target(func, a), arity(a) {}
    logical_bscore(const combo_tree& tr, int a)
            : target(tr, a), arity(a) {}

    behavioral_score operator()(const combo_tree& tr) const;

    combo::complete_truth_table target;
    int arity;
};

struct contin_bscore : public unary_function<combo_tree, behavioral_score> {
    template<typename Func>
    contin_bscore(const Func& func,
                  const contin_input_table& r,
                  RandGen& _rng)
        : target(func, r), cti(r), rng(_rng) {}

    contin_bscore(const combo::contin_output_table& t,
                  const contin_input_table& r,
                  RandGen& _rng)
        : target(t), cti(r), rng(_rng) {}

    behavioral_score operator()(const combo_tree& tr) const;

    combo::contin_output_table target;
    contin_input_table cti;
    RandGen& rng;
};

// Fitness function based on discretization of the output. If the
// classes match the bscore element is 0, or 1 otherwise. If wa (for
// weighted_average is true then each element of the bscore is
// weighted so that each class overall as the same weight in the
// scoring function.
struct discretize_contin_bscore : public unary_function<combo_tree,
                                                        behavioral_score> {

    discretize_contin_bscore(const combo::contin_output_table& ot,
                             const contin_input_table& it,
                             const vector<contin_t>& thres,
                             bool wa,
                             RandGen& _rng);

    behavioral_score operator()(const combo_tree& tr) const;
    combo::contin_output_table target;
    contin_input_table cit;
    vector<contin_t> thresholds;
    bool weighted_accuracy;     // whether the bscore is weighted to
                                // deal with unbalanced data
    RandGen& rng;

protected:
    // return the index of the class of value v
    size_t class_idx(contin_t v) const;
    // like class_idx but assume that the value v is within the class
    // [l_idx, u_idx)
    size_t class_idx_within(contin_t v, size_t l_idx, size_t u_idx) const;

    vector<size_t> classes;       // classes of the output, alligned with target

    // weight of each class so that each one weights as much as the
    // others even in case of unbalance sampling. For specifically:
    // weights[i] = s / (n * c_i) where s is the sample size, n the
    // number of classes and c_i the number of samples for class i.
    vector<score_t> weights;
};

/**
 * the occam_contin_bscore is based on the following thread
 * http://groups.google.com/group/opencog-developers/browse_thread/thread/a4771ecf63d38df?hl=en&pli=1
 *
 * Here's a summary:
 *
 * According to Bayes
 * dP(M|D) = dP(D|M) * P(M) / P(D)
 *
 * Now let's consider the log likelihood of M knowing D, since D is
 * constant we can ignore P(D), so:
 * LL(M) = log(dP(D|M)) + log(P(M))
 * 
 * Assume the output of M on input x has a Guassian noise of mean M(x)
 * and variance v, so dP(D|M) (the density probability)
 * dP(D|M) = Prod_{x\in D} (2*Pi*v)^(-1/2) exp(-(M(x)-D(x))^2/(2*v))
 *
 * Assume
 * P(M) = |A|^-|M|
 * where |A| is the alphabet size.
 *
 * After simplication we can get the following log-likelihood of dP(M|D)
 * -|M|*log(|A|)*2*v - Sum_{x\in D} (M(x)-D(x))^2
 *
 * Each datum corresponds to a feature of the bscore.
 *
 * |M|*log(|A|)*2*v corresponds to an additional feature when v > 0
 */
struct occam_contin_bscore : public unary_function<combo_tree, behavioral_score> {
    template<typename Scoring>
    occam_contin_bscore(const Scoring& score,
                        const contin_input_table& r,
                        float variance,
                        float alphabet_size,
                        RandGen& _rng)
        : target(score, r), cti(r), rng(_rng) {
        occam = variance > 0;
        set_complexity_coef(variance, alphabet_size);
    }

    occam_contin_bscore(const contin_output_table& t,
                        const contin_input_table& r,
                        float variance,
                        float alphabet_size,
                        RandGen& _rng)
        : target(t), cti(r), rng(_rng) {
        occam = variance > 0;
        set_complexity_coef(variance, alphabet_size);
    }

    behavioral_score operator()(const combo_tree& tr) const;

    contin_output_table target;
    contin_input_table cti;
    bool occam;
    score_t complexity_coef;
    RandGen& rng;

private:
    void set_complexity_coef(double variance, double alphabet_size);
};

/**
 * like occam_contin_bscore but for boolean, instead of considering a
 * variance the probability p that one datum is wrong is used.
 *
 * The details are in this thread
 * http://groups.google.com/group/opencog/browse_thread/thread/b7704419e082c6f1?hl=en
 *
 * Briefly after reduction of
 * LL(M) = -|M|*log(|A|) + Sum_{x\in D1} log(p) + Sum_{x\in D2} log(1-p)
 *
 * one gets the following log-likelihood
 * |M|*log|A|/log(p/(1-p)) - |D1|
 * with p<0.5 and |D1| the number of outputs that match
 */
struct occam_ctruth_table_bscore 
    : public unary_function<combo_tree, behavioral_score> {
    occam_ctruth_table_bscore(const ctruth_table& _ctt,
                              float p,
                              float alphabet_size,
                              RandGen& _rng);

    behavioral_score operator()(const combo_tree& tr) const;

    const ctruth_table& ctt;
    bool occam; // if true the Occam's razor is taken into account
    score_t complexity_coef;
    RandGen& rng;
};

// for testing only
struct dummy_score : public unary_function<combo_tree, score_t> {
    score_t operator()(const combo_tree& tr) const {
        return score_t();
    }
};

// for testing only
struct dummy_bscore : public unary_function<combo_tree, behavioral_score> {
    behavioral_score operator()(const combo_tree& tr) const {
        return behavioral_score();
    }
};

/**
 * Mostly for testing the optimization algos, returns minus the
 * hamming distance of the candidate to a given target instance and
 * constant null complexity.
 */
struct distance_based_scorer : public unary_function<instance,
                                                     composite_score> {
    distance_based_scorer(const field_set& _fs,
                          const instance& _target_inst)
        : fs(_fs), target_inst(_target_inst) {}

    composite_score operator()(const instance& inst) const {
        score_t sc = -fs.hamming_distance(target_inst, inst);
        // Logger
        if(logger().getLevel() >= Logger::FINE) {
            stringstream ss;
            ss << "distance_based_scorer - Evaluate instance: " 
               << fs.stream(inst) << std::endl << "Score = " << sc << std::endl;
            logger().fine(ss.str());
        }
        // ~Logger
        return composite_score(sc, 0);
    }

protected:
    const field_set& fs;
    const instance& target_inst;
};

template<typename Scoring>
struct complexity_based_scorer : public unary_function<instance,
                                                       composite_score> {
    complexity_based_scorer(const Scoring& s, representation& rep, bool reduce)
        : score(s), _rep(rep), _reduce(reduce) {}

    composite_score operator()(const instance& inst) const {
        using namespace reduct;

        // Logger
        if(logger().getLevel() >= Logger::FINE) {
            stringstream ss;
            ss << "complexity_based_scorer - Evaluate instance: " 
               << _rep.fields().stream(inst);
            logger().fine(ss.str());
        }
        // ~Logger

        try {
            combo_tree tr = _rep.get_candidate(inst, _reduce);
            return composite_score(score(tr), complexity(tr));
        } catch (...) {
            stringstream ss;
            ss << "The following instance has failed to be evaluated: " 
               << _rep.fields().stream(inst);
            logger().fine(ss.str());
            return worst_composite_score;
        }
    }

protected:
    const Scoring& score;
    representation& _rep;
    bool _reduce; // whether the exemplar is reduced before being
                  // evaluated, this may be advantagous if Scoring is
                  // also a cache
};

} //~namespace moses
} //~namespace opencog

#endif
