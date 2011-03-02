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

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>
#include <opencog/comboreduct/reduct/meta_rules.h>

#include "using.h"
#include "representation.h"
#include "types.h"
#include "ant_scoring.h"

namespace moses
{

using opencog::sq;

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
            if(logger().getLevel() >= opencog::Logger::FINE) {
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
            logger().warn(ss1.str());
            stringstream ss2;
            ss2 << "has failed to be evaluated,"
                << " raising the following exception: "
                << ee.get_message() << " " << ee.get_vertex();
            logger().warn(ss2.str());
            // ~Logger
            return get_score(worst_possible_score);
        }
    }
    const BScore& bscore;
};

/**
 * Compute the score of a boolean function in term of hamming distance
 * of its output and the output of the entire truth table of the
 * indended function.
 */
struct logical_score : public unary_function<combo_tree, int> {
    template<typename Func>
    logical_score(const Func& func, int a, opencog::RandGen& _rng)
            : target(func, a, _rng), arity(a), rng(_rng) { }

    int operator()(const combo_tree& tr) const;

    combo::truth_table target;
    int arity;
    opencog::RandGen& rng;
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

    combo::truth_table target;
    int arity;
};

struct contin_score : public unary_function<combo_tree, score_t> {
    template<typename Func>
    contin_score(const Func& func,
                 const contin_input_table& r,
                 opencog::RandGen& _rng)
            : target(func, r), cti(r), rng(_rng) { }

    contin_score(const combo::contin_table& t,
                 const contin_input_table& r,
                 opencog::RandGen& _rng)
        : target(t),cti(r),rng(_rng) { }

    score_t operator()(const combo_tree& tr) const;

    combo::contin_table target;
    contin_input_table cti;
    opencog::RandGen& rng;
};

struct contin_score_sq : public unary_function<combo_tree,score_t> {
    template<typename Func>
    contin_score_sq(const Func& func,
                    const contin_input_table& r,
                    opencog::RandGen& _rng)
        : target(func,r),cti(r),rng(_rng) { }
    
    contin_score_sq(const combo::contin_table& t,
                    const contin_input_table& r,
                    opencog::RandGen& _rng)
        : target(t),cti(r),rng(_rng) { }
    
    score_t operator()(const combo_tree& tr) const;
    
    combo::contin_table target;
    contin_input_table cti;
    opencog::RandGen& rng;
};

struct contin_bscore : public unary_function<combo_tree, behavioral_score> {
    template<typename Func>
    contin_bscore(const Func& func,
                  const contin_input_table& r,
                  opencog::RandGen& _rng)
        : target(func, r), cti(r), rng(_rng) { }

    contin_bscore(const combo::contin_table& t,
                  const contin_input_table& r,
                  opencog::RandGen& _rng)
        : target(t), cti(r), rng(_rng) { }

    behavioral_score operator()(const combo_tree& tr) const;

    combo::contin_table target;
    contin_input_table cti;
    opencog::RandGen& rng;
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
                        opencog::RandGen& _rng)
        : target(score, r), cti(r), rng(_rng) {
        occam = variance > 0;
        set_complexity_coef(variance, alphabet_size);
    }

    occam_contin_bscore(const combo::contin_table& t,
                        const contin_input_table& r,
                        float variance,
                        float alphabet_size,
                        opencog::RandGen& _rng)
        : target(t), cti(r), rng(_rng) {
        occam = variance > 0;
        set_complexity_coef(variance, alphabet_size);
    }

    behavioral_score operator()(const combo_tree& tr) const;

    combo::contin_table target;
    mutable contin_input_table cti; // mutable due to set_consider_args
    bool occam;
    score_t complexity_coef;
    opencog::RandGen& rng;

private:
    void set_complexity_coef(double variance, double alphabet_size);
};

/** 
 * like occam_truth_bscore but only binds variables that are present
 * in the candidate. This optimization is useful when the candidates
 * tend to be short, and both the number of inputs and the number of
 * samples are very large.
 */
struct occam_contin_bscore_opt_binding : public occam_contin_bscore {
    occam_contin_bscore_opt_binding(const combo::contin_table& t,
                                    const contin_input_table& r,
                                    float variance,
                                    float alphabet_size,
                                    opencog::RandGen& _rng) :
        occam_contin_bscore(t, r, variance, alphabet_size, _rng) {}

    behavioral_score operator()(const combo_tree& tr) const {
        cti.set_consider_args(argument_set(tr)); // to speed up binding
        return occam_contin_bscore::operator()(tr);
    }
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
struct occam_truth_table_bscore 
    : public unary_function<combo_tree, behavioral_score> {
    occam_truth_table_bscore(const partial_truth_table& t,
                             const truth_table_inputs& i,
                             float p,
                             float alphabet_size,
                             opencog::RandGen& _rng) 
        : target(t), tti(i), rng(_rng) {
        occam = p > 0 && p < 0.5;
        if(occam)
            complexity_coef = log((double)alphabet_size) / log(p/(1-p));
    }

    behavioral_score operator()(const combo_tree& tr) const;

    partial_truth_table target;
    mutable truth_table_inputs tti; // mutable due to set_consider_args
    bool occam; // if true the Occam's razor is taken into account
    score_t complexity_coef;
    opencog::RandGen& rng;
};

/** 
 * like occam_truth_bscore but only binds variables that are present
 * in the candidate. This optimization is useful when the candidates
 * tend to be short, and both the number of inputs and the number of
 * samples are very large.
 */
struct occam_truth_table_bscore_opt_binding : public occam_truth_table_bscore {
    occam_truth_table_bscore_opt_binding(const partial_truth_table& t,
                                         const truth_table_inputs& i,
                                         float p,
                                         float alphabet_size,
                                         opencog::RandGen& _rng) :
        occam_truth_table_bscore(t, i, p, alphabet_size, _rng) {}

    behavioral_score operator()(const combo_tree& tr) const {
        tti.set_consider_args(argument_set(tr)); // to speed up binding
        return occam_truth_table_bscore::operator()(tr);
    }
};

/**
 * Mostly for testing the optimization algos, returns minus the
 * hamming distance of the candidate to a given target instance and
 * constant null complexity.
 */
struct distance_based_scorer : public unary_function<eda::instance,
                                                     composite_score> {
    distance_based_scorer(const eda::field_set& _fs,
                          const eda::instance& _target_inst)
        : fs(_fs), target_inst(_target_inst) {}

    composite_score operator()(const eda::instance& inst) const {
        return composite_score(-fs.hamming_distance(target_inst, inst), 0);
    }

protected:
    const eda::field_set& fs;
    const eda::instance& target_inst;
};

template<typename Scoring>
struct complexity_based_scorer : public unary_function<eda::instance,
                                                       composite_score> {
    complexity_based_scorer(const Scoring& s, representation& rep, bool reduce)
        : score(s), _rep(rep), _reduce(reduce) {}

    composite_score operator()(const eda::instance& inst) const {
        using namespace reduct;

        // Logger
        if(logger().getLevel() >= opencog::Logger::FINE) {
            stringstream ss;
            ss << "complexity_based_scorer - Evaluate instance: " 
               << _rep.fields().stream(inst);
            logger().fine(ss.str());
        }
        // ~Logger

        _rep.transform(inst);

        try {
            combo_tree tr = _rep.get_clean_exemplar(_reduce);
            return composite_score(score(tr), complexity(tr.begin()));
         } catch (...) {
             stringstream ss;
             ss << "The following instance has failed to be evaluated: " 
                << _rep.fields().stream(inst);
             logger().warn(ss.str());
             return worst_possible_score;
         }
    }

protected:
    const Scoring& score;
    representation& _rep;
    bool _reduce; // whether the exemplar is reduced before being
                  // evaluated, this may be advantagous if Scoring is
                  // also a cache
};

template<typename Scoring>
struct count_based_scorer : public unary_function<eda::instance, 
                                                  composite_score> {
    count_based_scorer(const Scoring& s, representation& rep,
                       int base_count, bool reduce)
        : score(s), _base_count(base_count), _rep(rep), _reduce(reduce) {}

    composite_score operator()(const eda::instance& inst) const {
        // Logger
        if(logger().getLevel() >= opencog::Logger::FINE) {
            stringstream ss;
            ss << "count_based_scorer - Evaluate instance: " 
               << _rep.fields().stream(inst);
            logger().fine(ss.str());
        }
        // ~Logger

        _rep.transform(inst);

        try {
            return composite_score(score(_rep.get_clean_exemplar(_reduce)), 
                                   - int(_rep.fields().count(inst))
                                   + _base_count);
        } catch(...) {
             stringstream ss;
             ss << "The following instance has failed to be evaluated: " 
                << _rep.fields().stream(inst);
             logger().warn(ss.str());
             return worst_possible_score;
         }
    }
    
protected:
    const Scoring& score;
    int _base_count;
    representation& _rep;
    bool _reduce; // whether the exemplar is reduced before being
                  // evaluated, this may be advantagous if Scoring is
                  // also a cache
};


} //~namespace moses

#endif
