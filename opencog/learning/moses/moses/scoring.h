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

using opencog::sqr;

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
        } catch (...) {
            stringstream ss;
            ss << "The following candidate has failed to be evaluated: " << tr;
            logger().warn(ss.str());
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
    template<typename Scoring>
    logical_score(const Scoring& score, int a, opencog::RandGen& _rng)
            : target(score, a, _rng), arity(a), rng(_rng) { }

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
    template<typename Scoring>
    logical_bscore(const Scoring& score, int a, opencog::RandGen& _rng)
            : target(score, a, _rng), arity(a), rng(_rng) { }

    behavioral_score operator()(const combo_tree& tr) const;

    combo::truth_table target;
    int arity;
    opencog::RandGen& rng;
};

struct contin_score : public unary_function<combo_tree, score_t> {
    template<typename Scoring>
    contin_score(const Scoring& score,
                 const contin_table_inputs& r,
                 opencog::RandGen& _rng)
            : target(score, r), cti(r), rng(_rng) { }

    contin_score(const combo::contin_table& t,
                 const contin_table_inputs& r,
                 opencog::RandGen& _rng)
        : target(t),cti(r),rng(_rng) { }

    score_t operator()(const combo_tree& tr) const;

    combo::contin_table target;
    contin_table_inputs cti;
    opencog::RandGen& rng;
};

struct contin_score_sqr : public unary_function<combo_tree,score_t> {
    template<typename Scoring>
    contin_score_sqr(const Scoring& score,
                     const contin_table_inputs& r,
                     opencog::RandGen& _rng)
        : target(score,r),cti(r),rng(_rng) { }
    
    contin_score_sqr(const combo::contin_table& t,
                     const contin_table_inputs& r,
                     opencog::RandGen& _rng)
        : target(t),cti(r),rng(_rng) { }
    
    score_t operator()(const combo_tree& tr) const;
    
    combo::contin_table target;
    contin_table_inputs cti;
    opencog::RandGen& rng;
};

struct contin_bscore : public unary_function<combo_tree, behavioral_score> {
    template<typename Scoring>
    contin_bscore(const Scoring& score,
                  const contin_table_inputs& r,
                  opencog::RandGen& _rng)
        : target(score, r), cti(r), rng(_rng) { }

    contin_bscore(const combo::contin_table& t,
                  const contin_table_inputs& r,
                  opencog::RandGen& _rng)
        : target(t), cti(r), rng(_rng) { }

    behavioral_score operator()(const combo_tree& tr) const;

    combo::contin_table target;
    contin_table_inputs cti;
    opencog::RandGen& rng;
};

/**
 * Calculate the log of the density probability dP(D|M), given the sum
 * squared error of D vs M.
 * D represents the data, for instance a contin_table.
 * M represent the model, i.e. a Combo program.
 * Assuming M's outputs describe Guassians of mean M(x) and variance v, 
 * dP(D|M) = Prod_{x\in D} (2*Pi*v)^(-1/2) exp(-(M(x)-D(x))^2/(2*v))
 * = Sum_{x\in D} log((2*Pi*v)^(-1/2)) + log(exp(-(M(x)-D(x))^2/(2*v)))
 * = Sum_{x\in D} log((2*Pi*v)^(-1/2)) -(M(x)-D(x))^2/(2*v)
 * = |D|*log((2*Pi*v)^(-1/2)) -1/(2*v)*Sum_{x\in D} (M(x)-D(x))^2
 *
 * @param sse  sum squared error
 * @param ds   |D|
 * @param v    variance of the Guassian distribution of M's outputs
 * @return     dP(D|M)
*/
struct LogPDM {
    LogPDM(float v) : variance(v) {
        var_term = variance>0 ? 1/sqrt(log(2*PI*variance)) : 0;        
    }
    score_t operator()(score_t sse, unsigned int ds) const {
        return (score_t)ds*var_term - sse/(2*variance);
    }
    float variance;
    score_t var_term; // to speed up computation, precomputes
                      // 1/sqrt(log(2*PI*variance))
};

struct occam_contin_score : public unary_function<combo_tree,score_t> {
    template<typename Scoring>
    occam_contin_score(const Scoring& score,
                       const contin_table_inputs& r,
                       float v,
                       float alphabet_size,
                       opencog::RandGen& _rng)
        : target(score,r), cti(r), variance(v), logPDM(v), rng(_rng) {
        alphabet_size_log = log((double)alphabet_size);    
    }

    occam_contin_score(const combo::contin_table& t,
                       const contin_table_inputs& r,
                       float v,
                       float alphabet_size,
                       opencog::RandGen& _rng)
        : target(t), cti(r), variance(v), logPDM(v), rng(_rng) {
        alphabet_size_log = log((double)alphabet_size);
    }

    score_t operator()(const combo_tree& tr) const;

    combo::contin_table target;
    contin_table_inputs cti;
    score_t variance;
    LogPDM logPDM;
    score_t alphabet_size_log;
    opencog::RandGen& rng;
};

struct occam_contin_bscore : public unary_function<combo_tree, behavioral_score> {
    template<typename Scoring>
    occam_contin_bscore(const Scoring& score,
                        const contin_table_inputs& r,
                        float v,
                        float alphabet_size,
                        opencog::RandGen& _rng)
        : target(score, r), cti(r), variance(v), logPDM(v), rng(_rng) {
        alphabet_size_log_scaled_down = 
            log((double)alphabet_size) / (double)cti.size();
    }

    occam_contin_bscore(const combo::contin_table& t,
                        const contin_table_inputs& r,
                        float v,
                        float alphabet_size,
                        opencog::RandGen& _rng)
        : target(t), cti(r), variance(v), logPDM(v), rng(_rng) {
        alphabet_size_log_scaled_down = 
            log((double)alphabet_size) / (double)cti.size();
    }

    behavioral_score operator()(const combo_tree& tr) const;

    combo::contin_table target;
    contin_table_inputs cti;
    score_t variance;
    LogPDM logPDM;
    score_t alphabet_size_log_scaled_down;
    opencog::RandGen& rng;
};

/**
 * like occam_contin_bscore but for boolean, instead of considering a
 * variance the probability p that one datum is wrong is used.
 */
struct occam_truth_table_bscore 
    : public unary_function<combo_tree, behavioral_score> {
    occam_truth_table_bscore(const partial_truth_table& t,
                             const truth_table_inputs& i,
                             float p,
                             float alphabet_size,
                             opencog::RandGen& _rng) 
        : target(t), tti(i), rng(_rng) {
        occam = p > 0 && p < 1;
        if(occam) {
            log_p = log(p);
            log_cp = log(1-p);
        }
        alphabet_size_log_scaled_down = 
            log((double)alphabet_size) / (double)tti.size();
    }

    behavioral_score operator()(const combo_tree& tr) const;

    const partial_truth_table& target;
    const truth_table_inputs& tti;
    bool occam; // if true the Occam's razor is taken into account
    score_t log_p; // log probability that one datum is wrong
    score_t log_cp; // log probability that one datum is right
    score_t alphabet_size_log_scaled_down;
    opencog::RandGen& rng;
};

template<typename Scoring>
struct complexity_based_scorer : public unary_function<eda::instance,
                                                       combo_tree_score> {
    complexity_based_scorer(const Scoring& s,
                            representation& rep,
                            bool reduce,
                            opencog::RandGen& _rng)
            : score(s), _rep(rep), rng(_rng) { }

    combo_tree_score operator()(const eda::instance& inst) const {
        using namespace reduct;

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
            combo_tree tr = _rep.get_clean_exemplar(_reduce);
            return combo_tree_score(score(tr), complexity(tr.begin()));
        } catch (...) {
            stringstream ss;
            ss << "The following instance has failed to be evaluated: " 
               << _rep.fields().stream(inst);
            logger().warn(ss.str());
            return worst_possible_score;
        }
    }

protected:
    Scoring score;
    representation& _rep;
    bool _reduce; // whether the exemplar is reduced before being
                  // evaluated, this may be advantagous if Scoring is
                  // also a cache
    opencog::RandGen& rng;
};

template<typename Scoring>
struct count_based_scorer : public unary_function<eda::instance, 
                                                  combo_tree_score> {
    count_based_scorer(const Scoring& s,
                       representation& rep,
                       int base_count,
                       bool reduce,
                       opencog::RandGen& _rng)
        : score(s), _base_count(base_count), _rep(rep), _reduce(reduce),
          rng(_rng) {}

    combo_tree_score operator()(const eda::instance& inst) const {
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
            return combo_tree_score(score(_rep.get_clean_exemplar(_reduce)), 
                                    - int(_rep.fields().count(inst))
                                    + _base_count);
        } catch (...) {
            stringstream ss;
            ss << "The following instance has failed to be evaluated: " 
               << _rep.fields().stream(inst);
            logger().warn(ss.str());
            return worst_possible_score;
        }
    }
    
protected:
    Scoring score;
    int _base_count;
    representation& _rep;
    bool _reduce; // whether the exemplar is reduced before being
                  // evaluated, this may be advantagous if Scoring is
                  // also a cache
    opencog::RandGen& rng;
};

/**
 * return true if x dominates y
 *        false if y dominates x
 *        indeterminate otherwise
 */
tribool dominates(const behavioral_score& x, const behavioral_score& y);


/**
 * For all candidates c in [from, to), insert c in dst iff 
 * no element of dst dominates c.
 */
//this may turn out to be too slow...
template<typename It, typename Set>
void merge_nondominating(It from, It to, Set& dst)
{
    for (;from != to;++from) {
        // std::cout << "ook " << std::distance(from,to) << std::endl; // PJ
        bool nondominated = true;
        for (typename Set::iterator it = dst.begin();it != dst.end();) {
            tribool dom = dominates(from->second, it->second);
            if (dom) {
                dst.erase(it++);
            } else if (!dom) {
                nondominated = false;
                break;
            } else {
                ++it;
            }
        }
        if (nondominated)
            dst.insert(*from);
    }
}

} //~namespace moses

#endif
