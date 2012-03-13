/*
 * opencog/learning/moses/moses/scoring.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks, Nil Geisweiller
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
#include "scoring.h"

#include <cmath>

#include <boost/range/algorithm/sort.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm_ext/for_each.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

#include <opencog/util/numeric.h>
#include <opencog/util/KLD.h>
#include <opencog/util/MannWhitneyU.h>

namespace opencog { namespace moses {

using namespace std;
using boost::adaptors::map_values;
using boost::adaptors::map_keys;
using boost::adaptors::filtered;
using boost::transform;
using namespace boost::phoenix;
using boost::phoenix::arg_names::arg1;
using namespace boost::accumulators;

// helper to log a combo_tree and its behavioral score
inline void log_candidate_bscore(const combo_tree& tr,
                                 const behavioral_score& bs)
{
    if (!logger().isFineEnabled())
        return;

    stringstream ss;
    ss << "Evaluate candidate: " << tr << "\n";
    ss << "\tBScored: " << bs;
    logger().fine(ss.str());
}

////////////////////
// logical_bscore //
////////////////////
        
behavioral_score logical_bscore::operator()(const combo_tree& tr) const
{
    combo::complete_truth_table tt(tr, arity);
    behavioral_score bs(target.size());

    boost::transform(tt, target, bs.begin(), [](bool b1, bool b2) {
            return -score_t(b1 != b2); });

    return bs;
}

behavioral_score logical_bscore::best_possible_bscore() const {
    return behavioral_score(target.size(), 0);
}

score_t logical_bscore::min_improv() const
{
    return 0.5;
}

///////////////////
// contin_bscore //
///////////////////

score_t contin_complexity_coef(unsigned alphabet_size, double stdev)
{
    return log(alphabet_size) * 2 * sq(stdev);
}

behavioral_score contin_bscore::operator()(const combo_tree& tr) const
{
    // cti == ITable of inputs.
    // OTable ct will contain evaluation of tree tr on the inputs cti.
    OTable ct(tr, cti, rng);

    // OTable target is the table of output we want to get.
    behavioral_score bs(target.size() + (occam?1:0));

    // boost/range/algorithm/transform.
    // take two arrays ct, target, feed the elts to anon funtion[]
    // (which just computes square of the difference) and put the
    // results into bs.
    boost::transform(ct, target, bs.begin(),
                     [](const vertex& vl, const vertex& vr) {
                         return -sq(get_contin(vl) - get_contin(vr)); });
    // add the Occam's razor feature
    if (occam)
        bs.back() = complexity(tr) * complexity_coef;

    // Logger
    log_candidate_bscore(tr, bs);
    // ~Logger

    return bs;
}

behavioral_score contin_bscore::best_possible_bscore() const
{
    return behavioral_score(target.size() + (occam?1:0), 0);
}

score_t contin_bscore::min_improv() const
{
    // The backwards compat version of this is 0.0.  But for
    // continuously-variable scores, this is crazy, as the
    // system falls into a state of tweaking the tenth decimal place,
    // Limit any such tweaking to 4 decimal places of precision.
    // (thus 1e-4 below).
    //
    // Note: positive min_improv is taken as an absolute score.
    // Negative min_improve is treated as a relative score.
    return -1.0e-4;
}
        
void contin_bscore::set_complexity_coef(float alphabet_size, float stdev)
{
    if(occam)
        complexity_coef = contin_complexity_coef(alphabet_size, stdev);
}

//////////////////////
// precision_bscore //
//////////////////////

precision_bscore::precision_bscore(const CTable& _ctable,
                                   float alphabet_size, float p,
                                   float min_activation_, float max_activation_,
                                   float penalty_,
                                   RandGen& _rng, bool positive_)
    : ctable(_ctable), ctable_usize(ctable.uncompressed_size()),
      min_activation(min_activation_), max_activation(max_activation_),
      penalty(penalty_), rng(_rng), positive(positive_)
{
    // Both p==0.0 and p==0.5 are singularity points in the Occam's
    // razor formula for discrete outputs (see the explanation in the
    // comment above ctruth_table_bscore)
    occam = p > 0.0f && p < 0.5f;
    if (occam)
        complexity_coef = discrete_complexity_coef(alphabet_size, p);

    type_node output_type = *type_tree_output_type_tree(ctable.tt).begin();
    if (output_type == id::boolean_type) {
        vertex target = bool_to_vertex(positive);
        sum_outputs = [target](CTable::counter_t& c)->score_t {
            return c[target];
        };
    } else if (output_type == id::contin_type) {
        sum_outputs = [this](CTable::counter_t& c)->score_t {
            score_t res = 0.0;
            foreach(const CTable::counter_t::value_type& cv, c)
                res += get_contin(cv.first) * cv.second;
            return (positive? res : -res);
        };
    }
}

behavioral_score precision_bscore::operator()(const combo_tree& tr) const
{
    // The OTable constructor takes the combo tree tr and evaluate it
    // for every row of input in ctt. (The rng is passed straight
    // through to the combo evaluator).
    OTable ot(tr, ctable, rng);
    behavioral_score bs;
    
    vertex target = bool_to_vertex(positive);
    unsigned active = 0;   // total number of active outputs by tr
    score_t sao = 0.0;     // sum of all active outputs (in the boolean case)
    boost::for_each(ctable | map_values, ot,
                    [&](CTable::counter_t& c, const vertex& v) {
                        if (v == id::logical_true) {
                            sao += sum_outputs(c);
                            active += c.total_count();
                        }});

    // add precision component
    score_t precision = sao / active,
        activation = (score_t)active / ctable_usize;
    logger().fine("precision = %f", precision);
    bs.push_back(precision);
    
    // add activation_penalty component
    score_t activation_penalty = get_activation_penalty(activation);
    logger().fine("activation = %f", activation);
    logger().fine("activation penalty = %e", activation_penalty);
    bs.push_back(activation_penalty);
    
    // Add the Occam's razor component
    if (occam)                  // we divide by the number of active
                                // observations as to normalize the
                                // Occam's razor as well
        bs.push_back(complexity(tr) * complexity_coef / active);

    log_candidate_bscore(tr, bs);

    return bs;
}

behavioral_score precision_bscore::best_possible_bscore() const
{
    // the best possible precision is 1 with no activation penalty
    behavioral_score bs = {1, 0};

    // add the Occam's razor feature
    if(occam)
        bs.push_back(0);
    return bs;
}

score_t precision_bscore::get_activation_penalty(score_t activation) const
{
    score_t dst = max(max(min_activation - activation, score_t(0))
                      / min_activation,
                      max(activation - max_activation, score_t(0))
                      / (1 - max_activation));
    logger().fine("dst = %f", dst);
    return log(pow((1 - dst), penalty));
}

score_t precision_bscore::min_improv() const
{
    return 0.0;                 // not necessarily right, just the
                                // backward behavior
}
        
//////////////////////////////
// discretize_contin_bscore //
//////////////////////////////
        
score_t discrete_complexity_coef(unsigned alphabet_size, double p) {
    return -log((double)alphabet_size) / log(p/(1-p));
}

discretize_contin_bscore::discretize_contin_bscore(const OTable& ot,
                                                   const ITable& it,
                                                   const vector<contin_t>& thres,
                                                   bool wa,
                                                   float alphabet_size,
                                                   float p,
                                                   RandGen& _rng)
    : target(ot), cit(it), thresholds(thres), weighted_accuracy(wa), rng(_rng),
      classes(ot.size()), weights(thresholds.size() + 1, 1) {
    // enforce that thresholds is sorted
    boost::sort(thresholds);
    // precompute classes
    boost::transform(target, classes.begin(), [&](const vertex& v) {
            return this->class_idx(get_contin(v)); });
    // precompute weights
    multiset<size_t> cs(classes.begin(), classes.end());
    if(weighted_accuracy)
        for(size_t i = 0; i < weights.size(); ++i)
            weights[i] = classes.size() / (float)(weights.size() * cs.count(i));
    // precompute Occam's razor coefficient
    occam = p > 0 && p < 0.5;
    if(occam)
        complexity_coef = discrete_complexity_coef(alphabet_size, p);    
}

behavioral_score discretize_contin_bscore::best_possible_bscore() const {
    return behavioral_score(target.size(), 0);
}

score_t discretize_contin_bscore::min_improv() const
{
    return 0.0;                 // not necessarily right, just the
                                // backward behavior
}

size_t discretize_contin_bscore::class_idx(contin_t v) const {
    if(v < thresholds[0])
        return 0;
    size_t s = thresholds.size();
    if(v >= thresholds[s - 1])
        return s;
    return class_idx_within(v, 1, s);
}

size_t discretize_contin_bscore::class_idx_within(contin_t v,
                                                  size_t l_idx,
                                                  size_t u_idx) const
{
    // base case
    if(u_idx - l_idx == 1)
        return l_idx;
    // recursive case
    size_t m_idx = l_idx + (u_idx - l_idx) / 2;
    contin_t t = thresholds[m_idx - 1];
    if(v < t)
        return class_idx_within(v, l_idx, m_idx);
    else
        return class_idx_within(v, m_idx, u_idx);
}

behavioral_score discretize_contin_bscore::operator()(const combo_tree& tr) const
{
    OTable ct(tr, cit, rng);
    behavioral_score bs(target.size() + (occam?1:0));
    boost::transform(ct, classes, bs.begin(), [&](const vertex& v, size_t c_idx) {
            return (c_idx != this->class_idx(get_contin(v))) * this->weights[c_idx];
        });
    // add the Occam's razor feature
    if(occam)
        bs.back() = complexity(tr) * complexity_coef;

    // Logger
    log_candidate_bscore(tr, bs);
    // ~Logger

    return bs;    
}

/////////////////////////
// ctruth_table_bscore //
/////////////////////////
        
ctruth_table_bscore::ctruth_table_bscore(const CTable& _ctt,
                                         float alphabet_size, float p,
                                         RandGen& _rng)
    : ctt(_ctt), rng(_rng)
{
    // Both p==0.0 and p==0.5 are singularity points in the Occam's
    // razor formula for discrete outputs (see the explanation in the
    // comment above ctruth_table_bscore)
    occam = p > 0.0f && p < 0.5f;
    if (occam)
        complexity_coef = discrete_complexity_coef(alphabet_size, p);

    // define func
    func = [](const vertex& v, CTable::counter_t& c) {
        return -score_t(c[negate_vertex(v)]); };

    // define best_func
    best_func = [](CTable::counter_t& c) {
        return -score_t(min(c[id::logical_true], c[id::logical_false]));};
}

behavioral_score ctruth_table_bscore::operator()(const combo_tree& tr) const
{
    // The OTable constructor will take the combo tree tr and evaluate
    // it for every row of input in ctt. (The rng is passed straight
    // through to the combo evaluator).
    OTable ptt(tr, ctt, rng);
    behavioral_score bs;

    // Evaluate the bscore components for all rows of the ctable
    transform(ptt, ctt | map_values, back_inserter(bs), func);

    // Add the Occam's razor feature
    if (occam)
        bs.push_back(complexity(tr) * complexity_coef);

    log_candidate_bscore(tr, bs);

    return bs;
}

behavioral_score ctruth_table_bscore::best_possible_bscore() const
{
    behavioral_score bs;
    transform(ctt | map_values, back_inserter(bs), best_func);

    // add the Occam's razor feature
    if(occam)
        bs.push_back(0);
    return bs;
}

score_t ctruth_table_bscore::min_improv() const
{
    return 0.5;
}

//////////////////////////////////
// interesting_predicate_bscore //
//////////////////////////////////

interesting_predicate_bscore::interesting_predicate_bscore(const CTable& ctable_,
                                                           float alphabet_size,
                                                           float stdev,
                                                           RandGen& _rng,
                                                           weight_t kld_w_,
                                                           weight_t skewness_w_,
                                                           weight_t stdU_w_,
                                                           weight_t skew_U_w_,
                                                           score_t min_activation_,
                                                           score_t max_activation_,
                                                           score_t penalty_,
                                                           bool positive_,
                                                           bool abs_skewness_,
                                                           bool decompose_kld_)
    : ctable(ctable_), rng(_rng),
      kld_w(kld_w_), skewness_w(skewness_w_), abs_skewness(abs_skewness_),
      stdU_w(stdU_w_), skew_U_w(skew_U_w_), min_activation(min_activation_),
      max_activation(max_activation_), penalty(penalty_), positive(positive_),
      decompose_kld(decompose_kld_)
{
    // initialize Occam's razor
    occam = stdev > 0;
    if (occam)
        set_complexity_coef(alphabet_size, stdev);
    // define counter (mapping between observation and its number of occurences)
    boost::for_each(ctable | map_values, [this](const CTable::mapped_type& mv) {
            boost::for_each(mv, [this](const CTable::mapped_type::value_type& v) {
                    counter[get_contin(v.first)] += v.second; }); });
    // precompute pdf
    if (kld_w > 0) {
        pdf = counter;
        klds.set_p_pdf(pdf);
    }
    // compute the skewness of pdf
    accumulator_t acc;
    foreach(const auto& v, pdf)
        acc(v.first, weight = v.second);
    skewness = weighted_skewness(acc);
    logger().fine("skewness = %f", skewness);
}

behavioral_score interesting_predicate_bscore::operator()(const combo_tree& tr) const
{
    OTable pred_ot(tr, ctable, rng);

    vertex target = bool_to_vertex(positive);
    
    unsigned total = 0, // total number of observations (could be optimized)
        actives = 0; // total number of positive (or negative if
                     // positive is false) predicate values
    boost::for_each(ctable | map_values, pred_ot,
                    [&](const CTable::counter_t& c, const vertex& v) {
                        unsigned tc = c.total_count();
                        if (v == target)
                            actives += tc;
                        total += tc;
                    });

    logger().fine("total = %u", total);
    logger().fine("actives = %u", actives);

    behavioral_score bs;

    // filter the output according to pred_ot
    counter_t pred_counter;     // map obvervation to occurence
                                // conditioned by predicate truth
    boost::for_each(ctable | map_values, pred_ot,
                    [&](const CTable::counter_t& c, const vertex& v) {
                        if (v == target) {
                            foreach(const auto& mv, c)
                                pred_counter[get_contin(mv.first)] = mv.second;
                        }});

    logger().fine("pred_ot.size() = %u", pred_ot.size());
    logger().fine("pred_counter.size() = %u", pred_counter.size());

    if (pred_counter.size() > 1) { // otherwise the statistics are
                                   // messed up (for instance
                                   // pred_skewness can be inf)
        // compute KLD
        if (kld_w > 0) {
            if(decompose_kld) {
                klds(pred_counter, back_inserter(bs));
                boost::transform(bs, bs.begin(), kld_w * arg1);
            } else {
                score_t pred_klds = klds(pred_counter);
                logger().fine("klds = %f", pred_klds);
                bs.push_back(kld_w * pred_klds);
            }
        }

        if (skewness_w > 0 || stdU_w > 0 || skew_U_w > 0) {
            
            // gather statistics with a boost accumulator
            accumulator_t acc;
            foreach(const auto& v, pred_counter)
                acc(v.first, weight = v.second);

            score_t diff_skewness = 0;
            if (skewness_w > 0 || skew_U_w > 0) {
                // push the absolute difference between the
                // unconditioned skewness and conditioned one
                score_t pred_skewness = weighted_skewness(acc);
                diff_skewness = pred_skewness - skewness;
                score_t val_skewness = (abs_skewness?
                                        abs(diff_skewness):
                                        diff_skewness);
                logger().fine("pred_skewness = %f", pred_skewness);
                if (skewness_w > 0)
                    bs.push_back(skewness_w * val_skewness);
            }

            score_t stdU = 0;
            if (stdU_w > 0 || skew_U_w > 0) {

                // compute the standardized Mann–Whitney U
                stdU = standardizedMannWhitneyU(counter, pred_counter);
                logger().fine("stdU = %f", stdU);
                if (stdU_w > 0)
                    bs.push_back(stdU_w * abs(stdU));
            }
                
            // push the product of the relative differences of the
            // shift (stdU) and the skewness (so that if both go
            // in the same direction the value if positive, and
            // negative otherwise)
            if (skew_U_w > 0)
                bs.push_back(skew_U_w * stdU * diff_skewness);
        }
            
        // add activation_penalty component
        score_t activation = actives / (score_t) total,
            activation_penalty = get_activation_penalty(activation);
        logger().fine("activation = %f", activation);
        logger().fine("activation penalty = %e", activation_penalty);
        bs.push_back(activation_penalty);
        
        // add the Occam's razor feature
        if(occam)
            bs.push_back(complexity(tr) * complexity_coef);
    } else {
        bs.push_back(worst_score);
    }

    // Logger
    log_candidate_bscore(tr, bs);
    // ~Logger

    return bs;
}

behavioral_score interesting_predicate_bscore::best_possible_bscore() const
{
    return behavioral_score(1, best_score);
}

void interesting_predicate_bscore::set_complexity_coef(float alphabet_size,
                                                       float stdev)
{
    if(occam)
        complexity_coef = contin_complexity_coef(alphabet_size, stdev);
}

score_t interesting_predicate_bscore::get_activation_penalty(score_t activation) const
{
    score_t dst = max(max(min_activation - activation, score_t(0))
                      / min_activation,
                      max(activation - max_activation, score_t(0))
                      / (1 - max_activation));
    logger().fine("dst = %f", dst);
    return log(pow((1 - dst), penalty));
}

score_t interesting_predicate_bscore::min_improv() const
{
    return 0.0;                 // not necessarily right, just the
                                // backward behavior
}

} // ~namespace moses
} // ~namespace opencog
