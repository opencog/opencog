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
    if (logger().getLevel() < Logger::FINE)
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

///////////////////
// contin_bscore //
///////////////////

score_t contin_complexity_coef(unsigned alphabet_size, double stdev) {
    return log(alphabet_size) * 2 * sq(stdev);
}

behavioral_score contin_bscore::operator()(const combo_tree& tr) const
{
    OTable ct(tr, cti, rng);
    behavioral_score bs(target.size() + (occam?1:0));
    boost::transform(ct, target, bs.begin(),
                     [](const vertex& vl, const vertex& vr) {
                         return -sq(get_contin(vl) - get_contin(vr)); });
    // add the Occam's razor feature
    if(occam)
        bs.back() = complexity(tr) * complexity_coef;

    // Logger
    log_candidate_bscore(tr, bs);
    // ~Logger

    return bs;
}

behavioral_score contin_bscore::best_possible_bscore() const {
    return behavioral_score(target.size() + (occam?1:0), 0);
}

void contin_bscore::set_complexity_coef(float alphabet_size, float stdev)
{
    if(occam)
        complexity_coef = contin_complexity_coef(alphabet_size, stdev);
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
}

behavioral_score ctruth_table_bscore::operator()(const combo_tree& tr) const
{
    // The OTable constructor will take the combo tree tr and evaluate
    // it for every row of input in ctt. (The rng is passed straight
    // through to the combo evaluator).
    OTable ptt(tr, ctt, rng);
    behavioral_score bs(ctt.size() + (occam?1:0));

    transform(ptt, ctt | map_values, bs.begin(),
              [](const vertex& v, CTable::mapped_type& vd) {
                  return -score_t(vd[negate_vertex(v)]); });

    // Add the Occam's razor feature
    if (occam)
        bs.back() = complexity(tr) * complexity_coef;

    log_candidate_bscore(tr, bs);

    return bs;
}

behavioral_score ctruth_table_bscore::best_possible_bscore() const
{
    behavioral_score bs(ctt.size() + (occam?1:0));
    transform(ctt | map_values, bs.begin(), [](CTable::mapped_type& vd) {
            return -score_t(min(vd[id::logical_true], vd[id::logical_false])); });
    // add the Occam's razor feature
    if(occam)
        bs.back() = 0;
    return bs;
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
                                                           weight_t log_entropy_w_,
                                                           bool decompose_kld_)
    : ctable(ctable_), rng(_rng),
      kld_w(kld_w_), skewness_w(skewness_w_), stdU_w(stdU_w_),
      skew_U_w(skew_U_w_), log_entropy_w(log_entropy_w_),
      decompose_kld(decompose_kld_)
{
    // initialize Occam's razor
    occam = stdev > 0;
    set_complexity_coef(alphabet_size, stdev);
    // define counter (mapping between observation and its number of occurences)
    boost::for_each(ctable | map_values, [this](const CTable::mapped_type& mv) {
            boost::for_each(mv, [this](const CTable::mapped_type::value_type& v) {
                    counter[get_contin(v.first)] += v.second; }); });
    // precompute pdf
    pdf = counter;
    klds.set_p_pdf(pdf);
    // compute the skewness of pdf
    accumulator_t acc;
    foreach(const auto& v, pdf)
        acc(v.first, weight = v.second);
    skewness = weighted_skewness(acc);    
}

behavioral_score interesting_predicate_bscore::operator()(const combo_tree& tr) const
{
    static const double entropy_threshold = 0.1; /// @todo should be a parameter
    
    OTable pred_ot(tr, ctable, rng);

    // compute the entropy of the output of the predicate
    vector<double> prob;
    Counter<vertex, unsigned> vc;
    boost::for_each(ctable | map_values, pred_ot,
                    [&](const CTable::counter_t& c, const vertex& v) {
                        vc[v] += boost::accumulate(c | map_values, 0); });
    double total = klds.p_size();
    transform(vc | map_values, back_inserter(prob), arg1 / total);
    double pred_entropy = entropy(prob);
    logger().fine("pred_entropy = %f", pred_entropy);

    behavioral_score bs;
    if (pred_entropy > entropy_threshold) {
        // filter the output according to tr
        counter_t pred_counter;     // map obvervation to occurence
                                    // conditioned by predicate truth
        boost::for_each(ctable | map_values, pred_ot,
                        [&](const CTable::counter_t& c, const vertex& v) {
                            if (vertex_to_bool(v)) {
                                foreach(const auto& mv, c)
                                    pred_counter[get_contin(mv.first)] = mv.second;
                            }});

        logger().fine("pred_counter.size() = %u", pred_counter.size());

        // compute KLD
        if(decompose_kld) {
            klds(pred_counter, back_inserter(bs));
            boost::transform(bs, bs.begin(), kld_w * arg1);
        } else
            bs.push_back(kld_w * klds(pred_counter));

        // gather statistics with a boost accumulator
        accumulator_t acc;
        foreach(const auto& v, pred_counter)
            acc(v.first, weight = v.second);
        
        // push the absolute difference between the unconditioned
        // skewness and conditioned one
        score_t diff_skewness = weighted_skewness(acc) - skewness;
        bs.push_back(skewness_w * abs(diff_skewness));

        // compute the standardized Mann–Whitney U
        score_t stdU = standardizedMannWhitneyU(counter, pred_counter);
        bs.push_back(stdU_w * stdU);
        
        // push the product of the relative differences of the shift
        // (stdU) and the skewness (so that if both go in the same
        // direction the value if positive, and negative otherwise)
        bs.push_back(skew_U_w * stdU * diff_skewness);
        
        // add log entropy, this is completely heuristic, no
        // justification except that the log of the entropy is gonna
        // add a large penalty when the entropy tends to 0. Not sure
        // here what we really need, a perhaps something like 1 -
        // p-value, or some form of indefinite TV confidence on the
        // calculation of KLD.
        bs.push_back(log_entropy_w * log(pred_entropy));

        // add the Occam's razor feature
        if(occam)
            bs.push_back(complexity(tr) * complexity_coef);
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

} // ~namespace moses
} // ~namespace opencog
