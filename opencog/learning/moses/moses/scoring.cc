/*
 * opencog/learning/moses/moses/scoring.cc
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
#include "scoring.h"
#include <opencog/util/numeric.h>
#include <boost/range/algorithm/sort.hpp>
#include <cmath>

namespace opencog { namespace moses {

using namespace std;

// helper to log a combo_tree and its behavioral score
inline void log_candidate_bscore(const combo_tree& tr,
                                 const behavioral_score& bs) {
    if(logger().getLevel() >= Logger::FINE) {
        stringstream ss_tr;
        ss_tr << "Evaluate candidate: " << tr;
        logger().fine(ss_tr.str());
        stringstream ss_bsc;
        ss_bsc << "BScored: ";
        ostream_behavioral_score(ss_bsc, bs);
        logger().fine(ss_bsc.str());
    }
}

behavioral_score logical_bscore::operator()(const combo_tree& tr) const
{
    combo::complete_truth_table tt(tr, arity);
    behavioral_score bs(target.size());

    transform(tt.begin(), tt.end(), target.begin(), bs.begin(),
              not_equal_to<bool>()); //not_equal because lower is better

    return bs;
}

behavioral_score contin_bscore::operator()(const combo_tree& tr) const
{
    combo::contin_output_table ct(tr, cti, rng);
    behavioral_score bs(target.size());

    behavioral_score::iterator dst = bs.begin();
    for (combo::contin_output_table::const_iterator it1 = ct.begin(), it2 = target.begin();
         it1 != ct.end();)
        *dst++ = fabs((*it1++) - (*it2++));

    return bs;
}

behavioral_score occam_contin_bscore::operator()(const combo_tree& tr) const
{
    combo::contin_output_table ct(tr, cti, rng);
    behavioral_score bs(target.size() + (occam?1:0));
    behavioral_score::iterator dst = bs.begin();
    for(combo::contin_output_table::const_iterator it1 = ct.begin(),
            it2 = target.begin(); it1 != ct.end(); ++it1, ++it2)
        *dst++ = sq(*it1 - *it2);
    // add the Occam's razor feature
    if(occam)
        *dst = complexity(tr) * complexity_coef;

    // Logger
    log_candidate_bscore(tr, bs);
    // ~Logger

    return bs;
}

discretize_contin_bscore::discretize_contin_bscore(const combo::contin_output_table& ot,
                                                   const contin_input_table& it,
                                                   const vector<contin_t>& thres,
                                                   bool wa,
                                                   RandGen& _rng)
    : target(ot), cit(it), thresholds(thres), weighted_accuracy(wa), rng(_rng),
      weights(thresholds.size() + 1, 1), classes(ot.size()) {
    // enforce that thresholds is sorted
    boost::sort(thresholds);
    // precompute classes
    transform(target.begin(), target.end(), classes.begin(),
              [&](contin_t v) { return this->class_idx(v); });
    // precompute weights
    multiset<size_t> cs(classes.begin(), classes.end());
    if(weighted_accuracy)
        for(size_t i = 0; i < weights.size(); ++i) {
            weights[i] = classes.size() / (float)(weights.size() * cs.count(i));
            // std::cout << "i = " << i << " weight = " << weights[i] << std::endl;
        }
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
                                                  size_t l_idx, size_t u_idx) const
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
    combo::contin_output_table ct(tr, cit, rng);
    behavioral_score bs(target.size());
    transform(ct.begin(), ct.end(), classes.begin(), bs.begin(),
              [&](contin_t res, size_t c_idx) {
                  return (c_idx != this->class_idx(res)) * this->weights[c_idx];
              });
    // Logger
    log_candidate_bscore(tr, bs);
    // ~Logger

    return bs;    
}

void occam_contin_bscore::set_complexity_coef(double variance,
                                              double alphabet_size) {
    if(occam)
        complexity_coef = - log((double)alphabet_size) * 2 * variance;
}

occam_ctruth_table_bscore::occam_ctruth_table_bscore(const ctruth_table& _ctt,
                                                     float p,
                                                     float alphabet_size,
                                                     RandGen& _rng) 
    : ctt(_ctt), rng(_rng) {
    occam = p > 0 && p < 0.5;
    if(occam)
        complexity_coef = log((double)alphabet_size) / log(p/(1-p));
}

behavioral_score occam_ctruth_table_bscore::operator()(const combo_tree& tr) const
{
    truth_output_table ptt(tr, ctt, rng);
    behavioral_score bs(ctt.size() + (occam?1:0));
        
    behavioral_score::iterator dst = bs.begin();
    truth_output_table::const_iterator it1 = ptt.begin();
    ctruth_table::const_iterator it2 = ctt.begin();
    for(; it1 != ptt.end(); ++it1, ++it2) {
        *dst++ = *it1? it2->second.first : it2->second.second;
    }
    // add the Occam's razor feature
    if(occam)
        *dst = complexity(tr) * complexity_coef;

    // Logger
    log_candidate_bscore(tr, bs);
    // ~Logger
    return bs;
}

} // ~namespace moses
} // ~namespace opencog
