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
            it2 = target.begin(); it1 != ct.end(); ++it1, ++it2) {
        *dst++ = sq(*it1 - *it2);
    }
    // add the Occam's razor feature
    if(occam)
        *dst = complexity(tr) * complexity_coef;

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
