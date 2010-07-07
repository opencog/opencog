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

namespace moses
{

int logical_score::operator()(const combo_tree& tr) const
{
    // std::cout << "scoring " << tr << " " << arity << " "
    //   << target << " " << combo::truth_table(tr,arity) << " "
    //   << (target.hamming_distance(combo::truth_table(tr,arity))) << std::endl; // PJ

    return -int(target.hamming_distance(combo::truth_table(tr, arity, rng)));
}
behavioral_score logical_bscore::operator()(const combo_tree& tr) const
{
    combo::truth_table tt(tr, arity, rng);
    behavioral_score bs(target.size());

    std::transform(tt.begin(), tt.end(), target.begin(), bs.begin(),
                   std::not_equal_to<bool>()); //not_equal because lower is better

    return bs;
}

contin_t contin_score::operator()(const combo_tree& tr) const
{
    try {
        std::cout << "scoring " << tr << std::endl;
        contin_t sc = -target.abs_distance(combo::contin_table(tr, rands, rng));
        std::cout << sc << " X " << tr << std::endl;
        return sc;
    } catch (...) {
        std::cout << "threw" << std::endl;
        return get_score(worst_possible_score);
    }
}

contin_t contin_score_sqr::operator()(const combo_tree& tr) const
{
    try {
        return -target.sum_squared_error(combo::contin_table(tr, rands, rng));
    } catch (...) {
        stringstream ss;
        ss << "The following candidate has failed to be evaluated: " << tr;
        logger().warn(ss.str());
        return get_score(worst_possible_score);
    }
}

behavioral_score contin_bscore::operator()(const combo_tree& tr) const
{
    combo::contin_table ct(tr, rands, rng);
    behavioral_score bs(target.size());

    behavioral_score::iterator dst = bs.begin();
    for (combo::contin_table::const_iterator it1 = ct.begin(), it2 = target.begin();
         it1 != ct.end();)
        *dst++ = fabs((*it1++) - (*it2++));

    return bs;
}

score_t occam_contin_score::operator()(const combo_tree& tr) const
{
    try {
        score_t sse = 
            target.sum_squared_error(combo::contin_table(tr, rands, rng));
        if(variance > 0) 
            return logPDM(sse, target.size())
                - (score_t)tr.size()*alphabet_size_log; // occam's razor
        else 
            return -sse; // no occam's razor
    } catch (...) {
        stringstream ss;
        ss << "The following candidate has failed to be evaluated: " << tr;
        logger().warn(ss.str());
        return get_score(worst_possible_score);
    }
}

behavioral_score occam_contin_bscore::operator()(const combo_tree& tr) const
{
    combo::contin_table ct(tr, rands, rng);
    behavioral_score bs(target.size());
    score_t trs = tr.size(); //@todo replace this with complexity
        
    behavioral_score::iterator dst = bs.begin();
    for(combo::contin_table::const_iterator it1 = ct.begin(), 
             it2 = target.begin(); it1 != ct.end();)
        if(variance > 0) 
            *dst++ = trs*alphabet_size_log // occam's razor
                - logPDM(sqr((*it1++) - (*it2++)), 1);
        else
            *dst++ = sqr((*it1++) - (*it2++)); // no occam's razor
    // Logger
    if(logger().getLevel() >= opencog::Logger::FINE) {
        stringstream ss_tr;
        ss_tr << "Candidate: " << tr;
        if(ss_tr.str() == "Candidate: +(*(+(*(+(*(+(#1 1) #1 -0.375) -0.5) +(#1 1.75)) 0.5) #1) 7.5) ") {
            std::cout << "HERE!" << std::endl;
        }
        logger().fine(ss_tr.str());
        stringstream ss_bsc;
        ostream_behavioral_score(ss_bsc, bs);
        logger().fine(ss_bsc.str());
    }
    // ~Logger
    return bs;
}

// TODO
// behavioral_score occam_boolean_bscore::operator()(const combo_tree& tr) const
// {
//     combo::contin_table ct(tr, rands, rng);
//     behavioral_score bs(target.size());
//     score_t trs = tr.size(); //@todo replace this with complexity
        
//     behavioral_score::iterator dst = bs.begin();
//     for(combo::contin_table::const_iterator it1 = ct.begin(), 
//              it2 = target.begin(); it1 != ct.end();)
//         if(variance > 0) 
//             *dst++ = trs*alphabet_size_log // occam's razor
//                 - logPDM(sqr((*it1++) - (*it2++)), 1);
//         else
//             *dst++ = sqr((*it1++) - (*it2++)); // no occam's razor
//     // Logger
//     if(logger().getLevel() >= opencog::Logger::FINE) {
//         stringstream ss_tr;
//         ss_tr << "Candidate: " << tr;
//         logger().fine(ss_tr.str());
//         stringstream ss_bsc;
//         ostream_behavioral_score(ss_bsc, bs);
//         logger().fine(ss_bsc.str());
//     }
//     // ~Logger
//     return bs;
// }

/**
 * return true if x dominates y
 *        false if y dominates x
 *        indeterminate otherwise
 */
tribool dominates(const behavioral_score& x, const behavioral_score& y)
{
    //everything dominates an empty vector
    if (x.empty()) {
        if (y.empty())
            return indeterminate;
        return false;
    } else if (y.empty()) {
        return true;
    }

    tribool res = indeterminate;
    for (behavioral_score::const_iterator xit = x.begin(), yit = y.begin();
            xit != x.end();++xit, ++yit) {
        if (*xit < *yit) { //individual elements are assumed to represent error
            if (!res)
                return indeterminate;
            else
                res = true;
        } else if (*yit < *xit) {
            if (res)
                return indeterminate;
            else
                res = false;
        }
    }
    return res;
}

} //~namespace moses
