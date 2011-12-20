/** KLD.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_KLD_H
#define _OPENCOG_KLD_H

#include "dorepeat.h"

/**
 * Functions to compute the Kullback-Leibler Divergence of discrete
 * and continouous distirbution
 */

namespace opencog {

/**
 * Implementation of the algorithm described in the paper
 * Kullback-Leibler Divergence Estimation of Continuous Distributions
 * by Fernando Perez-Cruz.
 *
 * That structure allows to access to intermediary results. A function
 * returning the direct KLD is defined below for convenience.
 */
template<typename SortedSeq>
struct KLDS {
    typedef typename SortedSeq::value_type result_type;
    
    // @param p sorted sequence of values representing the distribution of P
    // @param q sorted sequence of values representing the distribution of Q
    KLDS(const SortedSeq& p_, const SortedSeq& q_) :
        p(p_), q(q_), margin(1.0), it_p(p.cbegin()), it_q(q.cbegin()),
        x_very_first(*it_p - margin), x_very_last(p.back() + margin),
        p_x_pre(x_very_first), q_x_pre(p_x_pre) {}

    // computes the components log(delta_p / delta_q) once at a
    // time. This is useful when one want to treat each component as a
    // seperate optimization of a multi-optimization problem.
    result_type next() {
        // compute delta P
        result_type p_x = *it_p, delta_p = 1.0 / (p_x - p_x_pre);
            
        // compute delta Q
        result_type q_x = it_q == q.cend()? x_very_last : *it_q;
        // search the points of q right before and after p_x
        while (q_x < p_x) {
            q_x_pre = q_x;
            ++it_q;
            q_x = it_q == q.cend()? x_very_last : *it_q;
        }
        result_type delta_q = 1.0 / (q_x - q_x_pre);

        p_x_pre = p_x;
        ++it_p;

        return std::log(delta_p / delta_q);
    }

    // @return estimate of KL(P||Q)
    result_type operator()() {
        result_type res = 0;
        size_t size = p.size();
        dorepeat(size)
            res += next();
        return res / size - 1;
    }

    const SortedSeq &p, &q;
    result_type margin;
    typename SortedSeq::const_iterator it_p, it_q;
    result_type x_very_first, x_very_last, p_x_pre, q_x_pre;
};

// function helper
template<typename SortedSeq>
typename SortedSeq::value_type KLD(const SortedSeq& p, const SortedSeq& q) {
    KLDS<SortedSeq> klds(p, q);
    return klds();
}
    
} // ~namespace opencog

#endif // _OPENCOG_KLD_H
