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
 * @param p sorted sequence of values representing the distribution of P
 * @param q sorted sequence of values representing the distribution of Q
 *
 * @return estimate of KL(P||Q)
 */
template<typename SortedSeq>
double KLD(const SortedSeq& p, const SortedSeq& q)
{
    typedef typename SortedSeq::value_type FloatT;
    
    static const FloatT margin = 1.0; // distance of the point from the min
                                      // and max points of *p.begin() and
                                      // *p.last()
    
    FloatT res = 0.0;           // final result
    
    auto it_p = p.cbegin(),
        it_q = q.cbegin();

    FloatT x_very_first = *it_p - margin,
        x_very_last = p.back() + margin,
        p_x_pre = x_very_first,
        q_x_pre = p_x_pre;
    for (; it_p != p.cend(); ++it_p) {
        // compute delta P
        FloatT p_x = *it_p, delta_p = 1.0 / (p_x - p_x_pre);
        
        // compute delta Q
        FloatT q_x = *it_q;
        // search the points of q right before and after p_x
        while (q_x < p_x) {
            q_x_pre = q_x;
            ++it_q;
            q_x = it_q == q.cend()? x_very_last : *it_q;
        }
        FloatT delta_q = 1.0 / (q_x - q_x_pre);

        // accumulate result and update previous value
        res += std::log(delta_p / delta_q);
        p_x_pre = p_x;
    }
    return res / p.size();
}

} // ~namespace opencog

#endif // _OPENCOG_KLD_H
