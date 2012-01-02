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
#include <opencog/util/Logger.h>
#include <opencog/util/Counter.h>

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
 * That structure (will) allow to evaluate KLD(P|Q) over different Q
 * for a fixed P and access to intermediary results. A function
 * returning the direct KLD is defined further below for convenience.
 */
template<typename SortedSeq>
struct KLDS {
    typedef typename SortedSeq::value_type result_type;
    
    // @param p sorted sequence of values representing the distribution of P
    // @param q sorted sequence of values representing the distribution of Q
    KLDS(const SortedSeq& p_, const SortedSeq& q_) :
        p(p_), q(q_), p_s(p.size()), q_s(q.size()),
        margin(1.0), epsilon(1e-16),
        p_pdf(Counter<result_type, result_type>(p)),
        q_pdf(Counter<result_type, result_type>(q)),
        it_p(p_pdf.cbegin()), it_q(q_pdf.cbegin()),
        x_very_first(p.front() - margin), x_very_last(p.back() + margin),
        p_x_pre(x_very_first), q_x_pre(p_x_pre) {
            // pre-compute the pdf of p and q
            // TODO
        }

    // computes the components log(delta_p / delta_q) once at a
    // time. This is useful when one want to treat each component as a
    // seperate optimization of a multi-optimization problem.
    result_type next() {
        // compute delta P
        result_type p_x = it_p->first,
            delta_p_x = std::max(epsilon, p_x - p_x_pre),
            delta_p = (q_s * it_p->second) / delta_p_x;

        // compute delta Q
        result_type q_x = it_q == q_pdf.cend()? x_very_last : it_q->first;
        // search the points of q right before and after p_x
        while (q_x < p_x) {
            q_x_pre = q_x;
            ++it_q;
            q_x = it_q == q_pdf.cend()? x_very_last : it_q->first;
        }
        result_type delta_q_x = std::max(epsilon, q_x - q_x_pre),
            n_duplicates = it_q == q_pdf.cend()? 1.0 : it_q->second,
            delta_q = (p_s * n_duplicates) / delta_q_x;
        
        p_x_pre = p_x;
        ++it_p;
        
        return std::log(delta_p / delta_q);
    }

    // @return estimate of KL(P||Q)
    result_type operator()() {
        result_type res = 0;
        dorepeat(p_pdf.size())
            res += next();
        return res / p_s - 1; // I don't fully understand the -1
    }

private:
    // map 1-dimensional inputs to their probability density
    typedef std::map<result_type, result_type> pdf_t;
    const SortedSeq &p, &q;
    const result_type p_s, q_s,            // sizes of p and q used in
                                           // the computation of LKD
        margin,
        epsilon;        // minimum distance between 2 values
                        // (to avoid getting infinite delta)
    pdf_t p_pdf, q_pdf;
    typename pdf_t::const_iterator it_p, it_q;
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
