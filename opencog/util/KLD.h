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

#include <boost/range/adaptor/map.hpp>
#include <boost/range/numeric.hpp>

#include "dorepeat.h"
#include "Logger.h"
#include "Counter.h"

/**
 * Functions to compute the Kullback-Leibler Divergence of discrete
 * and continouous distirbution
 */

namespace opencog {

 using boost::adaptors::map_values;

/**
 * Implementation of the algorithm described in the paper
 * Kullback-Leibler Divergence Estimation of Continuous Distributions
 * by Fernando Perez-Cruz.
 *
 * That structure (will) allow to evaluate KLD(P|Q) over different Q
 * for a fixed P and access to intermediary results. A function
 * returning the direct KLD is defined further below for convenience.
 */
template<typename FloatT>
struct KLDS {
    // map 1-dimensional inputs to their probability density
    typedef std::map<FloatT, FloatT> pdf_t;
    typedef typename pdf_t::iterator pdf_it;
    typedef typename pdf_t::const_iterator pdf_cit;

    /// set the distribution discribing Q to compute KL(P||Q)
    void set_p_pdf(const pdf_t& p_pdf_, FloatT p_s_ = -1) {
        p_pdf = p_pdf_;
        p_s = p_s_ < 0 ? boost::accumulate(p_pdf | map_values, 0) : p_s_;
        x_very_first = p_pdf.cbegin()->first - margin;
        x_very_last = p_pdf.crbegin()->first + margin;
        precompute_delta_p();
    }
    /// like above but takes a sequence of Float instead of a Counter
    template<typename SortedSeq>
    void set_p(const SortedSeq& p) {
        set_p_pdf(Counter<FloatT, FloatT>(p), p.size());
    }

    // empty constructor, must use set_p (or set_p_pdf) and set_q (or
    // set_q_pdf) to fill P and Q before computing KL(P||Q) via
    // operator()
    KLDS() : margin(1.0) {}
    
    // @param p sorted sequence of values representing the distribution of P
    //
    // Q should be specified by set_q (or set_q_pdf)
    template<typename SortedSeq>
    KLDS(const SortedSeq& p) : margin(1.0) {
        set_p(p);
    }

    
    // @param p_pdf_ mapping between value and its number of
    //               occurences sampled according to the distribution of P
    // @param p_s_   total number of observations of p_pdf_
    //
    // If p_s_ is negative then it is automatically calculated. One
    // might want to overwrite those default arguments in case one
    // already knows their size and not waste time recomputing them.
    //
    // Q should be specified by set_q (or set_p_pdf)
    KLDS(const pdf_t& p_pdf_, FloatT p_s_ = -1) : margin(1.0) {
        set_p_pdf(p_pdf_, p_s_);
    }

    // size of the pdf of p. This useful when one wants to know till
    // when next can be used
    size_t size() const {
        return p_pdf.size();
    }
    
    // computes the components log(delta_p / delta_q) once at a
    // time. This is useful when one want to treat each component as a
    // seperate optimization of a multi-optimization problem.
    FloatT next(const pdf_t& q_pdf, FloatT q_s, FloatT& q_x_pre,
                pdf_cit& cit_p, pdf_cit& cit_q) {
        OC_ASSERT(cit_p != p_pdf.end());

        // compute delta Q
        FloatT p_x = cit_p->first,
            q_x = cit_q == q_pdf.cend()? x_very_last : cit_q->first;
        // search the points of q right before and after p_x
        while (q_x < p_x) {
            q_x_pre = q_x;
            ++cit_q;
            q_x = cit_q == q_pdf.cend()? x_very_last : cit_q->first;
        }
        FloatT delta_p = cit_p->second,
            delta_q_x = q_x - q_x_pre,
            n_duplicates = cit_q == q_pdf.cend()? 1.0 : cit_q->second,
            delta_q = (p_s / q_s) * n_duplicates / delta_q_x;

        ++cit_p;
        return std::log(delta_p / delta_q);
    }
    
    // @return estimate of KL(P||Q)
    template<typename SortedSeq>
    FloatT operator()(const SortedSeq& q) {
        pdf_t q_pdf = Counter<FloatT, FloatT>(q);
        FloatT q_s = q.size(),
            q_x_pre = x_very_first,
            res = 0;
        pdf_cit cit_p = p_pdf.begin();
        pdf_cit cit_q = q_pdf.begin();
        dorepeat(p_pdf.size())
            res += next(q_pdf, q_s, q_x_pre, cit_p, cit_q);
        return res / p_s - 1; // I don't fully understand the -1
    }

    template<typename Out>
    void operator()(const pdf_t& q_pdf, Out out) {
        FloatT q_s = boost::accumulate(q_pdf | map_values, 0),
            q_x_pre = x_very_first;
        pdf_cit cit_p = p_pdf.begin(),
            cit_q = q_pdf.begin();
        dorepeat(p_pdf.size())
            *out++ = next(q_pdf, q_s, q_x_pre, cit_p, cit_q);
    }

private:
    /// replace the occurence count in p_pdf by delta_p
    void precompute_delta_p() {
        FloatT p_x_pre(x_very_first);
        foreach(typename pdf_t::value_type& v, p_pdf) {
            FloatT p_x = v.first,
                delta_p_x = p_x - p_x_pre,
                delta_p = v.second / delta_p_x;
            v.second = delta_p;
            p_x_pre = p_x;
        }
    }
    
    FloatT p_s,            // sizes of p used in
                           // the computation of LKD
        margin;

    pdf_t p_pdf;
    FloatT x_very_first, x_very_last;
};

// function helper
template<typename SortedSeq>
typename SortedSeq::value_type KLD(const SortedSeq& p, const SortedSeq& q) {
    KLDS<typename SortedSeq::value_type> klds(p);
    return klds(q);
}
    
} // ~namespace opencog

#endif // _OPENCOG_KLD_H
