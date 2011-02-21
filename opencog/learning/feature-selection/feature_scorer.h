/** feature_scorer.h --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
 *
 * Author: Nil Geisweiller <nilg@laptop>
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


#ifndef _OPENCOG_FEATURE_SCORER_H
#define _OPENCOG_FEATURE_SCORER_H

#include <functional>

#include <opencog/util/foreach.h>

namespace opencog {

/**
 * Computes the conditional entropy
 *
 * H(Y|X1, ..., Xn) = H(Y, X1, ..., Xn) - H(X1, ..., Xn)
 *
 * of the output Y given a set of features X1, ..., Xn. IT (a matrix)
 * is the type of the input table, OT (a vector) is the type of the
 * output table. It returns 1 - H(Y|X1, ..., Xn) to meet the
 * assumption that feature scorers return a value in [0, 1], where 0
 * is the lower score and 1 is the higher score.
 *
 * Note that in the literature most use mutual information instead of
 * conditional entropy but it actually amounts to the same, as
 * explained:
 *
 * H(X;Y) = H(X,Y)-H(X|Y) - H(Y|X)
 *        = H(X,Y)-H(X,Y) + H(Y) - H(X,Y) + H(X)
 *        = H(Y) + H(X) - H(X,Y)
 *
 * H(Y|X) = H(Y,X) - H(X)
 *
 * But H(Y) is constant, so maximizing H(X;Y) is equivalent to
 * minimizing H(Y|X).
 */
template<typename IT, typename OT, typename FeatureSet>
struct ConditionalEntropy : public std::unary_function<FeatureSet, double> {

    ConditionalEntropy(const IT& it, const OT& ot) 
        : _it(it), _ot(ot) {}

    double operator()(const FeatureSet& features) const {
        typedef typename IT::value_type::value_type EntryT;
        // the following mapping is used to keep track of the number
        // of inputs a given setting. For instance X1=false, X2=true,
        // X3=true is one possible setting. It is then used to compute
        // H(Y, X1, ..., Xn) and H(X1, ..., Xn)
        typedef std::map<std::vector<EntryT>, unsigned int> TupleCount;
        TupleCount ic, // for H(X1, ..., Xn)
            ioc; // for H(Y, X1, ..., Xn)
        double total = _ot.size();
        typename IT::const_iterator i_it = _it.begin();
        typename OT::const_iterator o_it = _ot.begin();
        for(; i_it != _it.end(); i_it++, o_it++) {
            std::vector<EntryT> ic_vec;
            foreach(const typename FeatureSet::value_type& idx, features)
                ic_vec.push_back((*i_it)[idx]);
            ic[ic_vec]++;
            std::vector<EntryT> ioc_vec(ic_vec);
            ioc_vec.push_back(*o_it);
            ioc[ioc_vec]++;
        }
        // Compute conditional entropy
        std::vector<double> ip, iop;
        foreach(const typename TupleCount::value_type& vic, ic)
            ip.push_back(vic.second/total);
        foreach(const typename TupleCount::value_type& vioc, ioc)
            iop.push_back(vioc.second/total);
        // compute H(Y|X1, ..., Xn)
        double cond_entropy =
            entropy(iop.begin(), iop.end()) - entropy(ip.begin(), ip.end());
        return 1 - cond_entropy;
    }

protected:
    const IT& _it;
    const OT& _ot;
};

} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SCORER_H
