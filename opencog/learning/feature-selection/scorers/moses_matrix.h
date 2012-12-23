/** scorers/moses_matrix.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_FS_SCORERS_MATRIX_H
#define _OPENCOG_FS_SCORERS_MATRIX_H

#include <opencog/util/numeric.h>

#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/comboreduct/combo/common_def.h>

namespace opencog {

using namespace moses;
using namespace combo;

/**
 * Wrapper to use moses scoring precision (see
 * opencog/learning/moses/moses/scoring.h).  This is one of the
 * confusion-matrix based scorers.
 *
 * That wrapper uses the method best_possible_score() given a certain
 * feature set. And therefore attempts to maximize the best possible
 * score one would get (w.r.t. some fitness function) given the
 * feature set being evaluated.
 */
template<typename FeatureSet>
struct pre_scorer : public unary_function<FeatureSet, double>
{
    pre_scorer(const CTable& ctable,
               float penalty = 1.0f,
               float min_activation = 0.5f,
               float max_activation = 1.0f,
               bool positive = true)
        : _ctable(ctable), _penalty(penalty),
          _min_activation(min_activation), _max_activation(max_activation),
          _positive(positive) {}

    double operator()(const FeatureSet& fs) const {
        // filter the ctable
        CTable filtered_ctable = _ctable.filtered(fs);
        // create the scorer
        precision_bscore sc(filtered_ctable, _penalty,
                            _min_activation, _max_activation, _positive);
        return boost::accumulate(sc.best_possible_bscore(), 0.0);
    }
protected:
    const CTable& _ctable;
    float _penalty, _min_activation, _max_activation;
    bool _positive;
};


} // ~namespace opencog

#endif // _OPENCOG_FS_SCORERS_MATRIX_H
