/*
 * opencog/learning/moses/scoring/boosting_ascore.h
 *
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Linas Vepstas
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
#ifndef _MOSES_BOOST_ASCORE_H
#define _MOSES_BOOST_ASCORE_H

#include <vector>

#include "scoring_base.h"

namespace opencog { namespace moses {

/**
 * Score calculated from the behavioral score, using boosting.
 *
 * The score is calculated as a weighted sum of the bscore over all samples:
 *      score = sum_x weight(x) * BScore(x)
 *
 * Each element in the bscore typically corresponds to a sample in a
 * supervised training set, that is, a row of a table contianing the
 * training data. The weight is an adjustment, computed by applying a
 * standard AdaBoost-style algorithm.  See, for example
 * http://en.wikipedia.org/wiki/AdaBoost
 */
class boosting_ascore : public ascore_base
{
public:
    boosting_ascore(size_t);

    score_t operator()(const behavioral_score&) const;

private:
    std::vector<double> _weights;
    size_t _size;
};


} //~namespace moses
} //~namespace opencog

#endif // _MOSES_BOOST_ASCORE_H
