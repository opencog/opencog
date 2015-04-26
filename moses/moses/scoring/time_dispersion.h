/*
 * opencog/learning/moses/scoring/time_dispersion.h
 *
 * Copyright (C) 2014 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _MOSES_TIME_DISPERSION_H
#define _MOSES_TIME_DISPERSION_H

#include "scoring_base.h"

namespace opencog { namespace moses {

using combo::CTableTime;

enum class TemporalGranularity {day, month};

/**
 * WARNING: that class does not support dynamical subsampling. That is
 * if subsampling occurs on ctable after the constructor has been
 * called then _Hmax might be correct anymore.
 */
struct bscore_ctable_time_dispersion : public bscore_ctable_base
{
    bscore_ctable_time_dispersion(const CTable& ctable,
                                  float time_dispersion_pressure = 0.0,
                                  float time_dispersion_exponent = 1.0,
                                  TemporalGranularity granularity =
                                  TemporalGranularity::day,
                                  unsigned multiplier = 1);

protected:
    /**
     * Dispersion granularity of the time dispersion (follows actual
     * calendar for month)
     */
    TemporalGranularity _granularity;

    /**
     * Temporal granularity multipler. So for instance if
     * multipler = 2 and granulatiy = month
     * the actually granularity will be of 2 months
     *
     * TODO: multiplier other than 1 are not supported at the moment
     */
    unsigned _multiplier;

    float _pressure,            // See comment of
        _exponent,              // get_time_dispersion_penalty
        _Hmax;                  // below

    /**
     * Return the timestamp class of a given timestamp.
     *
     * For instance if the granularity is monthly:
     *
     * 2014-Mar-10 and 2014-Mar-20 will return the same timestamp
     * class, 2014-Mar-01. As you may notice the first day of the
     * class is used to "pad" the date.
     *
     * But 2013-Feb-01 and 2012-Feb-24 will return 2 different
     * classes, 2013-Feb-01 and 2012-Feb-01 respectively, because the
     * years are different.
     */
    TTable::value_type get_timestamp_class(const TTable::value_type& timestamp) const;
    
    /**
     * Return a time dispersion penalty (weighted by
     * time_dispersion_pressure). Note the penalty is positive (that
     * is it's gonna be subtracted from the score at the end).
     *
     * Specifically the penalty is defined as
     *
     * pressure * (1 - H(time|active) / Hmax)^exponent
     *
     * Where H(time|active) is the entropy of the time distribution
     * conditioned on active observations, where the probability of a timestamp:
     *
     *               number of active observations at T
     * P(T|active) = ----------------------------------
     *               total number of active observations
     *
     * Hmax is the maximum entropy, that is the entropy of the uniform
     * distribution over timestamps.
     */
    score_t get_time_dispersion_penalty(const CTableTime& ctt) const;
};

} //~namespace moses
} //~namespace opencog

#endif
