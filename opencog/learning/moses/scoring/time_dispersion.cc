/*
 * opencog/learning/moses/scoring/time_dispersion.cc
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

#include "time_dispersion.h"

#include <opencog/comboreduct/table/table_io.h>
#include <opencog/util/numeric.h>

namespace opencog { namespace moses {

using namespace combo;

bscore_ctable_time_dispersion::bscore_ctable_time_dispersion(const CTable& _ctable,
                                                             float time_dispersion_pressure,
                                                             float time_dispersion_exponent,
                                                             TemporalGranularity granularity,
                                                             unsigned multiplier)
    : bscore_ctable_base(_ctable),
      _granularity(granularity), _multiplier(multiplier),
      _pressure(time_dispersion_pressure), _exponent(time_dispersion_exponent)
{
    // TODO multipler other than 1 is not supported yet
    OC_ASSERT(_multiplier == 1, "Multiplier other than 1 is not supported yet");

    // Set of timestamp classes
    std::set<TTable::value_type> timestamp_classes;
    for (const auto& iorow : _ctable)
        for (const TimedCounter::value_type& tcv : iorow.second)
            timestamp_classes.insert(get_timestamp_class(tcv.first.timestamp));

    // Compute Hmax
    _Hmax = log2(timestamp_classes.size());
}

TTable::value_type bscore_ctable_time_dispersion::get_timestamp_class(const TTable::value_type& timestamp) const
{
    switch(_granularity) {
    case TemporalGranularity::day:
        return timestamp;
    case TemporalGranularity::month:
        OC_ASSERT(!timestamp.is_not_a_date(),
                  "timestamp is not a valid date, "
                  "are you sure you have loaded a timestamp feature "
                  "(option --timestamp-feature)?")
        return TTable::value_type(timestamp.year(), timestamp.month(), 1);
    default: {
        std::stringstream ss;
        ss << "Case " << static_cast<size_t>(_granularity) << " not implemented";
        OC_ASSERT(false, ss.str());
        return TTable::value_type();
    }
    }
}

float bscore_ctable_time_dispersion::get_time_dispersion_penalty(const CTableTime& ctt) const
{
    // Log the ctabletime
    if (logger().isFineEnabled())
        ostreamCTableTime(logger().fine() << "bscore_ctable_time_dispersion:"
                          ":get_time_dispersion_penalty ctabletime\n", ctt);

    // Define time distribution conditional by activation
    std::vector<float> act_time_dist;
    unsigned total = 0;
    for (const auto& tio : ctt) {
        unsigned row_count = tio.second.total_count();
        act_time_dist.push_back(tio.second.get(id::logical_true));
        total += row_count;
    }

    // Normalize so the distribution sums up to 1
    for (float& prob : act_time_dist) prob /= total;
    
    // Compute its entropy
    float H = entropy(act_time_dist);

    // Compute
    // 1. the normalized entropy conditioned on activation
    // 2. its p-th power
    // 3. the final penalty
    float normalized_H = H / _Hmax,
        penalty = 1.0f - normalized_H,
        distorted_penalty = std::pow(penalty, _exponent),
        scaled_distorted_penalty = _pressure * distorted_penalty;

    // Log entropies and penalty
    if (logger().isFineEnabled())
        logger().fine() << "H = " << H
                        << ", time_dispersion_penalty = "
                        << scaled_distorted_penalty;

    // Return penalty
    return scaled_distorted_penalty;
}

} //~namespace moses
} //~namespace opencog
