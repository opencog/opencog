/*
 * DiffusionStat.h
 *
 * Copyright (C) 2016 Opencog Foundation
 * 
 * All Rights Reserved
 *
 * Written by Misgana Bayetta <misgana.bayetta@gmail.com>
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
         
#include <algorithm>
#include <cassert>

#include "DiffusionStat.h"

using namespace opencog;
using namespace opencog::ecan;

void update_bins(std::vector<Bin>& bins, double ivalue)
{
    for (auto& bin : bins) {
        if (bin.lb_sti > ivalue and bin.ub_sti <= ivalue) {
	    bin.increment();
            break;
        }
    }
}

int get_bin_index(const std::vector<Bin>& bins, double sti)
{
    for (std::vector<Bin>::size_type i = 0; i < bins.size(); i++) {
        if (bins[i].lb_sti <= sti and bins[i].ub_sti >= sti) return i;
    }
    
    return -1;
}

/**
 * given sorted set of importance values and quantile size K, generates
 * the K-quantile bins.
 *
 * @param ivalues - sorted set of importance values.
 * @params size   - the q value
 *
 * @returns a vector of Bin.
 */
std::vector<Bin> create_bins(std::set<double> ivalues, int size /*= 10*/)
{
    std::vector<Bin> bins;
    bins.reserve(size);
    double lbound = 0.0;
    double step_size = *std::max_element(ivalues.begin(), ivalues.end()) / size;

    for (int i = 0; i < size; i++) {
        bins.emplace_back(lbound, lbound + step_size);
        lbound += step_size;
    }

    return bins;
}

/**
 * Creates a new bin and sets each bin's frequency to a value derived from
 * the the two input vectors.
 *
 * @param past - A bin built from previous past( i.e for (T,Ts]  where T < Ts < Now)
 * @param recent - A bin built from recent past.(i.e for [Ts,Now] where T < Ts < Now)
 * @param bias - Biasing constant.
 *
 * @returns A vector of Bin.
 */
std::vector<Bin> merge_bins(std::vector<Bin>& past, std::vector<Bin>& recent,
                            double bias)
{
    assert(past.size() == recent.size());
    assert(bias <= 1);

    for (std::vector<Bin>::size_type i = 0; i < past.size(); i++) {
        auto freq = bias * past[i].freq + (1 - bias) * recent[i].freq;
        recent[i].freq = freq;
    }

    return recent;
}

