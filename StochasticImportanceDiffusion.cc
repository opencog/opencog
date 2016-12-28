/*
 * opencog/attentionbank/StochasticImportanceDiffusion.h
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
#include <math.h>

#include <opencog/attentionbank/AttentionBank.h>
#include "StochasticImportanceDiffusion.h"

using namespace opencog;
using namespace opencog::ecan;

unsigned int StochasticDiffusionAmountCalculator::bin_index(const Handle& h)
{
    return attentionbank()._importanceIndex.
                     importanceBin(h->getAttentionValue()->getSTI());
}

size_t StochasticDiffusionAmountCalculator::bin_size(unsigned int index)
{
   return attentionbank()._importanceIndex.size(index);
}

void StochasticDiffusionAmountCalculator::update_bin(const Handle& h)
{
    unsigned short index = bin_index(h);
    auto it = std::find_if(_bins.begin(),_bins.end(),
            [=](const DiffusionRecordBin& bin){ return (bin.index == index); });

    if (it == _bins.end())
    {
        _bins.push_back(DiffusionRecordBin());
        it = _bins.begin() + _bins.size()-1;
    }
    DiffusionRecordBin& bin = *it;

    bin.count += 1;
    seconds sec = duration_cast<seconds>(
            high_resolution_clock::now() - bin.last_update);
    bin.update_rate = bin.count/sec.count();

    bin.last_update = high_resolution_clock::now();
    bin.size = bin_size(index);
}

StochasticDiffusionAmountCalculator::StochasticDiffusionAmountCalculator(void)
{
}

StochasticDiffusionAmountCalculator::~StochasticDiffusionAmountCalculator()
{
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
std::vector<DiffusionRecordBin> StochasticDiffusionAmountCalculator::
            merge_bins(const std::vector<DiffusionRecordBin>& past,
            std::vector<DiffusionRecordBin>& recent, float bias)
{
    for (DiffusionRecordBin& bin : recent ) {
        auto it = std::find_if(past.begin(),past.end(),
                [=](const DiffusionRecordBin& b){ return (bin.index == b.index); });
        if (it == past.end() ) continue;
        const DiffusionRecordBin& b = *it;
        auto count = bias * b.count + (1 - bias) * bin.count;
        bin.count = count;
    }

    return recent;
}

float StochasticDiffusionAmountCalculator::diffusion_amount(const Handle& h,
                                                            float decay_rate)
{
    float average_elapsed_time = 0.0f;

    auto index = bin_index(h);
    auto it = std::find_if(_bins.begin(), _bins.end(),
            [index](const DiffusionRecordBin& b){ return (b.index == index); });
    if (it != _bins.end() )   average_elapsed_time = (*it).size / (*it).update_rate;   

    update_bin(h); // Update DiffusionRecordBin.

    return h->getAttentionValue()->getSTI() * pow((1 - decay_rate), average_elapsed_time);                
}


