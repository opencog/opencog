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
#ifndef _OPENCOG_STOCHASTIC_DIFFUSION_H
#define _OPENCOG_STOCHASTIC_DIFFUSION_H

#include <algorithm>
#include <chrono>
#include <vector>

using namespace std::chrono;
namespace opencog
{
    class Handle;
    class AtomSpace;
    class AttentionBank;
    namespace ecan
    {
        struct DiffusionRecordBin {
            unsigned int count = 0; // Number of atoms diffused.
            unsigned int index = 0; // The index of this Bin in ImportanceIndex vec.
            unsigned int size = 0;  // The total size of this Bin in ImportanceIndex vec.
            float update_rate = 0;  // Estimated update rate for this bin.

            // Latest time of diffusion event in this bin.
            time_point<high_resolution_clock> last_update = high_resolution_clock::now(); 
        };

        /**
         * Diffusion accross the whole atomspace in one time step is expensive.
         * So we randomly choose atom and diffuse their STI values. However, this
         * method requires keeping records of each atom's last diffusion time which
         * again will be computationaly expensive for large atomspace. To overcome
         * this problem, we use a stochastic estimation of the average elpased time by
         * grouping atoms in to fixed number of bins and recording and updating the last
         * diffusion event time related to an arbitrary atom belonging to a particular
         * bin and a count of how many atoms have been updated so far in that
         * particular bin. Then the average elapsed time since last diffusion event
         * for an atom will be claculated as total in the bin divided by update rate
         * ( count of diffused atoms divided by duration of time).
         */
        class StochasticDiffusionAmountCalculator
        {
            AtomSpace* _as;
            AttentionBank* _ab;
            std::vector<DiffusionRecordBin> _bins; 

            unsigned int bin_index(const Handle& h);
            size_t bin_size(unsigned int index);
            void update_bin(const Handle& h);

            public:
            StochasticDiffusionAmountCalculator(AtomSpace * as);
            ~StochasticDiffusionAmountCalculator();

            std::vector<DiffusionRecordBin> merge_bins(const std::vector<DiffusionRecordBin>& past,
                    std::vector<DiffusionRecordBin>& recent, float bias);
            float diffused_value(const Handle& h, float decay_rate);
            float elapsed_time(const Handle& h);
        };
    }
}

#endif // _OPENCOG_STOCHASTIC_DIFFUSION_H
