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
#include <chrono>
#include <iostream>
#include <set>
#include <vector>

/**
 * Given we want diffusion to be stochastically biased towards atoms with higher sti
 * for atoms outside of the attentional focus, with a predefined decay/sec parameter and
 * diffusion agent is an OS threads which its running time is managed by the OS 
 * process scheduler, how do we adjust the compound decay exponent for an atom 
 * selected? i.e for a decay function sti_new = sti_old - decay_rate^Ksec
 * how do we determine the elapsed seconds since last update Ksec.
 * 
 * One of the approach will be to keep record of each atoms updated and their timestamp
 * to compute the elapsed time. However, this seems a computationally expensive 
 * algorithm. The other approach we thought and implemented is, probabilistic 
 * estimation of the elapsed time by creating and updating the K-bins from the sti
 * distribution and each bin holding frequency of sti update.
 * 
 * Whenever an atoms is selected for an update, its bin will be determined from 
 * the K-bins and update count for the bin and update time will be recorded. Then
 * the average of the recorded times are used as last update time to calculate the 
 * elapsed time with an estimation bias factor.
 * 
 */
namespace opencog {
    namespace ecan {
        using hr_clock = std::chrono::high_resolution_clock;
        using chrono_d = std::chrono::duration<double>;

        struct Bin {
            // int atom_counts; // Total number of atoms falling in this bin. 
            std::vector<chrono_d> sti_update_times;

            int count = 0;
            std::chrono::time_point<hr_clock> last_update;
            double freq = 0;

            // upper and lower bounds of the bin
            double lb_sti, ub_sti;

            Bin(double lb, double ub) {
                lb_sti = lb;
                ub_sti = ub;
                last_update = hr_clock::now(); //seconds ago.	
            }

            void increment() {
                count += 1;
                auto current = hr_clock::now();
                chrono_d diff = current - last_update;
                sti_update_times.push_back(diff);
                last_update = current;

                double average_time = std::accumulate(sti_update_times.begin(),
                        sti_update_times.end(), chrono_d(0)).count()
                        / sti_update_times.size();

                freq = 1 / average_time;
            }

        };

        void update_bins(std::vector<Bin>& bins, double ivalue);

        int get_bin(const std::vector<Bin>& bins, double sti);

        std::vector<Bin> create_bins(std::set<double> ivalues, int size = 10);

        std::vector<Bin> merge_bins(std::vector<Bin>& past, std::vector<Bin>& recent, double bias);
    }
}

