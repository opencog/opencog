/** oc_omp.cc --- 
 *
 * Copyright (C) 2011 Nil Geisweiller
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

#include "oc_omp.h"

namespace opencog {

void setting_omp(unsigned num_threads, unsigned min_n) {
#ifdef OC_OMP
    omp_set_dynamic(false);
    omp_set_num_threads(num_threads);
    __gnu_parallel::_Settings gps;
    gps.transform_minimal_n = min_n;
    gps.for_each_minimal_n = min_n;
    gps.replace_minimal_n = min_n;
    __gnu_parallel::_Settings::set(gps);
#endif
}

unsigned num_threads() {
#ifdef OC_OMP
    return omp_get_max_threads();
#else
    return 1;
#endif
}

std::pair<unsigned, unsigned> split_jobs(unsigned n_jobs) {
    unsigned n_jobs1 = n_jobs / 2;
    unsigned n_jobs2 = std::max(1U, n_jobs - n_jobs1);
    return {n_jobs1, n_jobs2};
}

}
