/** oc_omp.h --- 
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


#ifndef _OPENCOG_OC_OMP_H
#define _OPENCOG_OC_OMP_H

/**
 * file containing definitions to ease multi-threading support. The
 * eaisest way I found to is to use paralllel versions of STL
 * algorithms. To use them include that file and put the namespace
 * OMP_ALGO in front of the algorithm, for instance
 * OMP_ALGO::tranform(...). That way a simple flag, OC_OMP can enable
 * or disable them (convenient to compile with the compiler that
 * doesn't implement OMP such as clang)
 */

// compile with multithread support
#define OC_OMP

#ifdef OC_OMP
#include <omp.h>
#include <parallel/algorithm>
#define OMP_ALGO __gnu_parallel
#else
#define OMP_ALGO std
#endif

namespace opencog {

// setting the parallel env, such as number of threads, number of
// minimal iterations to parallelize
void setting_omp(unsigned num_threads);

} // ~namespace opencog

#endif // _OPENCOG_OC_OMP_H
