/* oc_omp.h --- 
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

/** \addtogroup grp_cogutil
 *  @{
 */

/** @name Multi-threading support
 *
 * The easiest way I found to do this is to use the parallel versions
 * of the STL algorithms.  To use them, include this file and put the
 * namespace OMP_ALGO in front of the algorithm.  For example,
 * OMP_ALGO::for_each(...) provides the parallel version of
 * std::for_each().  
 *
 * If the cpp define OC_OMP is defined, the parallel versions of the
 * std algorithms are used.   Disabling this define allows code to be
 * compiled with compilers that do not (yet) implement OMP, such as 
 * LLVM clang.
 */
///@{

//! compile with multithread support (!)
#define OC_OMP

#ifdef OC_OMP
#include <omp.h>
#include <parallel/algorithm>
#define OMP_ALGO __gnu_parallel
#else
#define OMP_ALGO std
#endif

namespace opencog {

//! setting the parallel env, such as number of threads, number of
//! minimal iterations to parallelize
void setting_omp(unsigned num_threads, unsigned min_n = 50);

//! returns the number of threads as configured by setting_omp
unsigned num_threads();

//! split the number of jobs in 2. For instance if n_jobs is 3, then it
//! returns <1, 2>. 
/// This function is convenient for parallalizing
/// recursive functions
std::pair<unsigned, unsigned> split_jobs(unsigned n_jobs);

} // ~namespace opencog

///@}
/** @}*/

#endif // _OPENCOG_OC_OMP_H
