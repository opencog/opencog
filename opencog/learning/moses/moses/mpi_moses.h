/*
 * mpi_moses.h ---
 *
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Author: Linas Vepstas
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


#ifndef _OPENCOG_MPI_MOSES_H
#define _OPENCOG_MPI_MOSES_H

#include <atomic>
#include <future>
#include <opencog/util/pool.h>

#include "../metapopulation/metapopulation.h"
#include "moses_params.h"

namespace opencog { namespace moses {

#ifdef HAVE_MPI

// MPI wrapper class.  The goal of this class is to hide all of the 
// MPI communications details into a distinct .cc file, so that none
// of the rest of the moses code is polluted with MPI header files.
// This class is effectively stateless: viz, there is no real reason
// for having a class here, other than to put all the routines in 
// a common place.
//
// It might be agood idea to turn this into a virtual base class, and
// have the distributed_moses code use the same API.  That way, we'd
// have less dispatcher code to maintain.  But this will require some
// fair amount of work and testing, and there's no pressing need to do
// this just right now. XXX TODO: do this, someday.
//
class moses_mpi_comm
{
public:
    moses_mpi_comm();
    ~moses_mpi_comm();
    
    bool is_mpi_root();
    int num_workers();
    
    // root methods, to be used only by root node.
    void dispatch_deme(int target, const combo_tree&, int max_evals);
    int probe_for_deme();
    void recv_deme(int source, scored_combo_tree_set&, int& n_evals,
                   const demeID_t& demeID);
    void send_finished(int target);

    // worker methods, to be used only by workers.
    int recv_more_work();
    void recv_exemplar(combo_tree&);
    void send_deme(const metapopulation&, int);

    std::atomic<size_t> sent_bytes;
    std::atomic<size_t> recv_bytes;

protected:
    void send_tree(const combo_tree&, int target);
    void recv_tree(combo_tree&, int source);

    void send_cscore(const composite_score&, int target);
    void recv_cscore(composite_score&, int source);
};

/// mpi_moses_worker -- main loop for the worker node.
///
/// Listens for exemplars, expands them, sends back the results.
/// This is mono-threaded: i.e. both the send and receive happen
/// within the main thread.
//
void mpi_moses_worker(metapopulation& mp,
                      moses_mpi_comm& mompi);

void mpi_moses(metapopulation& mp,
               const moses_parameters& pa,
               moses_statistics& stats);


#else // HAVE_MPI

static inline void mpi_moses(metapopulation& mp,
               const moses_parameters& pa,
               moses_statistics& stats)
{
    OC_ASSERT(0, "There is no MPI support in this version of moses");
};

#endif  // HAVE_MPI

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MPI_MOSES_H
