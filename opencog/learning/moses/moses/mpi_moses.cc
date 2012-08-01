/* 
 * mpi_moses.cc --- 
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

#ifdef HAVE_MPI
#include <mpi.h>
#include <opencog/util/Logger.h>
#include "mpi_moses.h"

namespace opencog { namespace moses {

dispatch_thread::dispatch_thread()
{
cout<<"woot!"<<endl;
}

moses_mpi::moses_mpi()
{
    // MPI::Init(argc, argv);
    MPI::Init();
    
    int num_procs = MPI::COMM_WORLD.Get_size();
    int rank = MPI::COMM_WORLD.Get_rank();

    int namelen;
    char processor_name[MPI_MAX_PROCESSOR_NAME];
    MPI_Get_processor_name(processor_name, &namelen);

    logger().info() << "MPI initialized on host \"" << processor_name
        << "\" rank=" << rank << " out of num_procs=" << num_procs;

    // MPI::COMM_WORLD.Bcast(&answer, 1, MPI::INT, 0);

    // Only the root process maintains a pool.
    if (0 == rank) {
        workers.resize(num_procs);
        for (int i=0; i<num_procs; i++) {
            dispatch_thread& worker = workers[i];
            worker.rank = i;
            worker_pool.give_back(worker);
        }
    }
}

moses_mpi::~moses_mpi()
{
    MPI::Finalize();
}

bool moses_mpi::is_mpi_master()
{
    return (0 == MPI::COMM_WORLD.Get_rank());
}

void moses_mpi::dispatch_deme(const combo_tree &tr)
{
    dispatch_thread& worker = worker_pool.borrow();
cout<<"duude got worker "<<worker.rank<<endl;
}
void moses_mpi::do_work()
{
cout <<"duude done doing work"<<endl;
}

} // ~namespace moses
} // ~namespace opencog

#endif /* HAVE_MPI */
