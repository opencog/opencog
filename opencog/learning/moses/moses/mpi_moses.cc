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

#define ROOT_NODE 0

enum msg_types
{
    MSG_COMBO_TREE = 1,
    MSG_COMBO_TREE_LEN,
    MSG_MAX_EVALS,
    MSG_NUM_EVALS,
    MSG_NUM_COMBO_TREES,
};

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
    if (ROOT_NODE == rank) {
        workers.resize(num_procs);
        // The root process itself is not in the pool.
        // i.e. pool starts at 1 not at 0.
        for (int i=1; i<num_procs; i++) {
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
    return (ROOT_NODE == MPI::COMM_WORLD.Get_rank());
}

/// Send an exemplar from the root node to a worker node.
//
void moses_mpi::send_tree(const combo_tree &tr, int target)
{
    stringstream ss;
    ss << tr;
    const char * stree = ss.str().c_str();
    int stree_sz = ss.str().size();
    MPI::COMM_WORLD.Send(&stree_sz, 1, MPI::INT, target, MSG_COMBO_TREE_LEN);
    MPI::COMM_WORLD.Send(stree, strlen(stree), MPI::CHAR, target, MSG_COMBO_TREE);
}

void moses_mpi::recv_tree(combo_tree &tr, int source)
{
    int stree_sz = 0;
    MPI::COMM_WORLD.Recv(&stree_sz, 1, MPI::INT, source, MSG_COMBO_TREE_LEN);

    char stree[stree_sz+1];
    MPI::COMM_WORLD.Recv(stree, stree_sz, MPI::CHAR, source, MSG_COMBO_TREE);
    stree[stree_sz] = 0;
    stringstream ss;
    ss << stree;
    ss >> tr;
}

/// Send an exemplar from the root node to a worker node.
//
void moses_mpi::dispatch_deme(const combo_tree &tr, int max_evals)
{
    dispatch_thread& worker = worker_pool.borrow();
cout<<"duude got worker "<<worker.rank<<endl;
    MPI::COMM_WORLD.Send(&max_evals, 1, MPI::INT, worker.rank, MSG_MAX_EVALS);

    send_tree(tr, worker.rank);
}

/// Return true if there is more work pending in the recevie buffers.
/// This method should be called only once per iteration!
//
int moses_mpi::recv_more_work()
{
    int max_evals = 0;
    MPI::COMM_WORLD.Recv(&max_evals, 1, MPI::INT, ROOT_NODE, MSG_MAX_EVALS);
    return max_evals;
}

void moses_mpi::recv_exemplar(combo_tree& exemplar)
{
    recv_tree(exemplar, ROOT_NODE);
}

/// send_deme -- send the completed deme from the worker back to root
///
/// This sends a pretty big glob.
// XXX TODO -- trim the deme down, before sending, by using the worst acceptable score.
void moses_mpi::send_deme(const bscored_combo_tree_set& mp, int n_evals)
{
cout << "duude id="<< MPI::COMM_WORLD.Get_rank() << "returnin a deme after evals="<<n_evals<<endl;
    MPI::COMM_WORLD.Send(&n_evals, 1, MPI::INT, ROOT_NODE, MSG_NUM_EVALS);

    int num_trees = mp.size();
    MPI::COMM_WORLD.Send(&num_trees, 1, MPI::INT, ROOT_NODE, MSG_NUM_COMBO_TREES);

    bscored_combo_tree_set::const_iterator it;
    for (it = mp.begin(); it != mp.end(); it++) {
        const combo_tree& tr = *it;
        send_tree(tr, ROOT_NODE);
    }
}

/// recv_deme -- receive the deme sent by send_deme()
///
/// This will receive a deme from any source.
void moses_mpi::recv_deme(bscored_combo_tree_set& cands, int& n_evals)
{
    MPI::Status status;
    MPI::COMM_WORLD.Recv(&n_evals, 1, MPI::INT, MPI_ANY_SOURCE, MSG_NUM_EVALS, status);

    int source = status.Get_source();

cout<<"duuude ahh reved evals="<<n_evals<<" drom src="<<source<<endl;
    int num_trees = 0;
    MPI::COMM_WORLD.Recv(&num_trees, 1, MPI::INT, source, MSG_NUM_COMBO_TREES);
    for (int i=0; i<num_trees; i++) {
        combo_tree tr;
        recv_tree(tr, source);
        // cands.insert(tr);
    }
cout<<"duuude ahh reved trees="<<num_trees<<endl;
}

} // ~namespace moses
} // ~namespace opencog

#endif /* HAVE_MPI */
