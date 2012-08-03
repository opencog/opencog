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
#define COMPOSITE_SCORE_SIZE 3

enum msg_types
{
    MSG_COMBO_TREE = 1,  // combo_tree, in ascii string form.
    MSG_COMBO_TREE_LEN,  // length of ascii string, for above.
    MSG_MAX_EVALS,       // maximum allowed evaluations
    MSG_NUM_EVALS,       // number of evaluations actually performed.
    MSG_NUM_COMBO_TREES, // number of combo trees sent
    MSG_CSCORE,          // composite score
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

/// Send a combo tree to the target node 
void moses_mpi::send_tree(const combo_tree &tr, int target)
{
    stringstream ss;
    ss << tr;
    const char * stree = ss.str().c_str();
    int stree_sz = ss.str().size();
    MPI::COMM_WORLD.Send(&stree_sz, 1, MPI::INT, target, MSG_COMBO_TREE_LEN);
    MPI::COMM_WORLD.Send(stree, strlen(stree), MPI::CHAR, target, MSG_COMBO_TREE);
}

/// Receive a combo tree from the source node.
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

/// Send composite score to target node
void moses_mpi::send_cscore(const composite_score &cs, int target)
{
    double flattened_score[COMPOSITE_SCORE_SIZE];
    flattened_score[0] = cs.get_score();
    flattened_score[1] = cs.get_complexity();
    flattened_score[2] = cs.get_complexity_penalty();
    MPI::COMM_WORLD.Send(flattened_score, COMPOSITE_SCORE_SIZE,
         MPI::DOUBLE, target, MSG_CSCORE);
}

/// Receive composite score from source node
void moses_mpi::recv_cscore(composite_score &cs, int source)
{
    double flattened_score[COMPOSITE_SCORE_SIZE];
    MPI::COMM_WORLD.Recv(flattened_score, COMPOSITE_SCORE_SIZE,
         MPI::DOUBLE, source, MSG_CSCORE);

    cs = composite_score(flattened_score[0], flattened_score[1], flattened_score[2]);
}

/// dispatch_deme -- Send an exemplar to node for deme expansion.
///
/// This method will send an exemplar to the first available (free) worker,
/// and then block, waiting for results from the worker.  Upon receiving
/// the results, it will unblock and return to the caller.  This blocking
/// behaviour is intended for use within a thread.
///
/// @max_evals is the maximum number of evaluations the worker should perform.
/// @n_evals is the actual number of evaluations performed.
//
void moses_mpi::dispatch_deme(const combo_tree &tr, int max_evals,
                              bscored_combo_tree_set& cands, int& n_evals)
{
    dispatch_thread& worker = worker_pool.borrow();
cout<<"duude got worker "<<worker.rank<<endl;
    MPI::COMM_WORLD.Send(&max_evals, 1, MPI::INT, worker.rank, MSG_MAX_EVALS);

    // Note: recv_deme will block, until remote worker has delivered
    // the results back to us.
    send_tree(tr, worker.rank);
    recv_deme(cands, n_evals, worker.rank);

    worker_pool.give_back(worker);
cout<<"duude done with worker "<<worker.rank<<endl;
}

/// recv_more_work -- indicate to worker if there is more work to be done.
///
/// This method is intended for use on worker nodes; it will block until
/// the worker receives more work, or until the worker is told to exit.
/// If the return value is zero, the worker should exit.  If the return
/// value is positive, it is the "maximum allowed evaluations" (max_evals)
/// for the next batch, which follows next.
///
/// This method should be called only once per iteration!
//
int moses_mpi::recv_more_work()
{
    int max_evals = 0;
    MPI::COMM_WORLD.Recv(&max_evals, 1, MPI::INT, ROOT_NODE, MSG_MAX_EVALS);
    return max_evals;
}

/// recv_exemplar -- used by worker to receive an exemplar
///
/// This method is intended for use only on worker nodes.
//
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
        const bscored_combo_tree& btr = *it;

        // We are going to send only the composite score, and not the
        // full behavioural score.  Basically, the full bscore is just
        // not needed for the current most popular merge technique.
        send_cscore(get_composite_score(btr), ROOT_NODE);
        send_tree(get_tree(btr), ROOT_NODE);
    }
}

/// recv_deme -- receive the deme sent by send_deme()
///
/// This will receive a deme from any source; it is safe to call this
/// routine with source=MPI_ANY_SOURCE, it will then pick up a deme,
/// in an atomic, unfragmented way, from the first source that sent
/// one to us.
void moses_mpi::recv_deme(bscored_combo_tree_set& cands, int& n_evals, int source)
{
    MPI::Status status;
    MPI::COMM_WORLD.Recv(&n_evals, 1, MPI::INT, source, MSG_NUM_EVALS, status);

    // This allows recv to be called with MPI_ANY_SOURCE, yet finish
    // the reception with the correct materials.
    source = status.Get_source();

    int num_trees = 0;
    MPI::COMM_WORLD.Recv(&num_trees, 1, MPI::INT, source, MSG_NUM_COMBO_TREES);
    for (int i=0; i<num_trees; i++) {
        composite_score sc;
        recv_cscore(sc, source);
        combo_tree tr;
        recv_tree(tr, source);

        // The vectore behavioural score will be empty; only the 
        // composite score gets a non-trivial value.
        behavioral_score bs;
        penalized_behavioral_score pbs(bs, sc.get_complexity_penalty());
        composite_behavioral_score cbs(pbs, sc);
        bscored_combo_tree bsc_tr(tr, cbs);
        cands.insert(bsc_tr);
    }
}

} // ~namespace moses
} // ~namespace opencog

#endif /* HAVE_MPI */
