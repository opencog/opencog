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

#include "metapopulation.h"
#include "moses_params.h"

namespace opencog { namespace moses {

#ifdef HAVE_MPI

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
        void recv_deme(int source, bscored_combo_tree_set&, int& n_evals);
        void send_finished(int target);

        // worker methods, to be used only by workers.
        int recv_more_work();
        void recv_exemplar(combo_tree&);
        void send_deme(const bscored_combo_tree_set&, int);

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
template<typename Scoring, typename BScoring, typename Optimization>
void mpi_moses_worker(metapopulation<Scoring, BScoring, Optimization>& mp,
                      moses_mpi_comm& mompi)
{
    // Worker processes loop until done, then return.
    // Each worker waits for an exemplar, expands it, then returns
    // the results.
    while(1) {
        int max_evals = mompi.recv_more_work();
        if (0 >= max_evals)
            return;

        combo_tree exemplar;
        mompi.recv_exemplar(exemplar);
        if (!mp._dex.create_deme(exemplar)) {
            // XXX replace this with appropriate message back to root!
            OC_ASSERT(false, "Exemplar failed to expand!\n");
        }
        size_t evals_this_deme = mp._dex.optimize_deme(max_evals);

        mp.merge_deme(mp._dex._deme, mp._dex._rep, evals_this_deme);
        mp._dex.free_deme();
        mompi.send_deme(mp, evals_this_deme);

        // Clear the metapop -- start with a new slate each time.
        mp.clear();
    }
}

// #define HAVE_MULTI_THREAD_MPI 1
#ifdef HAVE_MULTI_THREAD_MPI

struct worker_node
{
   worker_node() : rank(-1) {}
   int rank;
};

struct worker_pool :
    public pool<worker_node>
{
    worker_pool(int num);
private:
    std::vector<worker_node> workers;
};

/// mpi_expander_threaded -- multi-threaded variant of a deme expander.
///
/// Expands one exemplar into a deme.  Exemplar is dispatched to an
/// available MPI node.  Dispatch and collection of results is done
/// in a distinct thread; thus, this class won't work/is unusable if
/// the MPI implementation doesn't support MPI_THREAD_MULTIPLE mode.
///
/// This is a "wrapper class"; it doesn't do much; the only reason this
/// class exists is to make thread creation simpler -- threads require
/// a function as an argument, and this class provides the needed
/// operator() to make this work.
///
/// This class has been lightly tested, and seems to work. It has not
/// been heavily teted, because I don't have a multi-node machine with
/// an MPI implementation that supports threads.
//
template<typename Scoring, typename BScoring, typename Optimization>
struct mpi_expander_threaded
{
private:
     worker_node _w;
     combo_tree _t;

     worker_node& _worker;
     combo_tree& _extree;

     moses_mpi_comm& _mompi;
     worker_pool& _pool;
     metapopulation<Scoring, BScoring, Optimization>& _mp;
     int _max_evals;
     moses_statistics& _stats;
     std::atomic<int>& _thread_count;

public:
    // Cache of arguments we will need.
    mpi_expander_threaded(moses_mpi_comm& mompi,
                 worker_pool& wpool,
                 metapopulation<Scoring, BScoring, Optimization>& mp,
                 int max_evals, moses_statistics& stats,
                 std::atomic<int>& tcount) :
        _worker(_w), _extree(_t),
        _mompi(mompi), _pool(wpool),
        _mp(mp), _max_evals(max_evals), _stats(stats),
        _thread_count(tcount)
    {}

    /// Select an exemplar out of the metapopulation. Cache it.
    /// Try to get an unoccupied node.  This method will block if all
    /// the nodes are busy.  When if finally unblocks, caller can
    /// start sending stuff to the worker.
    void pick_exemplar()
    {
        bscored_combo_tree_set::const_iterator exemplar = _mp.select_exemplar();
        if (exemplar == _mp.end()) {
            // XXX fixme we should block and wait, ...!  But how ..!?
            logger().warn("There are no more exemplars in the metapopulation "
                          "that have not been visited and yet a solution was "
                          "not found.  Perhaps reduction is too strict?");
            // If there are no more exemplars in our pool, we will
            // have to wait for some more to come rolling in.
            return;
        }
        _extree = get_tree(*exemplar);

        // borrow will block, if there are no workers free.
        _worker = _pool.borrow();
    }

    // Function that will run inside a thread
    // self-deletes upon exit of the thread.
    bool operator()()
    {
        _thread_count++;

        _mompi.dispatch_deme(_worker.rank, _extree, _max_evals);

        int n_evals = 0;
        bscored_combo_tree_set candidates;
        _mompi.recv_deme(_worker.rank, candidates, n_evals);
cout<<"duuude master "<<getpid() <<" from="<<_worker.rank << " got evals="<<n_evals <<" got cands="<<candidates.size()<<endl;
        _pool.give_back(_worker);

        _stats.n_expansions ++;
        _stats.n_evals += n_evals;

        _mp.update_best_candidates(candidates);
        _mp.merge_candidates(candidates);

        _thread_count--;
        return false;
    }
};

template<typename Scoring, typename BScoring, typename Optimization>
void mpi_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
               const moses_parameters& pa,
               moses_statistics& stats)
{
    logger().info("MPI Threaded MOSES starts, max_evals=%d max_gens=%d",
                  pa.max_evals, pa.max_gens);

    moses_mpi_comm mompi;

    // main worker dispatch loop
    if (!mompi.is_mpi_root()) {
        mpi_moses_worker(mp, mompi);
        return;
    }

    // If we are here, then we are the root node.  The root will act
    // as a dispatcher to all of the worker nodes.
// XXX is mp.best_score thread safe !???? since aonther thread migh be updating this as we
// come around ...

    worker_pool wrkpool(mompi.num_workers());

    std::atomic<int> thread_count(0);

    while ((stats.n_evals < pa.max_evals)
           && (pa.max_gens != stats.n_expansions)
           && (mp.best_score() < pa.max_score))
    {
        mpi_expander_threaded<Scoring, BScoring, Optimization>
            mex(mompi, wrkpool, mp, pa.max_evals - stats.n_evals, stats,
                thread_count);

        // This method blocks until there is a free worker...
        mex.pick_exemplar();

        // If we are here, we unblocked, and have a woker ready to do
        // the work. Create a thread to handle all communications with
        // this worker one-on-one.  Note that the async code will make a
        // copy of mex, put it in a thread private area, and use that,
        // so it is completely safe for us to have mex on stack here.
        std::async(std::launch::async, mex);

// XXX should print stats less often...
        // Print stats in a way that makes them easy to graph.
        // (columns of tab-seprated numbers)
        if (logger().isInfoEnabled()) {
            stringstream ss;
            ss << "Stats: " << stats.n_expansions;
            ss << "\t" << stats.n_evals;    // number of evaluations so far
            ss << "\t" << mp.size();       // size of the metapopulation
            ss << "\t" << mp.best_score(); // score of the highest-ranked exemplar.
            ss << "\t" << get_complexity(mp.best_composite_score()); // as above.
            logger().info(ss.str());
        }
    }

    // Shut down each of the workers.
    int np = mompi.num_workers();
    for (int i=0; i<np; i++) {
        worker_node& worker = wrkpool.borrow();
        mompi.send_finished(worker.rank);
    }

    logger().info("MPI MOSES ends");
};

#else // HAVE_MULTI_THREAD_MPI

template<typename Scoring, typename BScoring, typename Optimization>
struct merge_deme_threaded
{
private:
     metapopulation<Scoring, BScoring, Optimization>& _mp;
     bscored_combo_tree_set& _candidates;
     std::atomic<int>& _thread_count;

public:
    // Cache of arguments we will need.
    merge_deme_threaded(metapopulation<Scoring, BScoring, Optimization>& mp,
                      bscored_combo_tree_set& cands, std::atomic<int>& tcount)
        : _mp(mp), _candidates(cands), _thread_count(tcount)
    {}

    // Function that will run inside a thread
    // Self-deletes upon exit of the thread.
    bool operator()()
    {
        _thread_count++;
        _mp.update_best_candidates(_candidates);
        _mp.merge_candidates(_candidates);
        _thread_count--;
        return false;
    }
};

/// mpi_moses main -- non-theaded version
///
/// Main entry point for MPI moses, for the cases where the MPI
/// implementation does not support threading.  The algorithm is
/// structured into a single send-recv loop: work is sent out, so
/// as to give each worker something to do.  The a check for completed
/// work is made.  Any demes that are received are merged in a
/// separate thread; meanwhile, this loops back and sens out more
/// work.
///
/// The implementation of this loop is very similar to that of 
/// distributed moses.  With appropriate wrappers for the
/// communications API, it might be possible to consolidate both.
/// Not clear if such a consolidation is worth-while...
//
template<typename Scoring, typename BScoring, typename Optimization>
void mpi_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
               const moses_parameters& pa,
               moses_statistics& stats)

{
    logger().info("MPI mon-threaded MOSES starts, max_evals=%d max_gens=%d",
                  pa.max_evals, pa.max_gens);

    moses_mpi_comm mompi;

    // main worker dispatch loop
    if (!mompi.is_mpi_root()) {
        mpi_moses_worker(mp, mompi);
        return;
    }

    // If we are here, then we are the root node.  The root will act
    // as a dispatcher to all of the worker nodes.

    // Pool of free workers maintained in a circular queue.
    std::queue<int> wrkpool;
    size_t tot_workers = mompi.num_workers();
    for (size_t i=0; i<tot_workers; i++)
        wrkpool.push(i+1);

    std::atomic<int> thread_count(0);

    int source = 0;

    while ((stats.n_evals < pa.max_evals)
           && (pa.max_gens != stats.n_expansions)
           && (mp.best_score() < pa.max_score))
    {
        // Feeder: push work out to each worker.
        while (0 < wrkpool.size()) {
            bscored_combo_tree_set::const_iterator exemplar = mp.select_exemplar();
            if (exemplar == mp.end()) {
                if ((tot_workers == wrkpool.size()) && (0 == source)) {
                    logger().warn(
                        "There are no more exemplars in the metapopulation "
                        "that have not been visited and yet a solution was "
                        "not found.  Perhaps reduction is too strict?");
                    goto theend;
                }

                // If there are no more exemplars in our pool, we will
                // have to wait for some more to come rolling in.
                break;
            }

            const combo_tree& extree = get_tree(*exemplar); 
            int worker = wrkpool.front();
            wrkpool.pop();
            int max_evals = pa.max_evals - stats.n_evals;
            mompi.dispatch_deme(worker, extree, max_evals);
        }

        if (0 == source) {
            // Check for results and merge any that are found
            // Note that probe_for_deme() is blocking; it returns only if
            // there is work that we can receive.
            source = mompi.probe_for_deme();
            wrkpool.push(source);
            continue;
        }

        int n_evals = 0;
        bscored_combo_tree_set candidates;
        mompi.recv_deme(source, candidates, n_evals);
        source = 0;

        stats.n_expansions ++;
        stats.n_evals += n_evals;

        // Perform  deme merge in a distinct thread.
        merge_deme_threaded<Scoring, BScoring, Optimization>
            mex(mp, candidates, thread_count);
        std::async(std::launch::async, mex);
    }

theend:

    // Shut down each of the workers.
    for (size_t i=0; i<tot_workers; i++) {
        mompi.send_finished(i+1);
    }

    logger().info("MPI mono-threaded MOSES ends");
}

#endif // HAVE_MULTI_THREAD_MPI

#else // HAVE_MPI

template<typename Scoring, typename BScoring, typename Optimization>
void mpi_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
               const moses_parameters& pa,
               moses_statistics& stats)
{
    OC_ASSERT(0, "There is no MPI support in this version of moses");
};

#endif  // HAVE_MPI

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MPI_MOSES_H
