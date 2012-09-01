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
        void recv_deme(int source, bscored_combo_tree_set&, int& n_evals);
        void send_finished(int target);

        // worker methods, to be used only by workers.
        int recv_more_work();
        void recv_exemplar(combo_tree&);
        void send_deme(const bscored_combo_tree_set&, int);

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
template<typename Scoring, typename BScoring, typename Optimization>
void mpi_moses_worker(metapopulation<Scoring, BScoring, Optimization>& mp,
                      moses_mpi_comm& mompi)
{
    optim_stats *os = dynamic_cast<optim_stats *> (&mp._dex._optimize);

    // Print header for the loop stats.
    logger().info() << "Unit: # cnt\trun_secs\twait_secs\tevals\tmax_evals\tmetapop_size\tbest_score\tcomplexity\tfield_set_sz";

    // Worker processes loop until done, then return.
    // Each worker waits for an exemplar, expands it, then returns
    // the results.
    int cnt = 0;
    while(1) {
        struct timeval start;
        gettimeofday(&start, NULL);

        // Blocking wait for work unit.
        int max_evals = mompi.recv_more_work();
        if (0 >= max_evals)
            return;
        cnt ++;

        // Measure how long we blocked waiting for work.
        struct timeval stop, elapsed;
        gettimeofday(&stop, NULL);
        timersub(&stop, &start, &elapsed);
        unsigned wait_time = elapsed.tv_sec;
        start = stop;

        combo_tree exemplar;
        mompi.recv_exemplar(exemplar);
        logger().info() << "Allowed " << max_evals 
                        << " evals for recvd exemplar " << exemplar;
        if (!mp._dex.create_deme(exemplar)) {
            // XXX replace this with appropriate message back to root!
            OC_ASSERT(false, "Exemplar failed to expand!\n");
        }
        size_t evals_this_deme = mp._dex.optimize_deme(max_evals);

        mp.merge_deme(mp._dex._deme, mp._dex._rep, evals_this_deme);
        mp._dex.free_deme();

        // logger().info() << "Sending " << mp.size() << " results";
        mompi.send_deme(mp, evals_this_deme);

        // Print timing stats and counts for this work unit.
        if (logger().isInfoEnabled()) {
            gettimeofday(&stop, NULL);
            timersub(&stop, &start, &elapsed);

            stringstream ss;
            ss << "Unit: " << cnt <<"\t" 
               << elapsed.tv_sec << "\t"
               << wait_time << "\t"
               << evals_this_deme << "\t"
               << max_evals << "\t"
               << mp.size() << "\t"  // size of the metapopulation
               << mp.best_score() <<"\t"
               << get_complexity(mp.best_composite_score());
            if (os) {
                ss << "\t" << os->field_set_size;  // number of bits in the knobs
            }
            logger().info(ss.str());
        }

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

/// mpi_moses main -- threaded version
///
/// Main entry point for threaded MPI moses.  Exemplars from the
/// metapopulation are dispatched to different nodes for expansion; the 
/// results are returned here, for merging back into the metapopulation.
///
/// This variant assumes that MPI is thread-safe, and so handles
/// communication to each distinct node in a distinct thread.  Thus, it
/// will not work if the MPI implementation doesn't support
/// MPI_THREAD_MULTIPLE mode.
///
/// This routine has been lightly tested, and seems to work. It has not
/// been heavily teted, because I don't have a multi-node machine with
/// an MPI implementation that supports threads.  There's no particular
/// advantage of this class over the mono-threaded class; its just that
/// the design of this class seemed to be intuitively simpler and easier
/// than writing a home-grown dispatcher loop that the mono-threaed
/// version (below) needs.
///
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

    // Print legend for the columns of the stats.
    print_stats_header(NULL);

    // If we are here, then we are the root node.  The root will act
    // as a dispatcher to all of the worker nodes.
// XXX is mp.best_score thread safe !???? since aonther thread migh be updating this as we
// come around ...

    size_t tot_workers = mompi.num_workers();
    worker_pool wrkpool(tot_workers);

    std::atomic<int> thread_count(0);

    while ((stats.n_evals < pa.max_evals)
           && (pa.max_gens != stats.n_expansions)
           && (mp.best_score() < pa.max_score))
    {
        bscored_combo_tree_set::const_iterator exemplar = mp.select_exemplar();
        if (exemplar == mp.end()) {
            if (wrkpool.available() == tot_workers) {
                logger().warn(
                    "There are no more exemplars in the metapopulation "
                    "that have not been visited and yet a solution was "
                    "not found.  Perhaps reduction is too strict?");
                    goto theend;
                }

            // If there are no more exemplars in our pool, we will
            // have to wait for some more to come rolling in.
            sleep(1);
            continue;
        }

        const combo_tree& extree = get_tree(*exemplar);

        // borrow will block, if there are no workers free.
        worker_node& worker = wrkpool.borrow();

        // If we are here, pool unblocked, and have a woker ready to do
        // the work. Create a thread to handle all communications with
        // this worker one-on-one. Note that the async code will make a
        // copy of of all references, put them in a thread private area,
        // and use those ...
        std::async(std::launch::async, 
            [&]() {
                thread_count++;
                int max_evals = pa.max_evals - stats.n_evals;
                mompi.dispatch_deme(worker.rank, extree, max_evals);

                int n_evals = 0;
                bscored_combo_tree_set candidates;
                mompi.recv_deme(worker.rank, candidates, n_evals);
cout<<"duuude master "<<getpid() <<" from="<<worker.rank << " got evals="<<n_evals <<" got cands="<<candidates.size()<<endl;
                wrkpool.give_back(worker);

                stats.n_expansions ++;
                stats.n_evals += n_evals;

                mp.update_best_candidates(candidates);
                mp.merge_candidates(candidates);

                thread_count--;
                });

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

theend:
    // Shut down each of the workers.
    for (size_t i=0; i<tot_workers; i++) {
        worker_node& worker = wrkpool.borrow();
        mompi.send_finished(worker.rank);
    }

    logger().info("MPI Threaded MOSES ends");
};

#else // HAVE_MULTI_THREAD_MPI

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
    logger().info("MPI mono-threaded MOSES starts, max_evals=%d max_gens=%d",
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
    bool done = false;

    // Print legend for the columns of the stats.
    print_stats_header(NULL);

    while (true)
    {
        // Feeder: push work out to each worker.
        while ((0 < wrkpool.size()) && !done) {
            bscored_combo_tree_set::const_iterator exemplar = mp.select_exemplar();
            if (exemplar == mp.end()) {
                if ((tot_workers == wrkpool.size()) && (0 == source)) {
                    logger().warn(
                        "There are no more exemplars in the metapopulation "
                        "that have not been visited and yet a solution was "
                        "not found.  Perhaps reduction is too strict?");
                    done = true;
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

            // If we are in force-feed mode, don't look at results yet,
            // and instead, give worker more work (even if it's stale).
            if (pa.force_feed) continue;
        }

        int n_evals = 0;
        bscored_combo_tree_set candidates;
        mompi.recv_deme(source, candidates, n_evals);
        source = 0;

        stats.n_expansions ++;
        stats.n_evals += n_evals;

        // Merge results back into population.
        if (!pa.force_feed) {
            mp.update_best_candidates(candidates);
            mp.merge_candidates(candidates);
        } else {
            // Perform deme merge in a distinct thread.
            // We want to keep this thread available for i/o.
            // However, this has the side-effect of giving the worker
            // an exemplar issued from an older, more stale metapop.
            //
            std::async(std::launch::async,
                [&]() { 
                    thread_count++;
                    mp.update_best_candidates(candidates);
                    mp.merge_candidates(candidates);
                    thread_count--;
                });
        }

        // Print stats in a way that makes them easy to graph.
        // (columns of tab-seprated numbers)
        // XXX this is kind-of buggy, since the following data is not
        // updated and collected atomically... other threads may be
        // merging and updating as this print happens. Yuck. Oh well.
        if (logger().isInfoEnabled()) {
            stringstream ss;
            ss << "Stats: " << stats.n_expansions;
            ss << "\t" << stats.n_evals;    // number of evaluations so far
            ss << "\t" << mp.size();       // size of the metapopulation
            ss << "\t" << mp.best_score(); // score of the highest-ranked exemplar.
            ss << "\t" << get_complexity(mp.best_composite_score()); // as above.
            logger().info(ss.str());
        }

        done = done || (stats.n_evals >= pa.max_evals)
                    || (pa.max_gens == stats.n_expansions)
                    || (mp.best_score() >= pa.max_score);

        // Keep looping around till all pending messages have been
        // received.  Failure to do so will prevent MPI from finalizing.
        if (done && (0 == source) && (wrkpool.size() == tot_workers))
            break;
    }

    // Shut down each of the workers.
    for (size_t i=0; i<tot_workers; i++) {
        mompi.send_finished(i+1);
    }

    // Wait until all merge threads have finished.
    while (thread_count != 0) {
        sleep(1);
    }

    stringstream ss;
    ss << "Final stats:\n";
    ss << "Stats: " << stats.n_expansions;
    ss << "\t" << stats.n_evals;    // number of evaluations so far
    ss << "\t" << mp.size();       // size of the metapopulation
    ss << "\t" << mp.best_score(); // score of the highest-ranked exemplar.
    ss << "\t" << get_complexity(mp.best_composite_score()); // as above.
    logger().info(ss.str());

    logger().info() << "MPI: bytes sent=" << mompi.sent_bytes
                    << " bytes received=" << mompi.recv_bytes;
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
