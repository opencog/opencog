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

moses_mpi_comm::moses_mpi_comm() :
    sent_bytes(0), recv_bytes(0)
{
    int have_thread_support = MPI::Init_thread(MPI_THREAD_MULTIPLE);

    const char * sup = "";
    switch(have_thread_support) {
        case MPI_THREAD_SINGLE:     sup = "MPI_THREAD_SINGLE";     break; 
        case MPI_THREAD_FUNNELED:   sup = "MPI_THREAD_FUNNELED";   break; 
        case MPI_THREAD_SERIALIZED: sup = "MPI_THREAD_SERIALIZED"; break; 
        case MPI_THREAD_MULTIPLE:   sup = "MPI_THREAD_MULTIPLE";   break; 
    }
    logger().info() << "MPI support level: " << sup;

#ifdef HAVE_MULTI_THREAD_MPI
    if (MPI_THREAD_MULTIPLE != have_thread_support) {
        MPI::Finalize();
        logger().error() << "This MPI implementation lacks threading support!";
        OC_ASSERT(false, "This MPI implementation lacks threading support!");
    }
#endif
    
    int num_procs = MPI::COMM_WORLD.Get_size();
    int rank = MPI::COMM_WORLD.Get_rank();

    int namelen;
    char processor_name[MPI_MAX_PROCESSOR_NAME];
    MPI_Get_processor_name(processor_name, &namelen);

    logger().info() << "MPI initialized on host \"" << processor_name
        << "\" rank=" << rank << " out of num_procs=" << num_procs;

    // MPI::COMM_WORLD.Bcast(&answer, 1, MPI::INT, 0);
}

moses_mpi_comm::~moses_mpi_comm()
{
    MPI::Finalize();
}

bool moses_mpi_comm::is_mpi_root()
{
    return (ROOT_NODE == MPI::COMM_WORLD.Get_rank());
}

int moses_mpi_comm::num_workers()
{
    // Not counting the root...
    return MPI::COMM_WORLD.Get_size() - 1;
}


void moses_mpi_comm::send_finished(int target)
{
    int max_evals = 0;
    MPI::COMM_WORLD.Send(&max_evals, 1, MPI::INT,
                         target, MSG_MAX_EVALS);
    sent_bytes += sizeof(int);
}

/// Send a combo tree to the target node 
void moses_mpi_comm::send_tree(const combo_tree &tr, int target)
{
    std::stringstream ss;
    ss << tr;
    const std::string& xtree = ss.str();
    const char * stree = xtree.c_str();
    int stree_sz = xtree.size();
    MPI::COMM_WORLD.Send(&stree_sz, 1, MPI::INT, target, MSG_COMBO_TREE_LEN);
    MPI::COMM_WORLD.Send(stree, stree_sz, MPI::CHAR, target, MSG_COMBO_TREE);

    sent_bytes += sizeof(int) + stree_sz;
}

/// Receive a combo tree from the source node.
void moses_mpi_comm::recv_tree(combo_tree &tr, int source)
{
    int stree_sz = 0;
    MPI::COMM_WORLD.Recv(&stree_sz, 1, MPI::INT, source, MSG_COMBO_TREE_LEN);

    char stree[stree_sz+1];
    MPI::COMM_WORLD.Recv(stree, stree_sz, MPI::CHAR, source, MSG_COMBO_TREE);
    stree[stree_sz] = 0;
    std::stringstream ss;
    ss << stree;
    ss >> tr;

    recv_bytes += sizeof(int) + stree_sz;
}

/// Send composite score to target node
void moses_mpi_comm::send_cscore(const composite_score &cs, int target)
{
    double flattened_score[COMPOSITE_SCORE_SIZE];
    flattened_score[0] = cs.get_score();
    flattened_score[1] = cs.get_complexity();
    flattened_score[2] = cs.get_complexity_penalty();
    MPI::COMM_WORLD.Send(flattened_score, COMPOSITE_SCORE_SIZE,
         MPI::DOUBLE, target, MSG_CSCORE);

    sent_bytes += 3*sizeof(double);
}

/// Receive composite score from source node
void moses_mpi_comm::recv_cscore(composite_score &cs, int source)
{
    double flattened_score[COMPOSITE_SCORE_SIZE];
    MPI::COMM_WORLD.Recv(flattened_score, COMPOSITE_SCORE_SIZE,
         MPI::DOUBLE, source, MSG_CSCORE);

    cs = composite_score(flattened_score[0], flattened_score[1], flattened_score[2]);
    recv_bytes += 3*sizeof(double);
}

/// dispatch_deme -- Send an exemplar to node for deme expansion.
///
/// @max_evals is the maximum number of evaluations the worker should perform.
//
void moses_mpi_comm::dispatch_deme(int target, 
                              const combo_tree &tr, int max_evals)
{
    MPI::COMM_WORLD.Send(&max_evals, 1, MPI::INT, target, MSG_MAX_EVALS);
    send_tree(tr, target);
    sent_bytes += sizeof(int);
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
int moses_mpi_comm::recv_more_work()
{
    int max_evals = 0;
    MPI::COMM_WORLD.Recv(&max_evals, 1, MPI::INT, ROOT_NODE, MSG_MAX_EVALS);
    recv_bytes += sizeof(int);
    return max_evals;
}

/// recv_exemplar -- used by worker to receive an exemplar
///
/// This method is intended for use only on worker nodes.
//
void moses_mpi_comm::recv_exemplar(combo_tree& exemplar)
{
    recv_tree(exemplar, ROOT_NODE);
}

/// send_deme -- send the completed deme from the worker back to root
///
/// This sends a pretty big glob.
// XXX TODO -- trim the deme down, before sending, by using the worst acceptable score.
void moses_mpi_comm::send_deme(const metapopulation& mp, int n_evals)
{
    MPI::COMM_WORLD.Send(&n_evals, 1, MPI::INT, ROOT_NODE, MSG_NUM_EVALS);

    int num_trees = mp.size();
    MPI::COMM_WORLD.Send(&num_trees, 1, MPI::INT, ROOT_NODE, MSG_NUM_COMBO_TREES);
    sent_bytes += 2*sizeof(int);

    scored_combo_tree_ptr_set_cit it;
    for (it = mp.begin(); it != mp.end(); it++) {
        const scored_combo_tree& btr = *it;

        // We are going to send only the composite score, and not the
        // full behavioural score.  Basically, the full bscore is just
        // not needed for the current most popular merge technique.
        send_cscore(btr.get_composite_score(), ROOT_NODE);
        send_tree(btr.get_tree(), ROOT_NODE);
    }
}

/// probe_for_deme -- return value >0 if there is a deme ready to recv.
///
/// This method blocks if there is no work to receive.
int moses_mpi_comm::probe_for_deme()
{
    MPI::Status status;
    MPI::COMM_WORLD.Probe(MPI_ANY_SOURCE, MPI_ANY_TAG, status);
    int tag = status.Get_tag();
    OC_ASSERT(tag == MSG_NUM_EVALS);
    int source = status.Get_source();
    OC_ASSERT(source != 0);
    return source;
}

/// recv_deme -- receive the deme sent by send_deme()
/// @n_evals is the actual number of evaluations performed.
///
/// This will receive a deme from any source; it is safe to call this
/// routine with source=MPI_ANY_SOURCE, it will then pick up a deme,
/// in an atomic, unfragmented way, from the first source that sent
/// one to us.
void moses_mpi_comm::recv_deme(int source,
                               scored_combo_tree_set& cands,
                               int& n_evals,
                               const demeID_t& demeID)
{
    MPI::Status status;
    MPI::COMM_WORLD.Recv(&n_evals, 1, MPI::INT, source, MSG_NUM_EVALS, status);

    // This allows recv to be called with MPI_ANY_SOURCE, yet finish
    // the reception with the correct materials.
    source = status.Get_source();

    int num_trees = 0;
    MPI::COMM_WORLD.Recv(&num_trees, 1, MPI::INT, source, MSG_NUM_COMBO_TREES);
    recv_bytes += sizeof(int);
    for (int i=0; i<num_trees; i++) {
        composite_score sc;
        recv_cscore(sc, source);
        combo_tree tr;
        recv_tree(tr, source);

        scored_combo_tree bsc_tr(tr, demeID, sc);
        cands.insert(bsc_tr);
    }
}


void mpi_moses_worker(metapopulation& mp,
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
        if (!mp._dex.create_demes(exemplar, 0 /* TODO replace with the
                                                 right expansion
                                                 count */)) {
            // XXX replace this with appropriate message back to root!
            OC_ASSERT(false, "Exemplar failed to expand!\n");
        }

        // XXX TODO should probably fetch max_time from somewhere...
        time_t max_time = INT_MAX;
        vector<unsigned> actl_evals = mp._dex.optimize_demes(max_evals, max_time);

        mp.merge_demes(mp._dex._demes, mp._dex._reps, actl_evals);
        mp._dex.free_demes();

        // logger().info() << "Sending " << mp.size() << " results";
        unsigned total_evals = boost::accumulate(actl_evals, 0U);
        mompi.send_deme(mp, total_evals);

        // Print timing stats and counts for this work unit.
        if (logger().isInfoEnabled()) {
            gettimeofday(&stop, NULL);
            timersub(&stop, &start, &elapsed);

            std::stringstream ss;
            ss << "Unit: " << cnt <<"\t" 
               << elapsed.tv_sec << "\t"
               << wait_time << "\t"
               << total_evals << "\t"
               << max_evals << "\t"
               << mp.size() << "\t"  // size of the metapopulation
               << mp.best_score() <<"\t"
               << mp.best_composite_score().get_complexity();
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


worker_pool::worker_pool(int num_workers)
{
    workers.resize(num_workers);
    // The root process itself is not in the pool.
    // i.e. pool starts at 1 not at 0.
    for (int i=0; i<num_workers; i++) {
        worker_node& worker = workers[i];
        worker.rank = i+1;  // rank 0 is root, and not a worker.
        give_back(worker);
    }
}

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
/// been heavily tested, because I don't have a multi-node machine with
/// an MPI implementation that supports threads.  There's no particular
/// advantage of this class over the mono-threaded class; its just that
/// the design of this class seemed to be intuitively simpler and easier
/// than writing a home-grown dispatcher loop that the mono-threaded
/// version (below) needs.
///
void mpi_moses(metapopulation& mp,
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
// XXX is mp.best_score thread safe !???? since another thread might be updating this as we
// come around ...

    size_t tot_workers = mompi.num_workers();
    worker_pool wrkpool(tot_workers);

    std::atomic<int> thread_count(0);

    struct timeval start;
    gettimeofday(&start, NULL);

    while ((stats.n_evals < pa.max_evals)
           && (pa.max_gens != stats.n_expansions)
           && (mp.best_score() < pa.max_score))
    {
        scored_combo_tree_set::const_iterator exemplar = mp.select_exemplar();
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
                scored_combo_tree_set candidates;
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

            struct timeval stop, elapsed;
            gettimeofday(&stop, NULL);
            timersub(&stop, &start, &elapsed);
            start = stop;

            std::stringstream ss;
            ss << "Stats: " << stats.n_expansions;
            ss << "\t" << stats.n_evals;    // number of evaluations so far
            ss << "\t" << elapsed.tv_sec;   // wall-clock time.
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
/// structured into a single send-recv loop: work is sent out, so as
/// to give each worker something to do.  Then a check for completed
/// work is made.  Any demes that are received are merged in a
/// separate thread; meanwhile, this loops back and sends out more
/// work.
///
/// The implementation of this loop is very similar to that of 
/// distributed moses.  With appropriate wrappers for the
/// communications API, it might be possible to consolidate both.
/// Not clear if such a consolidation is worth-while...
//
void mpi_moses(metapopulation& mp,
               const moses_parameters& pa,
               moses_statistics& stats)

{
    logger().info("MPI mono-threaded MOSES starts, max_evals=%d max_gens=%d",
                  pa.max_evals, pa.max_gens);

    moses_mpi_comm mompi;

    // Worker or dispatcher?
    if (!mompi.is_mpi_root()) {
        mpi_moses_worker(mp, mompi);
        return;
    }

    // If we are here, then we are the root node.  The root will act
    // as a dispatcher to all of the worker nodes.

    // Pool of free workers maintained in a circular queue. The
    // content of wrkpool is the source of the workers (see below)
    std::queue<int> wrkpool;
    size_t tot_workers = mompi.num_workers();
    for (size_t i=0; i<tot_workers; i++)
        wrkpool.push(i+1);

    std::atomic<int> thread_count(0);

    // worker source (from 1 to number of workers).
    // OC_ASSERT(false, "TODO: understand what is the role source=0 exactly");
    int source = 0;
    bool done = false;

    // Print legend for the columns of the stats.
    print_stats_header(NULL, false /* XXX stats for diversity, should be fixed */);

    // Main worker dispatch loop
    while (true)
    {
        // Feeder: push work out to each worker.
        while ((0 < wrkpool.size()) && !done) {
            scored_combo_tree_ptr_set_cit exemplar = mp.select_exemplar();
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

            const combo_tree& extree = exemplar->get_tree(); 
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
        scored_combo_tree_set candidates;
        stats.n_expansions ++;

        // XXX TODO instead of overwritting the demeID it should be
        // correctly defined by the worker and send back to the
        // dispatcher. That way we can have the breadth_first
        // componant of the demeID right.
        mompi.recv_deme(source, candidates, n_evals,
                        demeID_t(stats.n_expansions));
        source = 0;

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
            std::stringstream ss;
            ss << "Stats: " << stats.n_expansions;
            ss << "\t" << stats.n_evals;    // number of evaluations so far
            ss << "\t" << mp.size();       // size of the metapopulation
            ss << "\t" << mp.best_score(); // score of the highest-ranked exemplar.
            ss << "\t" << mp.best_composite_score().get_complexity(); // as above.
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

    std::stringstream ss;
    ss << "Final stats:\n";
    ss << "Stats: " << stats.n_expansions;
    ss << "\t" << stats.n_evals;    // number of evaluations so far
    ss << "\t" << mp.size();       // size of the metapopulation
    ss << "\t" << mp.best_score(); // score of the highest-ranked exemplar.
    ss << "\t" << mp.best_composite_score().get_complexity(); // as above.
    logger().info(ss.str());

    logger().info() << "MPI: bytes sent=" << mompi.sent_bytes
                    << " bytes received=" << mompi.recv_bytes;
    logger().info("MPI mono-threaded MOSES ends");
}

#endif // HAVE_MULTI_THREAD_MPI


} // ~namespace moses
} // ~namespace opencog

#endif /* HAVE_MPI */
