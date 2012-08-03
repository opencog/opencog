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

#include <future>
#include <opencog/util/pool.h>

#include "metapopulation.h"
#include "moses_params.h"

namespace opencog { namespace moses {

#ifdef HAVE_MPI

struct dispatch_thread
{
   dispatch_thread();
   int rank;
};

class moses_mpi
{
    public:
        moses_mpi();
        ~moses_mpi();

        bool is_mpi_root();
        int num_workers();

        // root methods, to be used only by root node.
        void dispatch_deme(dispatch_thread&, const combo_tree&, int max_evals);
        void recv_deme(dispatch_thread&, bscored_combo_tree_set&, int& n_evals);

        // worker methods, to be used only by workers.
        int recv_more_work();
        void recv_exemplar(combo_tree&);
        void send_deme(const bscored_combo_tree_set&, int);

        pool<dispatch_thread> worker_pool;
    protected:
        void send_tree(const combo_tree&, int target);
        void recv_tree(combo_tree&, int source);

        void send_cscore(const composite_score&, int target);
        void recv_cscore(composite_score&, int source);

        void recv_deme(bscored_combo_tree_set&, int& n_evals, int source);
    private:
        // master state
        std::vector<dispatch_thread> workers;
};


// wrapper class; the only reason this class exists is to make thread
// creation simpler -- threads require a function as an argument, and
// this class provides the needed operator() to make this work.
template<typename Scoring, typename BScoring, typename Optimization>
struct mpi_expander
{
private:
     dispatch_thread _w;
     combo_tree _t;

     dispatch_thread& _worker;
     combo_tree& _extree;

     moses_mpi& _mompi;
     metapopulation<Scoring, BScoring, Optimization>& _mp;
     int _max_evals;
     moses_statistics& _stats;

public:
     // Cache of arguments we will need.
     mpi_expander(moses_mpi& mompi,
         metapopulation<Scoring, BScoring, Optimization>& mp,
         int max_evals, moses_statistics& stats) :
        _worker(_w), _extree(_t),
        _mompi(mompi), _mp(mp), _max_evals(max_evals), _stats(stats)
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
        _worker = _mompi.worker_pool.borrow();
    }

    // Function that will run inside a thread
    bool operator()()
    {
        _mompi.dispatch_deme(_worker, _extree, _max_evals);

        int n_evals = 0;
        bscored_combo_tree_set candidates;
        _mompi.recv_deme(_worker, candidates, n_evals);
cout<<"duuude master got evals="<<n_evals <<" got cands="<<candidates.size()<<endl;
        _mompi.worker_pool.give_back(_worker);

// XXX lock here...
        _stats.n_evals += n_evals;
        _mp.update_best_candidates(candidates);
        _mp.merge_candidates(candidates);
        _mp.log_best_candidates();
cout<<"duuude done merging!"<<endl;
        return false;
    }

};


template<typename Scoring, typename BScoring, typename Optimization>
void mpi_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
               const moses_parameters& pa,
               moses_statistics& stats)
{
    typedef bscored_combo_tree_set::const_iterator mp_cit;

    logger().info("MPI MOSES starts, max_evals=%d max_gens=%d",
                  pa.max_evals, pa.max_gens);

    moses_mpi mompi;

    // Worker processes loop until done, then return.
    // Each worker waits for an exemplar, expands it, then returns
    // the results.
    if (!mompi.is_mpi_root()) {
        while(1) {
            int max_evals = mompi.recv_more_work();
            if (0 >= max_evals) {
cout<<"duuude wioerkr= exiting"<<endl;
                return;
            }

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

    // If we are here, then we are the root node.  The root will act
    // as a dispatcher to all of the worker nodes.
// XXX is mp.best_score thread safe !???? since aonther thread migh be updating this as we
// come around ...
    while ((stats.n_evals < pa.max_evals) 
           && (pa.max_gens != stats.n_expansions)
           && (mp.best_score() < pa.max_score))
    {
        mpi_expander<Scoring, BScoring, Optimization>
            mex(mompi, mp, pa.max_evals - stats.n_evals, stats);

        // This method blocks until there is a free worker...
        mex.pick_exemplar();

        // If we are here, we unblocked, and have a woker ready to do
        // the work. Create a thread to handle all communications with
        // this worker one-on-one.
        std::future<bool> fudone = std::async(std::launch::async, mex);
        bool done = fudone.get();

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
        if (done) break;
    }

    logger().info("MPI MOSES ends");
cout<<"duude are we done yet!?"<<endl;
};

#else

class moses_mpi
{
    public:
        moses_mpi() {}
        ~moses_mpi() {}

        bool is_mpi_root() { return true; }
};

template<typename Scoring, typename BScoring, typename Optimization>
void mpi_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
               const moses_parameters& pa,
               moses_statistics& stats)
{
    OC_ASSERT(0, "There is no MPI support in this version of moses");
};

#endif

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MPI_MOSES_H
