/** partial.h ---
 *
 * Copyright (C) 2012 Poulin Holdings
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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

#include <vector>
#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/type_checker/type_tree.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/reduct/reduct.h>

#include "../optimization/hill-climbing.h"
#include "../metapopulation/metapopulation.h"
#include "moses_main.h"

namespace opencog { namespace moses {

using namespace combo;
using namespace reduct;

/// Implements the "leave well-enough alone" algorithm.
class partial_solver
{
    public:
        partial_solver(const vector<CTable> &ctables,
                       const type_tree& table_tt,
                       const vector<combo_tree>& exemplars,
                       const rule& reduct,
                       const optim_parameters& opt_params,
                       const hc_parameters& hc_params,
                       const metapop_parameters& meta_params,
                       const moses_parameters& moses_params,
                       const metapop_printer& mmr_pa);

        ~partial_solver();

        void solve();

        /// The metapop "printer" callback.
        /// This gives us an opportunity to get our hands on the best
        /// exemplars that moses found, so that we can see if they are
        /// "good enough".
        void operator()(metapopulation &metapop,
                        moses_statistics& stats)
        {
            _num_evals = stats.n_evals;

            // If _most_good is zero, then we are here because the
            // metapopulation came to a natural end. We are done, in
            // that case.  Otherwise, restart the search where we left
            // off.
            if (0 == _most_good)
                _done = true;

            // Well, the clock may have run out, and yet some perfect
            // scorers were found.  Ignore them. We are done, anyway.
            if (_moses_params.max_evals <= _num_evals)
                _done = true;

            if (_done)
                final_cleanup(metapop);
            else
                refresh(metapop);
        }

        static bool check_candidates(scored_combo_tree_set& cands, void *ud)
        {
            partial_solver *ps = (partial_solver *) ud;
            return ps->eval_candidates(cands);
        }

    protected:
        bool eval_candidates(const scored_combo_tree_set&);
        void eval_candidate(const combo_tree&);
        void record_prefix();
        void effective(combo_tree::iterator,
                       unsigned& good_count,  // return value
                       unsigned& fail_count); //return value
        void trim_table(std::vector<CTable>&,
                        const combo_tree::iterator,
                        unsigned& deleted,   // return value
                        unsigned& total);    // return value
        void refresh(const metapopulation&);

        void final_cleanup(const metapopulation&);
    private:

        // Copy, more or less, or arguments, so that moses
        // can be called with these values.
        std::vector<CTable> _ctables;
        std::vector<CTable> _orig_ctables;
        const type_tree& _table_type_signature;
        std::vector<combo_tree> _exemplars;
        combo_tree _leader;
        unsigned _prefix_count;
        const rule& _reduct;
        optim_parameters _opt_params;
        hc_parameters _hc_params;
        metapop_parameters _meta_params;
        moses_parameters _moses_params;
        const metapop_printer& _printer;

        // typedef enum_filter_bscore BScore;
        // typedef enum_graded_bscore BScore;
        typedef enum_effective_bscore BScore;
        multibscore_based_bscore *_bscore;
        multibehave_cscore *_cscore;

        typedef enum_table_bscore StraightBScore;
        multibscore_based_bscore *_straight_bscore;
        multibehave_cscore *_straight_cscore;

        int _num_evals;     // number of evaluations
        int _num_gens;      // number of generations
        bool _done;         // Are we there, yet?

        unsigned _most_good;
        combo_tree::iterator _best_predicate;

        // XXX keep these here due object lifetime weirdness ... 
        // There is something bizarre/wrong with how ptr_vector works!?!?
        typedef boost::ptr_vector<bscore_base> BScorerSeq;
        BScorerSeq score_seq;
        BScorerSeq straight_score_seq;
};

};};
