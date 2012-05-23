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
#include <opencog/comboreduct/combo/table.h>
#include <opencog/comboreduct/combo/type_tree.h>
#include <opencog/comboreduct/combo/vertex.h>
#include "../main/moses_exec.h"

namespace opencog { namespace moses {

using namespace combo;

/// Implements the "leave well-enough alone" algorithm.
class partial_solver
{
    public:
        partial_solver(const vector<CTable> &ctables,
                       const type_tree& table_tt,
                       const vector<combo_tree>& exemplars,
                       const rule& reduct,
                       const optim_parameters& opt_params,
                       const metapop_parameters& meta_params,
                       const moses_parameters& moses_params,
                       const metapop_printer& mmr_pa);

        ~partial_solver();

        void solve();

        /// The metapop "printer" callback.
        /// This gives us an opportunity to get our hands on the best
        /// exemplars that moses found, so that we can see if they are
        /// "good enough".
        template<typename Score, typename BScore, typename Optimization>
        void operator()(metapopulation<Score, BScore, Optimization> &metapop)
        {
            _num_evals = metapop.n_evals();

            if ((_moses_params.max_evals <= _num_evals) ||
                metapop.empty())
                _done = true;

            // _num_gens = metapop.??? FIXME

            if (!_done)
                candidates(metapop.best_candidates());

            if (_done)
                _printer(metapop);
        }

    protected:
        void candidates(const metapop_candidates&);
        bool candidate(const combo_tree&);
        bool recurse();
        void effective(combo_tree::iterator,
                       unsigned& good_count,  // return value
                       unsigned& fail_count); //return value
        void trim_table(std::vector<CTable>&,
                        const combo_tree::iterator,
                        unsigned& deleted,   // return value
                        unsigned& total);    // return value
        void refresh(const metapop_candidates&,
                     const combo_tree&);

    private:

        // Copy, more or less, or arguments, so that moses
        // can be called with these values.
        std::vector<CTable> _ctables;
        const type_tree& _table_type_signature;
        std::vector<combo_tree> _exemplars;
        const rule& _reduct;
        optim_parameters _opt_params;
        const metapop_parameters& _meta_params;
        moses_parameters _moses_params;
        const metapop_printer& _printer;

        // typedef enum_filter_bscore BScore;
        typedef enum_graded_bscore BScore;
        multibscore_based_bscore<BScore> *_bscore;

        score_t _bad_score; // Score we want to get to, at each round.
        int _num_evals;  // number of evaluations
        int _num_gens;   // number of generations
        bool _done;   // Are we there, yet?

        // XXX object lifetime weirdness ... 
        boost::ptr_vector<BScore> score_seq;

        // Use recursion to narrow the failure cases.
        unsigned _fail_recurse_count;
        double _best_fail_ratio;
        combo_tree _best_fail_tree;
        combo_tree::iterator _best_fail_pred;
};

};};
