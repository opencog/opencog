/** partial.cc ---
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

#include "partial.h"
#include "scoring.h"

namespace opencog { namespace moses {

using namespace std;

partial_solver::partial_solver(const vector<CTable> &ctables,
                               int as, float noise,
                               const type_tree& table_tt,
                               const vector<combo_tree>& exemplars,
                               const rule& contin_reduct,
                               const optim_parameters& opt_params,
                               const metapop_parameters& meta_params,
                               const moses_parameters& moses_params,
                               const metapop_moses_results_parameters& mmr_pa)

    :_ctables(ctables), _alf_sz(as), _noise(noise), _table_tt(table_tt),
     _exemplars(exemplars), _contin_reduct(contin_reduct),
     _opt_params(opt_params), _meta_params(meta_params),
     _moses_params(moses_params), _mmr_pa(mmr_pa)
{
}

/// Implements the "leave well-enough alone" algorithm.
void partial_solver::solve()
{
    typedef enum_table_bscore BScore;
    boost::ptr_vector<BScore> bscores;
    foreach(const CTable& ctable, _ctables)
       bscores.push_back(new BScore(ctable, _alf_sz, _noise));
    multibscore_based_bscore<BScore> bscore(bscores);
    metapop_moses_results(_exemplars, _table_tt,
                          _contin_reduct, _contin_reduct, bscore,
                          _opt_params, _meta_params, _moses_params,
                          _mmr_pa);

}

void partial_solver::candidate (const combo_tree& cond)
{

}

};};
