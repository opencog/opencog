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

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

partial_solver::partial_solver(const vector<CTable> &ctables,
                               int as, float noise,
                               const type_tree& table_tt,
                               const vector<combo_tree>& exemplars,
                               const rule& contin_reduct,
                               const optim_parameters& opt_params,
                               const metapop_parameters& meta_params,
                               const moses_parameters& moses_params,
                               const metapop_printer& mmr_pa)

    :_ctables(ctables), _alf_sz(as), _noise(noise),
     _table_type_signature(table_tt),
     _exemplars(exemplars), _contin_reduct(contin_reduct),
     _opt_params(opt_params), _meta_params(meta_params),
     _moses_params(moses_params), _mmr_pa(mmr_pa)
{
}

partial_solver::~partial_solver()
{
    delete _bscore;
}

/// Implements the "leave well-enough alone" algorithm.
void partial_solver::solve()
{
    boost::ptr_vector<BScore> bscores;
    foreach(const CTable& ctable, _ctables)
       bscores.push_back(new BScore(ctable, _alf_sz, _noise));
    _bscore = new multibscore_based_bscore<BScore>(bscores);

    _opt_params.terminate_if_gte = -75;
    _moses_params.max_score = -75;

    metapop_moses_results(_exemplars, _table_type_signature,
                          _contin_reduct, _contin_reduct, *_bscore,
                          _opt_params, _meta_params, _moses_params,
                          *this);

}

void partial_solver::candidate (const combo_tree& cand)
{
std::cout<<"duude in the candy="<<cand<<std::endl;
    behavioral_score bs = _bscore->operator()(cand);

    score_t total = 0;
    foreach(const score_t& sc, bs) {
        total += sc;
    }
std::cout<<"duuude total="<<total<<std::endl;

    combo_tree trim(cand);

    // Trim down the candidate to just its first predicate.
    pre_it it = trim.begin();
    OC_ASSERT(*it == id::cond, "Error: unexpcected candidate!");

    sib_it sib = it.begin();
    sib++;
    sib_it rest = sib;
    rest++;
    while (rest != it.end()) {
        sib_it next = rest;
        next++;
cout<<"duuude kill subtree="<<combo_tree(rest)<<endl;
        trim.erase(rest);
        rest = next;
    }
    trim.insert_after(sib, enum_t("-*-*-this-is-a-BOGUS-enum-*-*-"));
cout<<"trim="<<trim<<endl;

    bs = _bscore->operator()(trim);

    total = 0;
    foreach(const score_t& sc, bs) {
        total += sc;
    }
std::cout<<"duuude trim total="<<total<<std::endl;
}

};};
