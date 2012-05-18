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
    foreach(const CTable& ctable, _ctables) {
       // FYI, no mem-leak, as ptr_vector seems to call delete.
       bscores.push_back(new BScore(ctable, _alf_sz, _noise));
    }
    _bscore = new multibscore_based_bscore<BScore>(bscores);

    _bad_score = -75;
for(int i=0; i<5; i++) {
    _opt_params.terminate_if_gte = _bad_score;
    _moses_params.max_score = _bad_score;

    metapop_moses_results(_exemplars, _table_type_signature,
                          _contin_reduct, _contin_reduct, *_bscore,
                          _opt_params, _meta_params, _moses_params,
                          *this);
}
}

void partial_solver::candidate (const combo_tree& cand)
{
std::cout<<"duude in the candy="<<cand<<std::endl;
behavioral_score bsa = _bscore->operator()(cand);
int total = 0;
foreach(const score_t& sc, bsa) {
 total += sc;
}
std::cout<<"duuude candy score total="<<total<<std::endl;

    combo_tree trim(cand);

    // Trim down the candidate to just its first predicate.
    pre_it it = trim.begin();
    OC_ASSERT(*it == id::cond, "Error: unexpcected candidate!");

    sib_it sib = it.begin();
    sib++;
    sib_it rest = sib;
    rest++;
    while (rest != it.end()) {
        trim.erase(rest++);
    }
    // Replace the final consequent with a bogus value so that
    // it always scores badly.
    trim.insert_after(sib, enum_t::invalid_enum());
cout<<"trim="<<trim<<endl;

    behavioral_score bs = _bscore->operator()(trim);

total = 0;
foreach(const score_t& sc, bs) {
 total += sc;
}
std::cout<<"duuude trim score total="<<total<<std::endl;

    
    // Evaluate the bscore components for all rows of the ctable
    score_t deleted = 0;
    total = 0;
    foreach(CTable& ctable, _ctables) {
        for (CTable::iterator cit = ctable.begin(); cit != ctable.end(); ) {
            const vertex_seq& vs = cit->first;
            const CTable::counter_t& c = cit->second;
            total += c.total_count();
            // The number that are wrong equals total minus num correct.
            int sc = c.get(eval_binding(vs, trim));
            sc -= c.total_count();
            if (0 == sc) {
                deleted += c.total_count();
                ctable.erase(cit++);
            }
            else cit++;
        }
    }
std::cout<<"duuude tot="<<total<<" deleted ="<<deleted<<std::endl;

    // XXX replace /2 by paramter.
    _bad_score = -(total-deleted)/2;
cout<<"duude gonna ask for score of="<<_bad_score<<endl;

    // Redo the scoring tables, as they cache the score tables (why?)
    boost::ptr_vector<BScore> bscores;
    foreach(const CTable& ctable, _ctables) {
cout<<"new table="<<ctable<<endl;
       // FYI, no mem-leak, as ptr_vector seems to call delete.
       // XXX !?!?! relly?  as we've got a lifetime problem here.
       bscores.push_back(new BScore(ctable, _alf_sz, _noise));
    }
    delete _bscore;
    _bscore = new multibscore_based_bscore<BScore>(bscores);

cout<<endl;
}

};};
