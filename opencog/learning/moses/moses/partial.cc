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

    for(int i=0; i<10; i++) {
cout <<"duuude start loop =================================="<<i<<endl;
        _opt_params.terminate_if_gte = _bad_score;
        _moses_params.max_score = _bad_score;

        metapop_moses_results(_exemplars, _table_type_signature,
                              _contin_reduct, _contin_reduct, *_bscore,
                              _opt_params, _meta_params, _moses_params,
                              *this);
    }
}

void partial_solver::candidates(const metapop_candidates& cands)
{
    foreach(auto &item, cands) {
        const combo_tree& cand = item.first;
        if (candidate(cand)) return;
    }

    // If we are here, then none of the candidates were any good.
    // Tighten up the score, and try again.
    _bad_score *= 0.85;

    foreach(BScore& bs, _bscore->bscores)
        bs.punish += 1.0;

cout <<"duuude nothing good, try aaing with score="<<_bad_score<<endl;
}

bool partial_solver::candidate (const combo_tree& cand)
{
std::cout<<"duude in the candy="<<cand<<std::endl;

    // Are we done yet?
    behavioral_score bs = _bscore->operator()(cand);
    score_t total_score = 0.0;
    foreach(const score_t& sc, bs)
        total_score += sc;

    // XXX replace  by the correct compare, i.e. the orig gte.
    if (0.0 <= total_score) {
std::cout<<"duuude DOOOOOOOOOONE! ="<<total_score<<"\n"<<std::endl;
        return true;
    }
std::cout<<"duuude candy score total="<<total_score<<std::endl;

    // We don't want constants; what else we got?
    pre_it it = cand.begin();
    if (is_enum_type(*it)) {
std::cout<<"duuude got a const\n"<<std::endl;
        return false;
    }

    OC_ASSERT(*it == id::cond, "Error: unexpcected candidate!");

int total;
    // Yank out the first effective predicate, and evaluate it's accuracy
    // A predicate is effective if it makes at least one correct
    // identification.  Otherwise, it may as well be "always false"
    // and is ineffective at identifyng anything.
    sib_it sib = it.begin();
    sib_it predicate = sib;
    unsigned fail_count = 0;
    while(1) {

        if (is_enum_type(*it)) {
            // If we are here, all previous predicates were ineffective.
std::cout<<"duuude got an ineffective const\n"<<std::endl;
            return false;
        }

        predicate = sib;
        sib++;
        vertex consequent = *sib;
        sib++;

total = 0;
        // Count how many items the first predicate mis-identifies.
        fail_count = 0;
        unsigned good_count = 0;
        foreach(CTable& ctable, _ctables) {
            for (CTable::iterator cit = ctable.begin(); cit != ctable.end(); cit++) {
                const vertex_seq& vs = cit->first;
                const CTable::counter_t& c = cit->second;

total += c.total_count();
                vertex pr = eval_throws_binding(vs, predicate);
                if (pr == id::logical_true) {
                    if (c.total_count() != c.get(consequent))
                        fail_count++;
                    else
                        good_count++;
                }
            }
        }
std::cout<<"duuude tot="<<total<<" fail ="<<fail_count <<" good="<<good_count<<std::endl;

        // XXX Ineffective predicates may be due to enums that have been
        // completely accounted for ... not sure what to do about that...
        if (0 < good_count)
            break;
    }

    // If the fail count isn't zero, then punish more strongly, and try again.
    if (fail_count) {
cout<<"duuude non zero fail count\n"<<endl;
        return false;
    }

    // If we are here, the first predicate is correctly identifying
    // all cases; we can save it, and remove all table elements that
    // it correctly identified.

    total = 0;
    unsigned deleted = 0;
    foreach(CTable& ctable, _ctables) {
        for (CTable::iterator cit = ctable.begin(); cit != ctable.end(); ) {
            const vertex_seq& vs = cit->first;
            const CTable::counter_t& c = cit->second;

            unsigned tc = c.total_count();
            total += tc;

            vertex pr = eval_throws_binding(vs, predicate);
            if (pr == id::logical_true) {
                deleted += tc;
                ctable.erase(cit++);
            }
            else cit++;
        }
    }
    // XXX replace 0.5 by parameter.
    // XXX should be, ummm, less than the number of a single type in the table,
    // else a constant beats this score...
    _bad_score = -0.45 * (score_t(total) - score_t(deleted));
cout<<"duude deleted="<<deleted <<" out of total="<<total<< " gonna ask for score of="<<_bad_score<<endl;

    // Redo the scoring tables, as they cache the score tables (why?)
    boost::ptr_vector<BScore> bscores;
    foreach(const CTable& ctable, _ctables) {
        // FYI, no mem-leak, as ptr_vector seems to call delete.
        // XXX !?!?! relly?  as we've got a lifetime problem here.
        bscores.push_back(new BScore(ctable, _alf_sz, _noise));
    }
    delete _bscore;
    _bscore = new multibscore_based_bscore<BScore>(bscores);

cout<<endl;
    return true;
}

};};
