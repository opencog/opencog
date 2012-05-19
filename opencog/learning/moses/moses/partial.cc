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
                               const rule& reduct,
                               const optim_parameters& opt_params,
                               const metapop_parameters& meta_params,
                               const moses_parameters& moses_params,
                               const metapop_printer& mmr_pa)

    :_ctables(ctables), _alf_sz(as), _noise(noise),
     _table_type_signature(table_tt),
     _exemplars(exemplars), _reduct(reduct),
     _opt_params(opt_params), _meta_params(meta_params),
     _moses_params(moses_params), _printer(mmr_pa),
     _fail_recurse_count(0), _best_fail_ratio(1.0e30),
     _best_fail_tree(combo_tree()),
     _best_fail_pred(_best_fail_tree.end())
{
}

partial_solver::~partial_solver()
{
    delete _bscore;
}

/// Implements the "leave well-enough alone" algorithm.
void partial_solver::solve()
{
    _num_gens = 0;
    _done = false;

    unsigned tab_sz = 0;
    score_seq.clear();
    foreach(const CTable& ctable, _ctables) {
        // FYI, no mem-leak, as ptr_vector seems to call delete.
        // XXX??? That's just weird/wrong -- double check this.
        score_seq.push_back(new BScore(ctable, _alf_sz, _noise));

        tab_sz += ctable.uncompressed_size();
    }
    _bscore = new multibscore_based_bscore<BScore>(score_seq);

#define FRACTION 0.5
    // XXX this is wrong, should be delta on best_possible score.
    _bad_score = - floor(FRACTION * score_t(tab_sz));

int i=0;
    while(1) {
        if (_moses_params.max_evals <= _num_evals) break; 

        _opt_params.terminate_if_gte = _bad_score;
        _moses_params.max_score = _bad_score;
        _moses_params.max_evals -= _num_evals;
        _moses_params.max_gens -= _num_gens; // XXX wrong

cout <<"duuude start loop ===================== rec="<<_fail_recurse_count <<" ======= "<<i++<<" ask="<<_bad_score<<endl;
cout <<"duuude ================= nev="<<_num_evals <<" max_ev= "<<_moses_params.max_evals<<endl;
        metapop_moses_results(_exemplars, _table_type_signature,
                              _reduct, _reduct, *_bscore,
                              _opt_params, _meta_params, _moses_params,
                              *this);
        if (_done) break;
    }
cout<<"duuu YYYYYYYYYYYYYYYYYYYYYYYYYYYYY DONE !"<<endl;
}

/// Evaluate each of the candidate solutions, see if any of the leading
/// predicates are "good enough".  If we find one that is, then trim
/// the scoring tables, and return (so as to run moses on the smaller
/// problem).
void partial_solver::candidates(const metapop_candidates& cands)
{
cout<<"duuude got cands sz="<<cands.size()<<endl;
    foreach(auto &item, cands) {
        const combo_tree& cand = item.first;
        if (candidate(cand)) {
            refresh(cands, cand);
            return;
        }
    }

#if 0
// XXX disable recursion for now, its actually harder than it seems.
    if (_best_fail_ratio < 0.2) {
        if (recurse()) return;
    }
#endif

    // If we are here, then none of the candidates were any good.
#if 0
    // Tighten up the score, and try again.
    // XXX this is wrong, should be fraction of the max score.
    // _bad_score = ceil(0.8 * _bad_score);

    // XXX too much punishment just trains for ineffectual first predicates.
    foreach(BScore& bs, _bscore->bscores)
        bs.punish *= 1.5;
#endif
    // If we are here, then none of the candidates were any good.
    // Try again, priming the metapop with the previous best.
    _exemplars.clear();
    foreach(auto &item, cands) {
        const combo_tree& cand = item.first;
        _exemplars.push_back(cand);
    }


cout <<"duuude nothing good, try aaing with score="<<_bad_score<<endl;
}

/// Compute the effectiveness of the predicate.
/// That is, return how many answers it got right, and how many got
/// flat-out wrong.
void partial_solver::effective(combo_tree::iterator pred,
                       unsigned& good_count,  // return value
                       unsigned& fail_count)  //return value
{
    sib_it predicate = pred;
    sib_it conq = next(predicate);
    vertex consequent = *conq;

int total=0;
    // Count how many items the first predicate mis-identifies.
    foreach(CTable& ctable, _ctables) {
        for (CTable::iterator cit = ctable.begin(); cit != ctable.end(); cit++) {
            const vertex_seq& vs = cit->first;
            const CTable::counter_t& c = cit->second;

total += c.total_count();
            vertex pr = eval_throws_binding(vs, predicate);
            if (pr == id::logical_true) {
                unsigned num_right = c.get(consequent);
                unsigned num_total = c.total_count();
                if (num_right != num_total)
                    fail_count += num_total - num_right;
                else
                    good_count += num_right;
            }
        }
    }
std::cout<<"duuude effective tot="<<total<<" fail ="<<fail_count <<" good="<<good_count<<std::endl;
}


/// Remove all rows from the table that satisfy the predicate.
void partial_solver::trim_table(std::vector<CTable>& tabs,
                                const combo_tree::iterator predicate,
                                unsigned& deleted,   // return value
                                unsigned& total)    // return value

{
    foreach(CTable& ctable, tabs) {
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
}

/// Refresh the exemplars list.
/// We assume the previous list wasn't bad, but since we've handled the
/// leading predicate already, we don't need it.  Chop it off.
void partial_solver::refresh(const metapop_candidates& cands,
                             const combo_tree& curr_cand)
{
    foreach(auto &item, cands) {
        const combo_tree& cand = item.first;
        if (cand == curr_cand)
             continue;  // We already trimmed this one down, earlier.
        _exemplars.push_back(cand);
    }
}

/// Evaluate a single candidate
bool partial_solver::candidate (const combo_tree& cand)
{
std::cout<<"duude in the candy="<<cand<<std::endl;

    // Are we done yet?
    behavioral_score bs = _bscore->operator()(cand);
    score_t total_score = 0.0;
    foreach(const score_t& sc, bs)
        total_score += sc;

    // XXX replace  by the correct compare, i.e. the orig gte.
    if (-0.5 <= total_score) {
std::cout<<"duuude DOOOOOOOOOONE! ="<<total_score<<"\n"<<std::endl;
        _done = true;
        return true;
    }
std::cout<<"duuude candy score total="<<total_score<<std::endl;

    // Next time around, we need to beat this best score.
    if (_bad_score < total_score)
        _bad_score = ceil (0.8*total_score);

    // We don't want constants; what else we got?
    pre_it it = cand.begin();
    if (is_enum_type(*it)) {
std::cout<<"duuude got a const\n"<<std::endl;
        return false;
    }

    OC_ASSERT(*it == id::cond, "Error: unexpected candidate!");

    // Yank out the first effective predicate, and evaluate it's accuracy
    // A predicate is effective if it makes at least one correct
    // identification.  Otherwise, it may as well be "always false"
    // and is ineffective at identifyng anything.
    sib_it sib = it.begin();
    sib_it predicate = sib;
    unsigned fail_count = 0;
    unsigned good_count = 0;
    while(1) {

        if (is_enum_type(*sib)) {
            // If we are here, all previous predicates were ineffective.
std::cout<<"duuude got an ineffective const\n"<<std::endl;
            return false;
        }

        predicate = sib;
        sib++;
        sib++;

        // Count how many items the first predicate mis-identifies.
        fail_count = 0;
        good_count = 0;
        effective(predicate, good_count, fail_count);

        // XXX Ineffective predicates may be due to enums that have been
        // completely accounted for ... not sure what to do about that...
        if (0 < good_count)
            break;
    }

    // If the fail count isn't zero, at least try to find the most graceful
    // winner.
    if (fail_count) {
        double fail_ratio = double(fail_count) / double(good_count);
        if (fail_ratio < _best_fail_ratio) {
            _best_fail_ratio = fail_ratio;
            _best_fail_tree = cand;
            _best_fail_pred = predicate;
        }

cout<<"duuude non zero fail count\n"<<endl;
        return false;
    }

    // If we are here, the first predicate is correctly identifying
    // all cases; we can save it, and remove all table elements that
    // it correctly identified.
    unsigned total_rows = 0;
    unsigned deleted = 0;
    trim_table(_ctables, predicate, deleted, total_rows);

    // XXX replace 0.5 by parameter.
    // XXX should be, ummm, less than the number of a single type in the table,
    // else a constant beats this score...
    _bad_score = -floor(0.45 * (score_t(total_rows) - score_t(deleted)));
cout<<"duude deleted="<<deleted <<" out of total="<<total_rows<< " gonna ask for score of="<<_bad_score<<endl;

    // Redo the scoring tables, as they cache the score tables (why?)
    score_seq.clear();
    foreach(const CTable& ctable, _ctables) {
        // FYI, no mem-leak, as ptr_vector seems to call delete.
        // XXX !?!?! relly?  as we've got a lifetime problem here.
        score_seq.push_back(new BScore(ctable, _alf_sz, _noise));
    }
    delete _bscore;
    _bscore = new multibscore_based_bscore<BScore>(score_seq);

    // Prime the pump with the remainder of what had been learned.
    combo_tree fresh(cand);
    pre_it fit = fresh.begin();
    sib_it fsib = fit.begin();
    sib_it fpred = next(fsib, distance(it.begin(), predicate));
    while(1) {

        if (is_enum_type(*fsib))
            break;

        if (fsib == fpred) {
            fresh.erase(fsib++);
            fresh.erase(fsib++);
            break;
        }

        fresh.erase(fsib++);
        fresh.erase(fsib++);
    }
cout<<"duuude resh exemplar="<<fresh<<endl;

    _exemplars.clear();
    _exemplars.push_back(fresh);

cout<<endl;
    return true;
}

/// Recurse, and try to weed out a few more cases.
bool partial_solver::recurse()
{
    if (1 < _fail_recurse_count)
        return false;

    std::vector<CTable> tabs;

    // Make a copy of the tables
    foreach(CTable& ctable, _ctables) {
        tabs.push_back(CTable(ctable));
    }

    // Remove unwanted rows from the copy
    unsigned total_rows = 0;
    unsigned deleted = 0;
    trim_table(tabs, _best_fail_pred, deleted, total_rows);
cout<<"duude before recusion, deleted="<< deleted<<" out of="<<total_rows<<endl;

    // And lets try to find a predicate that does the trick.
    combo_tree tr(id::cond);
    tr.append_child(tr.begin(), enum_t::get_random_enum());
    vector<combo_tree> exempls;
    exempls.push_back(tr);
    partial_solver ps(tabs, 
                      _alf_sz, _noise, _table_type_signature,
                      exempls, _reduct, _opt_params, _meta_params,
                      _moses_params, _printer);

cout<<"duuu vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"<<endl;
    ps._fail_recurse_count++;
    ps.solve();
cout<<"duu ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;

    return false;
}

};};
