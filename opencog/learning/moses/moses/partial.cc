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
                               const type_tree& table_tt,
                               const vector<combo_tree>& exemplars,
                               const rule& reduct,
                               const optim_parameters& opt_params,
                               const metapop_parameters& meta_params,
                               const moses_parameters& moses_params,
                               const metapop_printer& mmr_pa)

    :_ctables(ctables),
     _orig_ctables(ctables),
     _table_type_signature(table_tt),
     _exemplars(exemplars), _leader(id::cond),
     _reduct(reduct),
     _opt_params(opt_params), _meta_params(meta_params),
     _moses_params(moses_params), _printer(mmr_pa),
     _bscore(NULL), _done(false), _print(false)

#ifdef TRY_DOING_RECURSION
     _fail_recurse_count(0), _best_fail_ratio(1.0e30),
     _best_fail_tree(combo_tree()),
     _best_fail_pred(_best_fail_tree.end())
#endif
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
        score_seq.push_back(new BScore(ctable));

        tab_sz += ctable.uncompressed_size();
    }
    _bscore = new multibscore_based_bscore<BScore>(score_seq);

#define FRACTION 0.5
    // XXX this is wrong, should be delta on best_possible score.
    _bad_score = - floor(FRACTION * score_t(tab_sz));

    unsigned loop_count = 0;
    while(1) {
        if (_moses_params.max_evals <= _num_evals) break; 

        _opt_params.terminate_if_gte = _bad_score;
        _moses_params.max_score = _bad_score;
        _moses_params.max_evals -= _num_evals;
        _moses_params.max_gens -= _num_gens; // XXX wrong

        logger().info() << "well-enough start loop " << loop_count++
                        << " ask for=" << _bad_score
                        << " num_evals so far=" << _num_evals
                        << " max_evals= " << _moses_params.max_evals;

        metapop_moses_results(_exemplars, _table_type_signature,
                              _reduct, _reduct, *_bscore,
                              _opt_params, _meta_params, _moses_params,
                              *this);

        // If done, one more time, but only to invoke the printer.
        if (_done) {
            _print = true;
            _opt_params.terminate_if_gte = _bad_score;
            _moses_params.max_score = _bad_score;

            // This should cause the metapop to terminate immediately,
            // doing nothing other than to invoke the final score printer.
            _moses_params.max_evals = 0;
            _moses_params.max_gens = 0;

            logger().info() << "well-enough DONE!";
            metapop_moses_results(_exemplars, _table_type_signature,
                                  _reduct, _reduct, *_bscore,
                                  _opt_params, _meta_params, _moses_params,
                                  *this);

            break;
        }
    }
    logger().info() << "well-enough good-bye!";
}

/// Evaluate each of the candidate solutions, see if any of the leading
/// predicates are "good enough".  If we find one that is, then trim
/// the scoring tables, and return (so as to run moses on the smaller
/// problem).
void partial_solver::candidates(const metapop_candidates& cands)
{
    logger().info() << "well-enough found " << cands.size() << " candidates";
    foreach(auto &item, cands) {
        const combo_tree& cand = item.first;
        if (candidate(cand)) {
            refresh(cands, cand);
            return;
        }
    }

#ifdef TRY_DOING_RECURSION
    // XXX disable recursion for now, its actually harder than it seems.
    if (_best_fail_ratio < 0.2) {
        if (recurse()) return;
    }
#endif

    // If we are here, then none of the candidates were any good.
    // Try again, priming the metapop with the previous best.
    _exemplars.clear();
    _exemplars = _fresh_exemplars;  // copy them.
    _fresh_exemplars.clear();

    foreach(auto &item, cands) {
        const combo_tree& cand = item.first;
        _exemplars.push_back(cand);
    }

    logger().info() << "well-enough no acceptable candidates.  "
        "Try again, asking for score = " << _bad_score;
}

/// Final cleanup, before termination.
///
/// We've run out of time. So assemble the best possible exemplars out
/// of the pieces we've accumulated, and feed those back into the main
/// algo as exemplars.  The main algo will realize that it's out of time,
/// it will just score these, print them, and then all is done.
void partial_solver::final_cleanup(const metapop_candidates& cands)
{
    logger().info() << "well-enough ending with " << cands.size()
                    << " exemplars. Prefix=" << _leader;

    // Prefix the leading clauses in front of the best candidates,
    // and feed them back in as exemplars, for scoring.
    _exemplars.clear();
    foreach(auto &item, cands) {
        combo_tree cand = item.first;
        sib_it ldr = _leader.begin();
        sib_it cit = cand.begin();
        cit = cit.begin();
        for (sib_it lit = ldr.begin(); lit != ldr.end(); lit++) {
            cit = cand.insert_subtree(cit, lit);
            cit++;
        }
        _reduct(cand);
        _exemplars.push_back(cand);
    }

    // Recreate the original scoring tables, too.
    score_seq.clear();
    foreach(const CTable& ctable, _orig_ctables) {
        // FYI, no mem-leak, as ptr_vector seems to call delete.
        // XXX !?!?! relly?  as we've got a lifetime problem here.
        score_seq.push_back(new BScore(ctable));
    }
    delete _bscore;
    _bscore = new multibscore_based_bscore<BScore>(score_seq);
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

    unsigned total_count = 0;
    // Count how many items the first predicate mis-identifies.
    foreach(CTable& ctable, _ctables) {
        for (CTable::iterator cit = ctable.begin(); cit != ctable.end(); cit++) {
            const vertex_seq& vs = cit->first;
            const CTable::counter_t& c = cit->second;

            total_count += c.total_count();
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
    logger().info() << "well-enough leading predicate fail=" << fail_count
                    << " good=" << good_count
                    << " out of total=" << total_count;
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
    // Are we done yet?
    penalized_behavioral_score pbs = _bscore->operator()(cand);
    score_t total_score = 0.0;
    foreach(const score_t& sc, pbs.first)
        total_score += sc;

    logger().info() << "well-enough candidate=" << total_score
                    << " " << cand;
    // XXX replace  by the correct compare, i.e. the orig gte.
    if (-0.05 <= total_score) {
        _done = true;
        return true;
    }

    // Next time around, we need to beat this best score.
    if (_bad_score <= total_score)
        _bad_score = ceil (0.9*total_score + 1.0);

    // We don't want constants; what else we got?
    pre_it it = cand.begin();
    if (is_enum_type(*it)) {
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
            logger().info() << "well-enough reject constant preceeded by ineffective clause";
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
        if ((0 < good_count) || (0 < fail_count))
            break;
    }

    if (fail_count) {

        // Yes, this can happen ... a clause that fails, and gets
        // nothing right.
        if (0 == good_count) {
            combo_tree fresh(cand);
            pre_it fit = fresh.begin();
            sib_it fsib = fit.begin();
            fresh.erase(fsib++);
            fresh.erase(fsib++);
            _fresh_exemplars.push_back(fresh);
        }

#ifdef TRY_DOING_RECURSION
        // If the fail count isn't zero, at least try to find the most
        // graceful winner.
        double fail_ratio = double(fail_count) / double(good_count);
        if (fail_ratio < _best_fail_ratio) {
            _best_fail_ratio = fail_ratio;
            _best_fail_tree = cand;
            _best_fail_pred = predicate;
        }
#endif
        return false;
    }

    // If we are here, the first predicate is correctly identifying
    // all cases; we can save it, and remove all table elements that
    // it correctly identified.
    unsigned total_rows = 0;
    unsigned deleted = 0;
    trim_table(_ctables, predicate, deleted, total_rows);
    logger().info() << "well-enough deleted " << deleted
                    << " rows out of total " << total_rows;

    // Save the predicate itself
    sib_it ldr = _leader.begin();
    _leader.insert_subtree(ldr.end(), predicate);
    _leader.insert_subtree(ldr.end(), ++predicate);
    logger().info() << "well-enough prefix=" << _leader;

    // XXX replace 0.5 by parameter.
    // XXX should be, ummm, less than the number of a single type in the table,
    // else a constant beats this score... err, and weighted too!?
    _bad_score = -floor(0.45 * (score_t(total_rows) - score_t(deleted)));

    // Redo the scoring tables, as they cache the score tables (why?)
    score_seq.clear();
    foreach(const CTable& ctable, _ctables) {
        // FYI, no mem-leak, as ptr_vector seems to call delete.
        // XXX !?!?! relly?  as we've got a lifetime problem here.
        score_seq.push_back(new BScore(ctable));
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

        // If we are here, the predicate was ineffective, so erase it.
        fresh.erase(fsib++);
        fresh.erase(fsib++);
    }
    logger().info() << "well-enough fresh exemplar=" << fresh;

    _exemplars.clear();
    _exemplars.push_back(fresh);

    return true;
}

#ifdef TRY_DOING_RECURSION
/// Recurse, and try to weed out a few more cases.
// XXX this is definitely not working right now.
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
                      _table_type_signature,
                      exempls, _reduct, _opt_params, _meta_params,
                      _moses_params, _printer);

cout<<"duuu vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"<<endl;
    ps._fail_recurse_count++;
    ps.solve();
cout<<"duu ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;

    return false;
}
#endif

};};
