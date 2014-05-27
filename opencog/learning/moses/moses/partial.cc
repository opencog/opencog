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
#include "../scoring/scoring.h"

namespace opencog { namespace moses {

using namespace std;

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

partial_solver::partial_solver(const vector<CTable> &ctables,
                               const type_tree& table_tt,
                               const vector<combo_tree>& exemplars,
                               const rule& reduct,
                               const optim_parameters& opt_params,
                               const hc_parameters& hc_params,
                               const metapop_parameters& meta_params,
                               const moses_parameters& moses_params,
                               const metapop_printer& mmr_pa)

    :_ctables(ctables),
     _orig_ctables(ctables),
     _table_type_signature(table_tt),
     _exemplars(exemplars), _leader(id::cond),
     _prefix_count(0),
     _reduct(reduct),
     _opt_params(opt_params),
     _hc_params(hc_params),
     _meta_params(meta_params),
     _moses_params(moses_params), _printer(mmr_pa),
     _bscore(NULL), 
     _cscore(NULL), 
     _straight_bscore(NULL), 
     _straight_cscore(NULL), 
     _num_evals(0), _num_gens(0),
     _done(false),
     _most_good(0)
{
}

partial_solver::~partial_solver()
{
    delete _bscore;
    delete _straight_bscore;
    delete _cscore;
    delete _straight_cscore;
}

/// Implements the "leave well-enough alone" algorithm.
void partial_solver::solve()
{
    _num_gens = 0;
    _done = false;

    unsigned tab_sz = 0;
    score_seq.clear();
    for (const CTable& ctable : _ctables) {
        score_seq.push_back(new BScore(ctable));

        tab_sz += ctable.uncompressed_size();
    }
    _bscore = new multibscore_based_bscore(score_seq);
    _cscore = new multibehave_cscore(score_seq);

    _meta_params.merge_callback = check_candidates;
    _meta_params.callback_user_data = (void *) this;
    
    unsigned loop_count = 0;
    while(1) {
        OC_ASSERT(_moses_params.max_evals > _num_evals,
            "well-enough: I'm confused. What happened?");

        _moses_params.max_evals -= _num_evals;
        
        // XXX TODO: we need to get the actual number of gens run, back
        // from moses, and subtract it here.  But there's no easy way
        // to get this number ...
        _moses_params.max_gens -= _num_gens;

        logger().info() << "well-enough start loop " << loop_count++
                        << " previous num_evals=" << _num_evals
                        << " max_evals= " << _moses_params.max_evals
                        << " num exemplars=" << _exemplars.size();

        metapop_moses_results(_exemplars, _table_type_signature,
                              _reduct, _reduct, *_bscore, *_cscore,
                              _opt_params, _hc_params, _meta_params, _moses_params,
                              *this);

        // If done, then we run one last time, but only to invoke the
        // original printer.
        if (_done) {
            // This should cause the metapop to terminate immediately,
            // doing nothing other than to invoke the final score printer.
            _moses_params.max_evals = 0;
            _moses_params.max_gens = 0;

            logger().info() << "well-enough DONE!";
            metapop_moses_results(_exemplars, _table_type_signature,
                                  _reduct, _reduct, *_straight_bscore, *_straight_cscore,
                                  _opt_params, _hc_params, _meta_params, _moses_params,
                                  _printer);

            break;
        }
    }
    logger().info() << "well-enough good-bye!";
unsigned tcount = 0;
 for (CTable& ctable : _ctables) tcount += ctable.uncompressed_size();
cout<<"duuude end with prefix_count=" << _prefix_count <<" table_size=" << tcount << endl;
}

/// Evaluate each of the candidate solutions, see if any of the leading
/// predicates are "good enough".  If we find one that is, then trim
/// the scoring tables, and return (so as to run moses on the smaller
/// problem).
bool partial_solver::eval_candidates(const scored_combo_tree_set& cands)
{
    logger().info() << "well-enough received " << cands.size() << " candidates";
    _most_good = 0;
    for (auto &item : cands) {
        const combo_tree& cand = item.get_tree();
        eval_candidate(cand);
    }

    // If none of the candidates were any good, we are done.
    if (0 == _most_good)  return false;

    // Make note of the best one found, and restart the metapop.
    record_prefix();
    return true;   
}

/// Final cleanup, before termination.
///
/// We've run out of time. So assemble the best possible exemplars out
/// of the pieces we've accumulated, and feed those back into the main
/// algo as exemplars.  The main algo will realize that it's out of time,
/// it will just score these, print them, and then all is done.
void partial_solver::final_cleanup(const metapopulation& cands)
{
    logger().info() << "well-enough ending with " << cands.size()
                    << " exemplars. Prefix count= " << _prefix_count
                    << " prefix=" << _leader;

    // Prefix the leading clauses in front of the best candidates,
    // and feed them back in as exemplars, for scoring.
    _exemplars.clear();
    for (auto &item : cands) {
        combo_tree cand = item.get_tree();
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
    // This time, use the non-graded (flat) scorer, that simply counts
    // the number of right & wrong, without weighting.
    straight_score_seq.clear();
    for (const CTable& ctable : _orig_ctables)
        straight_score_seq.push_back(new StraightBScore(ctable));

    _straight_bscore = new multibscore_based_bscore(straight_score_seq);
    _straight_cscore = new multibehave_cscore(straight_score_seq);
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
    interpreter_visitor iv(predicate);
    auto interpret_predicate = boost::apply_visitor(iv);
    for (CTable& ctable : _ctables) {
        for (CTable::iterator cit = ctable.begin(); cit != ctable.end(); cit++) {
            vertex pr = interpret_predicate(cit->first.get_variant());
            const CTable::counter_t& c = cit->second;
            total_count += c.total_count();
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
    logger().debug() << "well-enough leading predicate fail=" << fail_count
                     << " good=" << good_count
                     << " out of total=" << total_count;
}


/// Remove all rows from the table that satisfy the predicate.
void partial_solver::trim_table(std::vector<CTable>& tabs,
                                const combo_tree::iterator predicate,
                                unsigned& deleted,   // return value
                                unsigned& total)    // return value

{
    interpreter_visitor iv(predicate);
    auto interpret_predicate = boost::apply_visitor(iv);
    for (CTable& ctable : tabs) {
        for (CTable::iterator cit = ctable.begin(); cit != ctable.end(); ) {
            vertex pr = interpret_predicate(cit->first.get_variant());
            const CTable::counter_t& c = cit->second;
            unsigned tc = c.total_count();
            total += tc;

            if (pr == id::logical_true) {
                deleted += tc;
                ctable.erase(cit++);
            }
            else cit++;
        }
    }
}

/// Refresh the exemplars list.  Basically, just copy the entire
/// metapopulation from the previous run.
void partial_solver::refresh(const metapopulation& cands)
{
    for (const auto &item : cands)
        _exemplars.push_back(item.get_tree());
}

/// Evaluate a single candidate
void partial_solver::eval_candidate (const combo_tree& cand)
{
    // Are we done yet?
    behavioral_score bs = _bscore->operator()(cand);

    // XXX is this correct? I think we need to ask the cscorer for the total ...
    score_t total_score = 0.0;
    for (const score_t& sc : bs)
        total_score += sc;

    logger().debug() << "well-enough candidate=" << total_score
                     << " " << cand;

    // We don't want constants; what else we got?
    pre_it it = cand.begin();
    if (is_enum_type(*it))
        return;

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
            logger().debug() << "well-enough reject constant preceeded by ineffective clause";
            return;
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

    // Predicate must have perfect accuracy to be acceptable to us.
    if (fail_count)
        return;

    // ... and it must be better than what we already know of.
    if (good_count <= _most_good)
        return;

    logger().info() << "well-enough found a good predicate good_count=" << good_count;
    // If we are here, we found something...
    _most_good = good_count;
    _best_predicate = predicate;
}

    // If we are here, the first predicate is correctly identifying
    // all cases; we can save it, and remove all table elements that
    // it correctly identified.
void partial_solver::record_prefix()
{
    unsigned total_rows = 0;
    unsigned deleted = 0;
    trim_table(_ctables, _best_predicate, deleted, total_rows);
    logger().info() << "well-enough deleted " << deleted
                    << " rows out of total " << total_rows;

    // Save the predicate itself
    _prefix_count++;
    sib_it ldr = _leader.begin();
    sib_it prd = _best_predicate;
    _leader.insert_subtree(ldr.end(), prd);
    _leader.insert_subtree(ldr.end(), ++prd);
    logger().info() << "well-enough " << _prefix_count
                    << " prefix=" << _leader;

    // Redo the scoring tables, as they cache the score tables (why?)
    score_seq.clear();
    for (const CTable& ctable : _ctables)
        score_seq.push_back(new BScore(ctable));

    delete _bscore;
    _bscore = new multibscore_based_bscore(score_seq);
    delete _cscore;
    _cscore = new multibehave_cscore(score_seq);
}


};};
