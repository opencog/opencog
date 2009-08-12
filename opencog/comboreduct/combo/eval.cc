/*
 * opencog/comboreduct/combo/eval.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
 *            Moshe Looks
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
#include "eval.h"

namespace combo
{

void set_bindings(combo_tree& tr, combo_tree::iterator it,
                  const std::vector<vertex>& args, arity_t explicit_arity)
{
    combo_tree::iterator end = it;
    end.skip_children();
    ++end;
    arity_t implicit_idx = explicit_arity;
    arity_t ap_args = args.size();
    for (combo_tree::iterator at = it;at != end;++at) {
        if (at.is_childless()) {

            //if argument #idx then substitute it by
            //args[idx-1]
            if (is_argument(*at)) {
                argument& arg = get_argument(*at);
                if (arg.is_negated()) {
                    tr.append_child(at, *at);
                    *at = combo::id::logical_not;
                    at = at.begin();
                    arg.negate();
                }
                *at = args[arg.idx-1];
            } else {
                arity_t a = get_arity(*at);
                if (a != 0) {
                    arity_t ama = abs_min_arity(a);
                    arity_t rest_ap_arg = ap_args - implicit_idx;
                    if (ama <= rest_ap_arg) {
                        arity_t idx_bound;
                        if (a > 0) //that is the arity is fixed
                            idx_bound = implicit_idx + ama;
                        else //that is at uses arg_list
                            idx_bound = ap_args;
                        for (; implicit_idx < idx_bound; implicit_idx++)
                            tr.append_child(at, args[implicit_idx]);
                    } else { //raise an assert
                        std::stringstream ss;
                        ss << *at;
                        OC_ASSERT(false,
                                          "There is not enough arguments given"
                                          " in input, %s needs at least %d"
                                          " arguments and only %d are provided",
                                          ss.str().c_str(),
                                          static_cast<int>(ama),
                                          static_cast<int>(rest_ap_arg));
                    }
                }
            }
        }
    }
}

void set_bindings(combo_tree& tr, combo_tree::iterator it,
                  combo_tree::iterator arg_parent, arity_t explicit_arity)
{
    std::vector<combo_tree::iterator>
    args(boost::make_counting_iterator(arg_parent.begin()),
         boost::make_counting_iterator(arg_parent.end()));
    combo_tree::iterator end = it;
    end.skip_children();
    ++end;
    arity_t implicit_idx = explicit_arity;
    arity_t ap_args = args.size();
    for (combo_tree::iterator at = it;at != end;++at) {
        if (at.is_childless()) {
            //if argument #idx then substitute it by
            //args[idx-1]
            if (is_argument(*at)) {
                argument& arg = get_argument(*at);
                if (arg.is_negated()) {
                    tr.append_child(at, *at);
                    *at = id::logical_not;
                    at = at.begin();
                    arg.negate();
                }
                combo_tree tmp(args[arg.idx-1]);
                at = tr.move_ontop(at, tmp.begin());
                at.skip_children();
            } else {
                arity_t a = get_arity(*at);
                if (a != 0) {
                    arity_t ama = abs_min_arity(a);
                    arity_t rest_ap_arg = ap_args - implicit_idx;
                    if (ama <= rest_ap_arg) {
                        arity_t idx_bound;
                        if (a > 0) //that is the arity is fixed
                            idx_bound = implicit_idx + ama;
                        else //that is at uses arg_list
                            idx_bound = ap_args;
                        for (; implicit_idx < idx_bound; implicit_idx++) {
                            combo_tree tmp(args[implicit_idx]);
                            tr.move_ontop(tr.append_child(at), tmp.begin());
                        }
                        at.skip_children();
                    } else { //raise an assert
                        std::stringstream ss;
                        ss << *at;
                        OC_ASSERT(false,
                                          "There is not enough arguments given"
                                          " in input, %s needs at least %d"
                                          " arguments and only %d are provided",
                                          ss.str().c_str(),
                                          static_cast<int>(ama),
                                          static_cast<int>(rest_ap_arg));
                    }
                }
            }
        }
    }
}

void set_bindings(combo_tree& tr, const std::vector<vertex>& args)
{
    if (!tr.empty())
        set_bindings(tr, tr.begin(), args, explicit_arity(tr));
}

void set_bindings(combo_tree& tr, combo_tree::iterator arg_parent)
{
    if (!tr.empty())
        set_bindings(tr, tr.begin(), arg_parent, explicit_arity(tr));
}

truth_table::size_type
truth_table::hamming_distance(const truth_table& other) const
{
    OC_ASSERT(other.size() == size(),
                      "truth_tables size should be the same.");

    size_type res = 0;
    for (const_iterator x = begin(), y = other.begin();x != end();)
        res += (*x++ != *y++);
    return res;
}

RndNumTable::RndNumTable(int sample_count, int arity, opencog::RandGen& rng, 
                         double max_randvalue , double min_randvalue )
{
    //populate the matrix
    for (int i = 0; i < sample_count; ++i) {
        contin_vector cv;
        for (int j = 0; j < arity; ++j)
        //   cv.push_back(rng.randdouble()*2.0 - 1.0); //TODO : rescale wrt
        cv.push_back((max_randvalue - min_randvalue) * rng.randdouble() + min_randvalue); 
        // input interval
        push_back(cv);
    }
}

contin_table::contin_table(const combo_tree& t, const RndNumTable& rnt, opencog::RandGen& rng)
{
    for (const_cm_it i = rnt.begin(); i != rnt.end(); ++i) {
        int arg = 1;
        for (const_cv_it j = (*i).begin(); j != (*i).end(); ++j, ++arg)
            binding(arg) = *j;
        //assumption : all inputs of t are contin_t
        vertex res = eval_throws(rng, t);
        //assumption : res is contin_t
        OC_ASSERT(is_contin(res),
                          "vertex isn't contin (contin_table)");
        push_back(get_contin(res));
    }
}

bool contin_table::operator==(const contin_table& ct) const
{
    if (ct.size() == size()) {
        const_cv_it ct_i = ct.begin();
        for (const_cv_it i = begin(); i != end(); ++i, ++ct_i) {
            if (!isApproxEq(*i, *ct_i))
                return false;
        }
        return true;
    } else return false;
}


contin_t contin_table::abs_distance(const contin_table& other) const
{
    OC_ASSERT(other.size() == size(),
                      "contin_tables should have the same size.");

    contin_t res = 0;
    for (const_iterator x = begin(), y = other.begin();x != end();)
        res += fabs(*x++ -*y++);
    return res;
}

} //~namespace combo
