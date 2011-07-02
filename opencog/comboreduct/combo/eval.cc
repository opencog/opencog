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

namespace opencog { namespace combo {

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
                    *at = id::logical_not;
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
                        for (; implicit_idx < idx_bound; ++implicit_idx)
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
                        for (; implicit_idx < idx_bound; ++implicit_idx) {
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

}} // ~namespaces combo opencog
