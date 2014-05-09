/*
 * opencog/comboreduct/reduct/meta_rules.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Nil Geisweiller
 *            Adam Ehlers Nyholm Thomsen
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
#ifndef _META_RULES_H
#define _META_RULES_H

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <opencog/util/functional.h>

#include "../type_checker/type_tree.h"
#include "reduct.h"

namespace opencog { namespace reduct {

using std::string;

// apply rule r_ only when cond_ is true
struct when : public crule<when> {
    when(const rule& r_, bool cond_, string name = "when")
        : crule<when>::crule(name), r(r_.clone()),
          cond(cond_) {}
    when(const when& w)
        : crule<when>::crule(w.get_name()),
          r(w.r->clone()), cond(w.cond) { }
    void operator()(combo_tree&, combo_tree::iterator) const;

protected:
    std::shared_ptr<const rule> r;
    bool cond;
};

// if applying rule r_ does not decrease the size of the combo_tree
// then ignore it
struct ignore_size_increase : public crule<ignore_size_increase> {
    explicit ignore_size_increase(const rule& r_,
                                  string name = "ignore_size_increase")
        : crule<ignore_size_increase>::crule(name),
          r(r_.clone()) {}
    ignore_size_increase(const ignore_size_increase& i)
        : crule<ignore_size_increase>::crule(i.get_name()), r(i.r->clone()) { }
    void operator()(combo_tree&, combo_tree::iterator) const;

protected:
   std::shared_ptr<const rule> r;
};

//apply rule in pre-order (left-to-right, parents before children, leftward
//subtrees before rightward subtrees) from a given node, to visit all
//children in the given iterator's subtree (e.g., the node the iterator
//points at gets visited first)
struct downwards : public crule<downwards>
{
    explicit downwards(const rule& r_, string name = "downwards")
        : crule<downwards>::crule(name), r(r_.clone()),
          input(combo::id::unknown_type), output(combo::id::unknown_type) { }

    downwards(const rule& r_, combo::type_node t, string name = "downwards")
        : crule<downwards>::crule(name),
          r(r_.clone()), input(t), output(t) { }

    downwards(const rule& r_, combo::type_node input_, combo::type_node output_,
              string name = "downwards")
        : crule<downwards>::crule(name),
          r(r_.clone()), input(input_), output(output_) { }

    downwards(const downwards& d)
        : crule<downwards>::crule(d.get_name()),
          r(d.r->clone()), input(d.input), output(d.output) { }

    void operator()(combo_tree&, combo_tree::iterator) const;

protected:
    std::shared_ptr<const rule> r;
    combo::type_tree input;
    combo::type_node output;
};
//apply rule in post-order (left-to-right, children before parents,
//starting from the leftmost lowermost node) from a given node, to visit
//all children in the given iterator's subtree (e.g., the node the iterator
//points at gets visited last)
struct upwards : public crule<upwards> {
    explicit upwards(const rule& r_, string name = "upwards")
        : crule<upwards>::crule(name), r(r_.clone()) {}
    upwards(const upwards& u)
        : crule<upwards>::crule(u.get_name()), r(u.r->clone()) {}

    void operator()(combo_tree&, combo_tree::iterator) const;

protected:
    std::shared_ptr<const rule> r;
};

//apply a rule repeatedly to a point-of-application until the tree no
//longer changes
struct iterative : public crule<iterative> {
    iterative(string name = "iterative")
        : crule<iterative>::crule(name) {}
    explicit iterative(const rule& r_, string name = "iterative")
        : crule<iterative>::crule(name), r(r_.clone()) {}
    iterative(const iterative& i)
        : crule<iterative>::crule(i.get_name()), r(i.r->clone()) { }
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
protected:
    std::shared_ptr<const rule> r;
};

//like iterative but take into account the assumption set and is a bit slower
//if the assumption set changes assum_iterative keeps iterating
struct assum_iterative : public crule<assum_iterative> {
    assum_iterative(string name = "assum_iterative")
        : crule<assum_iterative>::crule(name) {}
    explicit assum_iterative(const rule& r_, string name = "assum_iterative") :
        crule<assum_iterative>::crule(name), r(r_.clone()) {}
    assum_iterative(const assum_iterative& i)
        : crule<assum_iterative>::crule(i.get_name()), r(i.r->clone()) { }
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
protected:
    std::shared_ptr<const rule> r;
};

//apply rules sequentially to a particular point-of-application
//overloaded up to 50 arguments.
struct sequential : public crule<sequential> {
    //sequential() { }
    sequential(const sequential& rhs)
        : crule<sequential>::crule(rhs.get_name()),
          rules(rhs.rules.begin(), rhs.rules.end()) { }

#define OC_RULES_PUSH_BACK(z, n, name) rules.push_back(BOOST_PP_CAT(name, n).clone());

#define OC_SEQ_CONSTRUCTOR(z, n, unused)                                \
    sequential(const rule &r BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, const rule& r), string name = "sequential") \
        : crule<sequential>::crule(name)                                \
    {                                                                   \
        rules.push_back(r.clone());                                     \
        BOOST_PP_CAT(BOOST_PP_REPEAT_, z)(n, OC_RULES_PUSH_BACK, r)     \
    }

    BOOST_PP_REPEAT(50, OC_SEQ_CONSTRUCTOR, unused)

#undef OC_SEQ_CONSTRUCTOR
#undef OC_RULES_PUSH_BACK

    void operator()(combo_tree& tr, combo_tree::iterator it) const;
    boost::ptr_vector<rule> rules;
};

} // ~namespace reduct
} // ~namespace opencog

#endif
