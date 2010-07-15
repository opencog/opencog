/*
 * opencog/comboreduct/reduct/meta_rules.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Nil Geisweiller
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

#include <opencog/comboreduct/combo/type_tree.h>
#include "reduct.h"

namespace reduct {

using std::string;

// apply rule r_ only when cond_ is true
struct when : public crule<when> {
    when(const rule& r_, bool cond_, string name = "when")
        : crule<when>::crule(name), r(r_.clone()),
          cond(cond_) {}
    when(const when& w)
        : crule<when>::crule(w.get_name()),
          r(w.r->clone()),cond(w.cond) { }
    void operator()(combo_tree&,combo_tree::iterator) const;

protected:
    shared_ptr<const rule> r;
    bool cond;
};      

// if applying rule r_ increases the size of the combo_tree then ignore it
struct ignore_size_increase : public crule<ignore_size_increase> {
    explicit ignore_size_increase(const rule& r_,
                                  string name = "ignore_size_increase")
        : crule<ignore_size_increase>::crule(name),
          r(r_.clone()) {}
    ignore_size_increase(const ignore_size_increase& i)
        : crule<ignore_size_increase>::crule(i.get_name()), r(i.r->clone()) { }
    void operator()(combo_tree&,combo_tree::iterator) const;

protected:
    shared_ptr<const rule> r;
};      

//apply rule in pre-order (left-to-right, parents before children, leftward
//subtrees before rightward subtrees) from a given node, to visit all
//children in the given iterator's subtree (e.g., the node the iterator
//points at gets visited first)
struct downwards : public crule<downwards> {
    explicit downwards(const rule& r_, string name = "downwards")
        : crule<downwards>::crule(name), r(r_.clone()),
          input(combo::id::unknown_type), output(combo::id::unknown_type) { }
    downwards(const rule& r_,combo::type_node t, string name = "downwards")
        : crule<downwards>::crule(name),
          r(r_.clone()), input(t), output(t) { }
    downwards(const rule& r_,combo::type_node input_,combo::type_node output_, 
              string name = "downwards")
        : crule<downwards>::crule(name),
          r(r_.clone()),input(input_),output(output_) { }
    downwards(const downwards& d)
        : crule<downwards>::crule(d.get_name()),
          r(d.r->clone()),input(d.input),output(d.output) { }

    void operator()(combo_tree&,combo_tree::iterator) const;

protected:
    shared_ptr<const rule> r;
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

    void operator()(combo_tree&,combo_tree::iterator) const;

protected:
    shared_ptr<const rule> r;
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
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
protected:
    shared_ptr<const rule> r;
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
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
protected:
    shared_ptr<const rule> r;
};

//apply rules sequentially to a particular point-of-application
//overloaded up to 50 arguments.
struct sequential : public crule<sequential> {
    //sequential() { }
    sequential(const sequential& rhs) 
        : crule<sequential>::crule(rhs.get_name()),
          rules(rhs.rules.begin(), rhs.rules.end()) { }
    sequential(const rule& r1, string name = "sequential") : crule<sequential>::crule(name)
    {
        rules.push_back(r1.clone()); 
    }
    sequential(const rule& r1,const rule& r2, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42,const rule& r43, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
        rules.push_back(r43.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42,const rule& r43,const rule& r44, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
        rules.push_back(r43.clone()); rules.push_back(r44.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42,const rule& r43,const rule& r44,
               const rule& r45, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
        rules.push_back(r43.clone()); rules.push_back(r44.clone()); 
        rules.push_back(r45.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42,const rule& r43,const rule& r44,
               const rule& r45,const rule& r46, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
        rules.push_back(r43.clone()); rules.push_back(r44.clone()); 
        rules.push_back(r45.clone()); rules.push_back(r46.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42,const rule& r43,const rule& r44,
               const rule& r45,const rule& r46,const rule& r47, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
        rules.push_back(r43.clone()); rules.push_back(r44.clone()); 
        rules.push_back(r45.clone()); rules.push_back(r46.clone()); 
        rules.push_back(r47.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42,const rule& r43,const rule& r44,
               const rule& r45,const rule& r46,const rule& r47,const rule& r48, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
        rules.push_back(r43.clone()); rules.push_back(r44.clone()); 
        rules.push_back(r45.clone()); rules.push_back(r46.clone()); 
        rules.push_back(r47.clone()); rules.push_back(r48.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42,const rule& r43,const rule& r44,
               const rule& r45,const rule& r46,const rule& r47,const rule& r48,
               const rule& r49, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
        rules.push_back(r43.clone()); rules.push_back(r44.clone()); 
        rules.push_back(r45.clone()); rules.push_back(r46.clone()); 
        rules.push_back(r47.clone()); rules.push_back(r48.clone()); 
        rules.push_back(r49.clone()); 
    }
    sequential(const rule& r1,const rule& r2,const rule& r3,const rule& r4,
               const rule& r5,const rule& r6,const rule& r7,const rule& r8,
               const rule& r9,const rule& r10,const rule& r11,const rule& r12,
               const rule& r13,const rule& r14,const rule& r15,const rule& r16,
               const rule& r17,const rule& r18,const rule& r19,const rule& r20,
               const rule& r21,const rule& r22,const rule& r23,const rule& r24,
               const rule& r25,const rule& r26,const rule& r27,const rule& r28,
               const rule& r29,const rule& r30,const rule& r31,const rule& r32,
               const rule& r33,const rule& r34,const rule& r35,const rule& r36,
               const rule& r37,const rule& r38,const rule& r39,const rule& r40,
               const rule& r41,const rule& r42,const rule& r43,const rule& r44,
               const rule& r45,const rule& r46,const rule& r47,const rule& r48,
               const rule& r49,const rule& r50, string name = "sequential") : crule<sequential>::crule(name) {
        rules.push_back(r1.clone()); rules.push_back(r2.clone()); 
        rules.push_back(r3.clone()); rules.push_back(r4.clone()); 
        rules.push_back(r5.clone()); rules.push_back(r6.clone()); 
        rules.push_back(r7.clone()); rules.push_back(r8.clone()); 
        rules.push_back(r9.clone()); rules.push_back(r10.clone()); 
        rules.push_back(r11.clone()); rules.push_back(r12.clone()); 
        rules.push_back(r13.clone()); rules.push_back(r14.clone()); 
        rules.push_back(r15.clone()); rules.push_back(r16.clone()); 
        rules.push_back(r17.clone()); rules.push_back(r18.clone()); 
        rules.push_back(r19.clone()); rules.push_back(r20.clone()); 
        rules.push_back(r21.clone()); rules.push_back(r22.clone()); 
        rules.push_back(r23.clone()); rules.push_back(r24.clone()); 
        rules.push_back(r25.clone()); rules.push_back(r26.clone()); 
        rules.push_back(r27.clone()); rules.push_back(r28.clone()); 
        rules.push_back(r29.clone()); rules.push_back(r30.clone()); 
        rules.push_back(r31.clone()); rules.push_back(r32.clone()); 
        rules.push_back(r33.clone()); rules.push_back(r34.clone()); 
        rules.push_back(r35.clone()); rules.push_back(r36.clone()); 
        rules.push_back(r37.clone()); rules.push_back(r38.clone()); 
        rules.push_back(r39.clone()); rules.push_back(r40.clone()); 
        rules.push_back(r41.clone()); rules.push_back(r42.clone()); 
        rules.push_back(r43.clone()); rules.push_back(r44.clone()); 
        rules.push_back(r45.clone()); rules.push_back(r46.clone()); 
        rules.push_back(r47.clone()); rules.push_back(r48.clone()); 
        rules.push_back(r49.clone()); rules.push_back(r50.clone()); 
    }

    void operator()(combo_tree& tr,combo_tree::iterator it) const;
    ptr_vector<rule> rules;
};

} //~namespace reduct

#endif
