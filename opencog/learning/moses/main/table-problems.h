/**
 * table-problems.h ---
 *
 * Copyright (C) 2013 Linas Vepstas
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

#ifndef _OPENCOG_MOSES_TABLE_PROBLEMS_H
#define _OPENCOG_MOSES_TABLE_PROBLEMS_H

#include "problem.h"

namespace opencog { namespace moses {

void register_table_problems();

class table_problem_base : public problem_base
{
protected:
    typedef boost::ptr_vector<bscore_base> BScorerSeq;

    void common_setup(problem_params&);
    void common_type_setup(problem_params&);

    // Input data for table-based problems.
    vector<Table> tables;
    vector<CTable> ctables;
    vector<string> ilabels;     // labels of the input table (table.itable)
    combo::arity_t arity;

    type_tree table_type_signature;
    type_tree table_output_tt;
    type_node table_output_tn;
    type_node output_type;
};

/// Find interesting predicates
class ip_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "ip"; }
        virtual const std::string description() const {
             return "Find interesting patterns"; }
        virtual void run(problem_params&);
};

/// Regression based on combo program using ann
class ann_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "ann-it"; }
        virtual const std::string description() const {
             return "ANN-based regression on input table"; }
        virtual void run(problem_params&);
};

// ==================================================================
/// precision-based scoring
// regression based on input table by maximizing precision (or negative
// predictive value), holding activation const.
class pre_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "pre"; }
        virtual const std::string description() const {
             return "Precision-Activation scoring"; }
        virtual void run(problem_params&);
};

// ==================================================================
/// precision-based scoring (maximizing number of conjunctions)
class pre_conj_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "pre-conj"; }
        virtual const std::string description() const {
             return "Precision-Conjunction-Maximization"; }
        virtual void run(problem_params&);
};

// ==================================================================
/// maximize precision, holding recall const.
class prerec_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "prerec"; }
        virtual const std::string description() const {
             return "Precision Maximization (holding recall constant)"; }
        virtual void run(problem_params&);
};

// ==================================================================
/// maximize recall, holding precision const.
class recall_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "recall"; }
        virtual const std::string description() const {
             return "Recall Maximization (holding precision constant)"; }
        virtual void run(problem_params&);
};

// ==================================================================
/// bep == beak-even point between bep and precision.
class bep_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "bep"; }
        virtual const std::string description() const {
             return "Maximize Break-even Point"; }
        virtual void run(problem_params&);
};

// ==================================================================
/// f_one = F_1 harmonic mean of recall and precision
class f_one_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "f_one"; }
        virtual const std::string description() const {
             return "Maximize F_1 score"; }
        virtual void run(problem_params&);
};

// ==================================================================
/// Maximize accuracy
class it_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "it"; }
        virtual const std::string description() const {
             return "Maximize Accuracy"; }
        virtual void run(problem_params&);
};

// ==================================================================
/// Perform clusterering
class cluster_table_problem : public table_problem_base
{
    public:
        virtual const std::string name() const { return "cluster"; }
        virtual const std::string description() const {
             return "Discover clustering function"; }
        virtual void run(problem_params&);
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_TABLE_PROBLEMS_H
