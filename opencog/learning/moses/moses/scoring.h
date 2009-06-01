/*
 * opencog/learning/moses/moses/scoring.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _MOSES_SCORING_H
#define _MOSES_SCORING_H

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/reduct/meta_rules.h>

#include "using.h"
#include "representation.h"
#include "types.h"
#include "ant_scoring.h"

#include <iostream>
#include <fstream>


namespace moses
{

#define NEG_INFINITY INT_MIN

typedef float fitness_t;


double information_theoretic_bits(const eda::field_set& fs);

struct logical_score : public unary_function<combo_tree, int> {
    template<typename Scoring>
    logical_score(const Scoring& score, int a, opencog::RandGen& _rng)
            : target(score, a, _rng), arity(a), rng(_rng) { }

    int operator()(const combo_tree& tr) const;

    combo::truth_table target;
    int arity;
    opencog::RandGen& rng;
};

struct logical_bscore : public unary_function<combo_tree, behavioral_score> {
    template<typename Scoring>
    logical_bscore(const Scoring& score, int a, opencog::RandGen& _rng)
            : target(score, a, _rng), arity(a), rng(_rng) { }

    behavioral_score operator()(const combo_tree& tr) const;

    combo::truth_table target;
    int arity;
    opencog::RandGen& rng;
};

struct contin_score : public unary_function<combo_tree, contin_t> {
    template<typename Scoring>
    contin_score(const Scoring& score,
                 const RndNumTable& r,
                 opencog::RandGen& _rng)
            : target(score, r), rands(r), rng(_rng) { }

    contin_t operator()(const combo_tree& tr) const;

    combo::contin_table target;
    RndNumTable rands;
    opencog::RandGen& rng;
};

struct contin_score_sqr : public unary_function<combo_tree,contin_t> {
    template<typename Scoring>
    contin_score_sqr(const Scoring& score,
                     const RndNumTable& r,
                     opencog::RandGen& _rng)
        : target(score,r),rands(r),rng(_rng) { }
    
    contin_score_sqr(const combo::contin_table& t,
                     const RndNumTable& r,
                     opencog::RandGen& _rng)
        : target(t),rands(r),rng(_rng) { }
    
    contin_t operator()(const combo_tree& tr) const;
    
    combo::contin_table target;
    RndNumTable rands;
    opencog::RandGen& rng;
};

struct contin_bscore : public unary_function<combo_tree, behavioral_score> {
    template<typename Scoring>
    contin_bscore(const Scoring& score,
                  const RndNumTable& r,
                  opencog::RandGen& _rng)
        : target(score, r), rands(r), rng(_rng) { }

    contin_bscore(const combo::contin_table& t,
                  const RndNumTable& r,
                  opencog::RandGen& _rng)
        : target(t), rands(r), rng(_rng) { }

    behavioral_score operator()(const combo_tree& tr) const;

    combo::contin_table target;
    RndNumTable rands;
    opencog::RandGen& rng;
};

template<typename Scoring>
struct complexity_based_scorer : public unary_function<eda::instance, tree_score> {
    complexity_based_scorer(const Scoring& s,
                            representation& rep,
                            opencog::RandGen& _rng)
            : score(s), _rep(&rep), rng(_rng) { }

    tree_score operator()(const eda::instance& inst) const {
        using namespace reduct;

        //cout << "top, got " << _rep->fields().stream(inst) << endl;
        _rep->transform(inst);
        combo_tree tr = _rep->exemplar();

        //reduct::apply_rule(reduct::downwards(reduct::remove_null_vertices()),tr);
        //if simplification of all trees is enabled, we should instead do
        //apply_rule(downwards(remove_null_vertices()),tr);
        //clean_and_full_reduce(tr);
        clean_and_full_reduce(tr, rng);

        //to maybe speed this up, we can score directly on the exemplar,
        //and have complexity(tr) ignore the null vertices

#ifdef DEBUG_INFO
        std::cout << "scoring " << tr << endl;
        /*std::cout << "scoring " << tr << " -> " << score(tr)
        << " " << complexity(tr.begin()) << std::endl;*/
#endif

        return tree_score(score(tr), complexity(tr.begin()));
    }

    Scoring score;
protected:
    representation* _rep;
    opencog::RandGen& rng;
};

template<typename Scoring>
struct count_based_scorer : public unary_function<eda::instance, tree_score> {
    count_based_scorer(const Scoring& s,
                       representation& rep,
                       int base_count,
                       opencog::RandGen& _rng)
            : score(s), _rep(&rep), _base_count(base_count), rng(_rng) { }

    tree_score operator()(const eda::instance& inst) const {
#ifdef DEBUG_INFO
        std::cout << "transforming " << _rep->fields().stream(inst) << std::endl;
#endif
        _rep->transform(inst);

        combo_tree tr;

        try {
            // tr=_rep->exemplar();       // otherwise dangling junctors and the scoring
            _rep->get_clean_exemplar(tr); // are not in accordance with each other  // PJ

        } catch (...) {
            std::cout << "get_clean_exemplar threw" << std::endl;
            return worst_possible_score;
        }

        // sequential(clean_reduction(),logical_reduction())(tr,tr.begin());
        // std::cout << "OK " << tr << std::endl;
        // reduct::clean_and_full_reduce(tr);
        // reduct::clean_reduce(tr);
        // reduct::contin_reduce(tr,rng);

        tree_score ts = tree_score(score(tr),
                                   -int(_rep->fields().count(inst))
                                   + _base_count);

#ifdef DEBUG_INFO
        std::cout << "OKK " << tr << std::endl;
        std::cout << "Score:" << ts << std::endl;
#endif
        return ts;
    }

    Scoring score;
protected:
    representation* _rep;
    int _base_count;
    opencog::RandGen& rng;

};

/**
 * return true if x dominates y
 *        false if y dominates x
 *        indeterminate otherwise
 */
tribool dominates(const behavioral_score& x, const behavioral_score& y);


/**
 * For all candidates c in [from, to), insert c in dst iff 
 * no element of dst dominates c
 */
//this may turn out to be too slow...
template<typename It, typename Set>
void merge_nondominating(It from, It to, Set& dst)
{
    for (;from != to;++from) {
        // std::cout << "ook " << std::distance(from,to) << std::endl; // PJ
        bool nondominated = true;
        for (typename Set::iterator it = dst.begin();it != dst.end();) {
            tribool dom = dominates(from->second, it->second);
            if (dom) {
                dst.erase(it++);
            } else if (!dom) {
                nondominated = false;
                break;
            } else {
                ++it;
            }
        }
        if (nondominated)
            dst.insert(*from);
    }
}


} //~namespace moses

#endif
