/*
 * opencog/learning/moses/representation/instance_scorer.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012,2013 Poulin Holdings LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks, Nil Geisweiller, Linas Vepstas
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
#ifndef _INSTANCE_SCORER_H
#define _INSTANCE_SCORER_H

#include "instance.h"
#include "field_set.h"
#include "representation.h"
#include "../scoring/behave_cscore.h"

namespace opencog { namespace moses {

struct iscorer_base : public std::unary_function<instance, composite_score>
{
    virtual composite_score operator()(const instance&) const = 0;
    virtual ~iscorer_base() {}
};

/**
 * Mostly for testing the optimization algos.  Returns minus the
 * hamming distance of the candidate to a given target instance and
 * constant null complexity.
 */
struct distance_based_scorer : public iscorer_base
{
    distance_based_scorer(const field_set& _fs,
                          const instance& _target_inst)
        : fs(_fs), target_inst(_target_inst) {}

    composite_score operator()(const instance& inst) const
    {
        score_t sc = -fs.hamming_distance(target_inst, inst);
        // Logger
        if (logger().isFineEnabled()) {
            logger().fine() << "distance_based_scorer - Evaluate instance: "
                            << fs.to_string(inst) << "\n"
                            << "Score = " << sc << std::endl;
        }
        // ~Logger
        return composite_score(sc, 0, 0, 0);
    }

protected:
    const field_set& fs;
    const instance& target_inst;
};

struct complexity_based_scorer : public iscorer_base
{
    complexity_based_scorer(behave_cscore& cs,
                            representation& rep, bool reduce)
        : _cscorer(cs), _rep(rep), _reduce(reduce) {}

    composite_score operator()(const instance& inst) const
    {
        if (logger().isFineEnabled()) {
            logger().fine() << "complexity_based_scorer - Evaluate instance: "
                            << _rep.fields().to_string(inst);
        }

        try {
            combo_tree tr = _rep.get_candidate(inst, _reduce);
            return _cscorer.get_cscore(tr);
        } catch (...) {
// XXX FIXME, calling score_tree above does not throw the exception; this should be done
// differntly, maybe call bscorer directly, then ascorer...
// ??? Huh? why couldn't we evaluate a tree anyway?  why would we want an exception here?
            combo_tree raw_tr = _rep.get_candidate(inst, false);
            combo_tree red_tr = _rep.get_candidate(inst, true);
            logger().warn() << "The following instance could not be evaluated: "
                            << _rep.fields().to_string(inst)
                            << "\nUnreduced tree: " << raw_tr
                            << "\nreduced tree: "<< red_tr;
        }
        return worst_composite_score;
    }

protected:
    behave_cscore& _cscorer;
    representation& _rep;
    bool _reduce; // whether the exemplar should be reduced before being
                  // evaluated.  This is advantagous when _cscorer is
                  // also a cache; the reduced form will have more cache
                  // hits.
};

} //~namespace moses
} //~namespace opencog

#endif
