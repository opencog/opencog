/*
 * SuRealPMCB.h
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#ifndef _OPENCOG_SUREAL_PMCB_H
#define _OPENCOG_SUREAL_PMCB_H

#include <unordered_map>

#include <opencog/atoms/base/Handle.h>
#include <opencog/query/InitiateSearchMixin.h>
#include <opencog/query/TermMatchMixin.h>
#include <opencog/query/SatisfyMixin.h>

namespace opencog
{
namespace nlp
{

/**
 * A PatternMatchCallback for Surface Realization.
 *
 * Override the neccessary callbacks to do special handling of variables
 * and LG dictionary checks.
 */
class SuRealPMCB :
    public InitiateSearchMixin,
    public TermMatchMixin,
    public SatisfyMixin
{
public:
    SuRealPMCB(AtomSpace* as, const HandleSet& vars, bool use_cache);
    ~SuRealPMCB();

    virtual bool variable_match(const Handle& hPat, const Handle& hSoln);
    virtual bool clause_match(const Handle& pattrn_link_h, const Handle& grnd_link_h);
    virtual bool propose_grounding(const HandleMap &var_soln,
                                   const HandleMap &pred_soln);
    virtual bool perform_search(PatternMatchCallback&);
    virtual void set_pattern(const Variables& vars,
                             const Pattern& pat)
    {
        InitiateSearchMixin::set_pattern(vars, pat);
        TermMatchMixin::set_pattern(vars, pat);
    }

    std::map<Handle, HandleMapSeq> m_results;   // store the PM results

private:
    virtual Handle find_starter_recursive(const PatternTermPtr&, size_t&, PatternTermPtr&, size_t&);
    bool disjunct_match(const Handle&, const Handle&);

    AtomSpace* m_as;
    bool m_use_cache;
    HandleSet m_vars;   // store nodes that are variables

    std::unordered_map<Handle, HandleSeq> m_disjuncts;   // store the disjuncts of nodes in the pattern

    std::unordered_map<Handle, Handle> m_words;   // store the corresponding WordNodes of the nodes in the pattern

    HandleSet m_interp;   // store a set of InterpretationNodes correspond to some clauses accepted in clause_match()
    HandleSet m_targets;   // store a set of target InterpretationNodes

    struct CandHandle
    {
        Handle handle;
        size_t r2lSetLinkSize;
    };
};

}
}

#endif // _OPENCOG_SUREAL_PMCB_H
