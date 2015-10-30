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


#include <opencog/query/DefaultPatternMatchCB.h>
#include <opencog/query/InitiateSearchCB.h>


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
    public InitiateSearchCB,
    public DefaultPatternMatchCB
{
public:
    SuRealPMCB(AtomSpace* as, const std::set<Handle>& vars);
    ~SuRealPMCB();

    virtual bool variable_match(const Handle& hPat, const Handle& hSoln);
    virtual bool clause_match(const Handle& pattrn_link_h, const Handle& grnd_link_h);
    virtual bool grounding(const std::map<Handle, Handle> &var_soln,
                           const std::map<Handle, Handle> &pred_soln);
    virtual bool initiate_search(PatternMatchEngine*);
    virtual void set_pattern(const Variables& vars,
                             const Pattern& pat)
    {
        InitiateSearchCB::set_pattern(vars, pat);
        DefaultPatternMatchCB::set_pattern(vars, pat);
    }

    std::map<Handle, std::vector<std::map<Handle, Handle> > > m_results;   // store the PM results

private:
    virtual Handle find_starter(const Handle&, size_t&, Handle&, size_t&);

    AtomSpace* m_as;
    std::set<Handle> m_vars;   // store nodes that are variables

    std::unordered_map<Handle, HandleSeq> m_disjuncts;   // store the disjuncts of nodes in the pattern

    std::unordered_map<Handle, Handle> m_words;   // store the corresponding WordNodes of the nodes in the pattern

    struct CandHandle
    {
        Handle handle;
        size_t r2lSetLinkSize;
    };
};

}
}

#endif // _OPENCOG_SUREAL_PMCB_H
