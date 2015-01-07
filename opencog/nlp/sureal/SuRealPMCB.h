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
class SuRealPMCB : public DefaultPatternMatchCB
{
public:
    SuRealPMCB(AtomSpace* as, std::set<Handle> vars);

    virtual bool variable_match(Handle& hPat, Handle& hSoln);
    virtual bool clause_match(Handle& pattrn_link_h, Handle& grnd_link_h);
    virtual bool grounding(const std::map<Handle, Handle> &var_soln,
                           const std::map<Handle, Handle> &pred_soln);

    std::map<Handle, std::map<Handle, Handle> > m_results;   // store the PM results

private:
    Handle find_starter(Handle, size_t&, Handle&, size_t&);

    std::set<Handle> m_vars;   // store nodes that are variables
};

}
}

#endif // _OPENCOG_SUREAL_PMCB_H
