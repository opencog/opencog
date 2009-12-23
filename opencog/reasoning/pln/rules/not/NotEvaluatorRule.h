/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
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

#ifndef NOTEVALUATORRULE_H
#define NOTEVALUATORRULE_H

#include "../GenericRule.h"

namespace opencog { namespace pln {

class NotEvaluatorRule : public GenericRule<NotFormula>
{
protected:
    TruthValue** formatTVarray(const std::vector<Vertex>& premiseArray,
                               int* newN) const {
        TruthValue** tvs = new TruthValue*[2];

        const int N = (int)premiseArray.size();
        assert(N == 1);

        //std::vector<Handle> real = premiseArray[0];
#if 0
        // Welter's comment: this change is waiting for Ari's aproval
        tvs[0] = (TruthValue*) & (TruthValue::TRIVIAL_TV());
#else
        tvs[0] = new SimpleTruthValue(0, 0); //nm->getTV(premiseArray[0]);
        //! @todo create the TrivialTV to use here
#endif
        tvs[1] = (TruthValue*) 
            & (asw->getTV(boost::get<pHandle>(premiseArray[0])));
        return tvs;
    }

    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

public:
    NotEvaluatorRule(AtomSpaceWrapper *_asw);
    meta i2oType(const std::vector<Vertex>& h) const;

    bool validate2(MPs& args) const {
        return true;
    }
    NO_DIRECT_PRODUCTION;
};

}} // namespace opencog { namespace pln {
#endif // NOTEVALUATORRULE_H
