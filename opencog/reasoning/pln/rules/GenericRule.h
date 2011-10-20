/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#ifndef GENERICRULE_H
#define GENERICRULE_H

#include "Rule.h"
#include "../formulas/Formula.h"

namespace opencog { namespace pln {

template<typename FormulaType>
class GenericRule : public Rule
{
protected:
    mutable FormulaType formula;

public:
    /**                                
     * Unlike o2iMetaExtra, i2oType is used on the way back
     * up the BIT (i.e. once it has reached existing atoms,
     * and is evaluating Rules on the path back up to the target).
     */
    virtual meta i2oType(const VertexSeq& h) const = 0;

    /**
     * formatTVarray generates the array of TVs to be passed to formula.compute
     * given the permises.
     * 
     * @param premiseArray the array of premises
     * @param newN the number of TVs output
     *
     * @return the array of TVs to be used by formula.compute
     */
    virtual TVSeq formatTVarray(const VertexSeq& premiseArray) const = 0;

    /// Always a composer
    GenericRule(AtomSpaceWrapper *_asw, bool _FreeInputArity,
                std::string _name = "")
        : Rule(_asw, _FreeInputArity, true, _name) { }

    BoundVertex compute(const VertexSeq& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const {
        const int n = (const int) premiseArray.size();

        cprintf(-3, "<Generic rule args> ");
        for (int j = 0;j < n;j++) {
            const pHandle *ph = boost::get<pHandle>(&premiseArray[j]);
            cprintf(-3, "[%u] ", *ph);
            //printTree(premiseArray[j],0,3);
        }
        cprintf(-3, " </Generic rule args>\n");

        cprintf(-3, "validate()");
        OC_ASSERT(validate(premiseArray));
        cprintf(-3, "/ validate()");

        cprintf(-3, "formatTVarray...\n");

        TVSeq tvs = formatTVarray(premiseArray);
        OC_ASSERT((int)tvs.size() <= formula.TVN);

        cprintf(-3, "formatTVarray OK\n");

        cprintf(-3, "Computing TV... \n");
        //! @todo Maybe fill in the Universe size.
        TruthValue* retTV = formula.compute(tvs);

        cprintf(-3, "TV computation ok\n");

        /// i2otype gives the atom skeleton (tree) w/o TV. addAtom inserts into AtomSpace with TV
        pHandle ret = asw->addAtom(*i2oType(premiseArray), *retTV, fresh);
//   false);

        delete retTV;

        cprintf(-3, "Atom added.");

//  printTree(ret,0,3);

        return Vertex(ret);
    }

    // macro for attemptDirectProduction doing nothing
    // this is because a GenericRule is a composer
    NO_DIRECT_PRODUCTION;
};

}} // namespace opencog { namespace pln {
#endif // GENERICRULE_H

