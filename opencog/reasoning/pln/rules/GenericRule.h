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

#ifndef GENERICRULE_H
#define GENERICRULE_H

#include "Rule.h"

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
    virtual meta i2oType(const std::vector<Vertex>& h) const = 0;

    /**
     * formatTVarray generates the array of TVs to be passed to formula.compute
     * given the permises.
     * 
     * @param premiseArray the array of premises
     * @param newN the number of TVs output
     *
     * @return the array of TVs to be used by formula.compute
     */
    virtual TruthValue** formatTVarray(const std::vector<Vertex>& premiseArray,
                                       int* newN) const = 0;

    /// Always a composer
    GenericRule(AtomSpaceWrapper *_asw, bool _FreeInputArity,
                std::string _name = "")
        : Rule(_asw, _FreeInputArity, true, _name) { }

    BoundVertex compute(const std::vector<Vertex>& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const {
        const int n = (const int) premiseArray.size();

        printf("<Generic rule args> ");
        for (int j = 0;j < n;j++) {
            const pHandle *ph = boost::get<pHandle>(&premiseArray[j]);
            printf("[%u] ", *ph);
            //printTree(premiseArray[j],0,3);
        }
        printf(" </Generic rule args>\n");

        std::cout << "validate()" << std::endl;
        assert(validate(premiseArray));
        std::cout << "/ validate()" << std::endl;

        printf("formatTVarray...\n");
        int TVN = formula.TVN;
        TruthValue** tvs = formatTVarray(premiseArray, &TVN);
        printf("formatTVarray OK\n");

        if (!tvs) {
            printf("Warning only: GenericRule: TV array formatting failure.");
            return Vertex(PHANDLE_UNDEFINED);
        }

        printf("Computing TV... \n");
        TruthValue* retTV = formula.compute(tvs, TVN, fresh);
        printf("TV computation ok\n");

        delete[] tvs;
        printf("tvs[] freed.\n");

        /// i2otype gives the atom skeleton (tree) w/o TV. addAtom inserts into AtomSpace with TV
        pHandle ret = asw->addAtom(*i2oType(premiseArray), *retTV, true);
//   false);

        delete retTV;

        printf("Atom added.");

//  printTree(ret,0,3);

        return Vertex(ret);
    }

    // macro for attemptDirectProduction doing nothing
    // this is because a GenericRule is a composer
    NO_DIRECT_PRODUCTION;
};

}} // namespace opencog { namespace pln {
#endif // GENERICRULE_H

