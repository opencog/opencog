/** ForAllInstantiationRule.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_FORALLINSTANTIATIONRULE_H
#define _OPENCOG_FORALLINSTANTIATIONRULE_H

#include "BaseInstantiationRule.h"
#include "../../formulas/Formulas.h"

namespace opencog { namespace pln {

static const std::string ForAllInstantiationRulePrefixStr = 
    "ForAllInstantiationRule";

/**
 * @todo the formula is not right
 */
class ForAllInstantiationRule : public BaseInstantiationRule<IdentityFormula> {
private:
    typedef BaseInstantiationRule<IdentityFormula> super;

public:
    //! Constructor for calling from add/remove signal handler
    ForAllInstantiationRule(Type t, pHandle _forAllLink, AtomSpaceWrapper *_asw) :
        super(t, _forAllLink, _asw, ForAllInstantiationRulePrefixStr) { }

    ForAllInstantiationRule(pHandle _forAllLink, AtomSpaceWrapper *_asw) :
        super(_forAllLink, _asw, ForAllInstantiationRulePrefixStr) { }
};

}} // namespace opencog { namespace pln {

#endif // _OPENCOG_FORALLINSTANTIATIONRULE_H
