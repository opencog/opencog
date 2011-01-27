/** InheritanceSubstRule.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_INHERITANCESUBSTRULE_H
#define _OPENCOG_INHERITANCESUBSTRULE_H

#include "BaseSubstRule.h"
#include "../../formulas/Formulas.h"

namespace opencog { namespace pln {

/**
 * Rule for substituting an atom A by an atom B inside an
 * expression given that A inherits from B
 *
 * InheritanceLink <TV1>
 *     A
 *     B
 * C[A] <TV2>
 * |-
 * C[A/B] <TV3>
 *
 * where TV3 is determined by InhSubstFormula
 */
class InheritanceSubstRule : public BaseSubstRule<InhSubstFormula> {
    typedef BaseSubstRule<InhSubstFormula> super;
public:
    /// If LByR is true then C[A] is replaced by C[A/B], but if it
    /// false then C[B] is replaced by C[B/A]
    InheritanceSubstRule(AtomSpaceWrapper* _asw, bool LByR = true)
        : super(_asw, INHERITANCE_LINK, LByR) {}
};

}} // namespace opencog { namespace pln {

#endif // _OPENCOG_INHERITANCESUBSTRULE_H
