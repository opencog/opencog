/** ContextFreeToSensitiveRule.h --- 
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


#ifndef _OPENCOG_CONTEXTFREETOSENSITIVERULE_H
#define _OPENCOG_CONTEXTFREETOSENSITIVERULE_H

#include "../GenericRule.h"
#include "../../formulas/Formulas.h"

namespace opencog { namespace pln {

/**
 * Rule to contextualize an atom A given no prior assumption aboput A and C.
 *
 * Specifically apply the inference
 *
 * C <TV1>
 * A <TV2>
 * |-
 * ContextLink <TV3>
 *     C
 *     A
 *
 * where TV3 is determined by ContextFreeToSensitiveFormula
 *
 * Such a rule since it makes no assumption should be called in last
 * resort (after ContextualizerRule for instance) and therefore have a
 * low priority.
 *
 */
class ContextFreeToSensitiveRule : public GenericRule<ContextFreeToSensitiveFormula>
{
    typedef GenericRule<ContextFreeToSensitiveFormula> super;
protected:
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
    meta targetTemplate() const;
public:
    ContextFreeToSensitiveRule(AtomSpaceWrapper* _asw);
    meta i2oType(const VertexSeq& h) const;
    bool validate2(MPs& args) const { return true; } // not sure it's enough
    TVSeq formatTVarray(const VertexSeq& premiseArray) const;
};

}} // namespace opencog { namespace pln {

#endif // _OPENCOG_CONTEXTFREETOSENSITIVENODERULE_H
