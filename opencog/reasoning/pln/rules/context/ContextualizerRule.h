/** ContextualizerRule.h --- 
 *
 * Copyright (C) 2010 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@laptop>
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


#ifndef _OPENCOG_CONTEXTUALIZERRULE_H
#define _OPENCOG_CONTEXTUALIZERRULE_H

#include "../GenericRule.h"
#include "../../formulas/Formulas.h"

namespace opencog { namespace pln {

/**
 * Rule to turn non contextual knowledge into Contextual knowledge
 *
 * More specifically apply the following inference
 * a)
 *
 * R <TV>
 *     C ANDLink A
 *     C ANDLink B
 * |-
 * ContextLink <TV>
 *     C
 *     R A B
 *
 * @todo this should be generalized for n-ari ANDLink
 *
 * b)
 *
 * SubsetLink <TV>
 *     C
 *     A
 * |-
 * ContextLink <TV>
 *     C
 *     A
 *
 * where A is a Node. that is because
 *
 * A <TV> is equivalent to
 *
 * SubsetLink <TV> Universe A
 *
 * the rest follows from a).
 */
class ContextualizerRule : public GenericRule<IdentityFormula>
{
protected:
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
    meta targetTemplate() const;
public:
    ContextualizerRule(AtomSpaceWrapper* _asw);
    meta i2oType(const std::vector<Vertex>& h) const;
    bool validate2(MPs& args) const { return true; } // not sure it's enough
    TruthValue** formatTVarray(const std::vector<Vertex>& premiseArray,
                               int* newN) const;
};

}} // namespace opencog { namespace pln {

#endif // _OPENCOG_CONTEXTUALIZERRULE_H
